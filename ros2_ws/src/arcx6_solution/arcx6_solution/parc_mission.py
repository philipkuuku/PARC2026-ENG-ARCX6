#!/usr/bin/env python3
"""PARC real-world mission node. Adds the scored-run flow ON TOP of terminal_nav.

It IS a TerminalNav (subclass), so every typed /command, the Groq LLM parser,
retries, close-enough arrival, TTS announcements and /task/status keep working.
Run this INSTEAD of terminal_nav (not both): it already contains it.

Flow
  1. node ready          -> publishes True on  /parc/robot_started   (latched)
  2. order arrives       <- /parc/tag_order   Int32MultiArray        (latched)
  3. robot waits for the English word "go" (and nothing else starts a run)
  4. for each tag in order: drive there, hold, photo ->
        /parc/tag_visited (Int32), /parc/tag_image (Image)
     a tag it cannot reach is skipped (brief: no going back), loop continues
  5. after the last tag  -> /parc/run_finished (String, JSON)

"go" can come from:  /speech/transcript (exact word only), /parc/start_word,
or typing  go  or  /go  in this terminal.  "go to tag 7" is NOT a start word:
it still goes to the LLM exactly as before.

All /parc/* names are DUMMIES until the organisers' status package is published.
They live in TOPICS below, so swapping them later is a one-place edit.
"""
import json
import re
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, Int32MultiArray, String

try:                                   # normal case: run as part of the package
    from .terminal_nav import HELP, TerminalNav
except ImportError:                    # running the file directly
    from terminal_nav import HELP, TerminalNav

# ----------------------------------------------------------------------
# Dummy topic contract (replace names/types when the status package arrives)
# ----------------------------------------------------------------------
TOPICS = {
    'robot_started': '/parc/robot_started',   # std_msgs/Bool         robot -> organiser
    'tag_order':     '/parc/tag_order',       # std_msgs/Int32MultiArray  organiser -> robot
    'start_word':    '/parc/start_word',      # std_msgs/String       offline voice node -> robot
    'run_started':   '/parc/run_started',     # std_msgs/Bool         robot -> organiser
    'tag_visited':   '/parc/tag_visited',     # std_msgs/Int32        robot -> organiser
    'tag_image':     '/parc/tag_image',       # sensor_msgs/Image     robot -> organiser
    'run_finished':  '/parc/run_finished',    # std_msgs/String JSON  robot -> organiser
}
# Latched: a node that starts later still receives the last value.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

START_WORD = 'go'          # English only; a second language earns nothing

MISSION_HELP = """Mission (PARC run):
  go  or  /go             start the run (needs the tag order first)
  /order N N N N          set the tag order by hand (dummy organiser)
  /run                    mission state, order, visited and skipped tags
  /stop                   abort the run (also: say "stop")"""

# Mission states
WAITING, READY, RUNNING, DONE = 'WAITING_FOR_ORDER', 'READY', 'RUNNING', 'DONE'


def is_go(text):
    """True only for the bare start word ('Go.', 'go!'), never for 'go to tag 7'."""
    return re.findall(r"[a-z']+", text.lower()) == [START_WORD]


class MissionNav(TerminalNav):

    def __init__(self):
        super().__init__()
        self.declare_parameter('image_topic', '/camera1/camera1/color/image_raw')
        self.declare_parameter('image_wait_s', 2.0)       # wait this long for a fresh frame
        self.declare_parameter('hold_s', 3.0)             # stand still at the tag (judges confirm)
        self.declare_parameter('run_time_limit_s', 600.0)  # 10-minute run clock

        self.run_state = WAITING
        self.order = None                 # list of tag ids, set by /parc/tag_order or /order
        self.visited, self.skipped = [], []
        self.abort = threading.Event()    # set by a real stop; ends the loop
        self.run_lock = threading.Lock()
        self.latest_image = None          # (Image msg, monotonic time received)

        self.started_pub = self.create_publisher(Bool, TOPICS['robot_started'], LATCHED)
        self.run_started_pub = self.create_publisher(Bool, TOPICS['run_started'], 10)
        self.visited_pub = self.create_publisher(Int32, TOPICS['tag_visited'], 10)
        self.image_pub = self.create_publisher(Image, TOPICS['tag_image'], 10)
        self.finished_pub = self.create_publisher(String, TOPICS['run_finished'], 10)

        self.create_subscription(
            Int32MultiArray, TOPICS['tag_order'], self.on_order_msg, LATCHED)
        self.create_subscription(
            String, TOPICS['start_word'], self.on_start_word, 10)
        # Sensor QoS (best effort) can read a reliable camera publisher.
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.on_image, qos_profile_sensor_data)

        self.started_pub.publish(Bool(data=True))
        self.publish_status(self.run_state)
        self.reply('Ready. Waiting for the tag order.')

    # -------------------------------------------------------- inputs
    def handle_line(self, line):
        """Intercept only the mission commands; everything else is unchanged."""
        s = line.strip()
        low = s.lower()
        if is_go(s) or low == '/go':
            self.on_go()
        elif low.startswith('/order'):
            try:
                self.set_order([int(a) for a in low.split()[1:]])
            except ValueError:
                self.say('Bad command. Use /order 7 1 9 4')
        elif low == '/run':
            self.say(f'Mission: {self.run_state}, order={self.order}, '
                     f'visited={self.visited}, skipped={self.skipped}')
        elif low == '/help':
            self.say(HELP)
            self.say(MISSION_HELP)
        else:
            super().handle_line(line)

    def on_start_word(self, msg):
        if is_go(msg.data):               # this topic accepts the start word only
            self.on_go()

    def on_order_msg(self, msg):
        self.set_order(list(msg.data))

    def on_image(self, msg):
        self.latest_image = (msg, time.monotonic())

    def cancel(self, quiet=False):
        """A real stop (typed or spoken) ends the run. navigate() cancels quietly."""
        if not quiet:
            self.abort.set()
        super().cancel(quiet)

    # -------------------------------------------------------- state changes
    def set_order(self, order):
        valid = sorted(self.registry.tags)
        bad = [t for t in order if t not in valid]
        if not order or bad:
            self.say(f'Order rejected: {order} (valid tags: {valid}).')
            return
        with self.run_lock:
            if self.run_state == RUNNING:
                self.say('Run in progress: ignoring the new order.')
                return
            self.order = list(order)
            self.run_state = READY
        self.publish_status(READY, order=self.order)
        self.reply(f'Order received: {" ".join(str(t) for t in order)}. Say go to start.')

    def on_go(self):
        with self.run_lock:
            if self.run_state == RUNNING:
                self.say('Already running; ignoring go.')
                return
            if self.run_state != READY or not self.order:
                self.reply('I do not have the tag order yet.')
                return
            order, self.order = self.order, None      # one order = one run
            self.run_state = RUNNING
        threading.Thread(target=self.run_mission, args=(order,), daemon=True).start()

    # -------------------------------------------------------- the run
    def run_mission(self, order):
        self.abort.clear()
        self.visited, self.skipped = [], []
        limit = float(self.get_parameter('run_time_limit_s').value)
        deadline = time.monotonic() + limit
        reason = 'completed'
        self.run_started_pub.publish(Bool(data=True))
        self.publish_status(RUNNING, order=order)
        self.reply('Starting.')
        try:
            for tid in order:                          # <- the for loop
                if self.abort.is_set():
                    reason = 'aborted'
                    break
                if time.monotonic() >= deadline:
                    reason = 'time_limit'
                    break
                reached = self.visit(tid, deadline)
                if self.abort.is_set():
                    reason = 'aborted'
                    break
                if time.monotonic() >= deadline:
                    reason = 'time_limit'
                    break
                if reached:
                    self.report_visit(tid)
                    self.visited.append(tid)
                else:                                  # brief: no going back to a skipped tag
                    self.skipped.append(tid)
                    self.reply(f'Skipping tag {tid}.')
        finally:
            self.finish_run(reason)

    def visit(self, tid, deadline):
        """Drive to one tag using the existing machinery. True if it was reached."""
        with self.lock:
            self.last_reached = None
        self.navigate([{'type': 'tag', 'id': tid}])
        while rclpy.ok() and not self.abort.is_set():
            if time.monotonic() >= deadline:
                self.cancel(quiet=True)
                return False
            with self.lock:
                idle = self.current is None and not self.queue
                reached = self.last_reached
            if idle:                                   # arrived, failed, or refused
                return reached == f'tag {tid}'
            time.sleep(0.2)
        return False

    def report_visit(self, tid):
        """At the tag: fresh photo, publish photo + id, then hold still."""
        t0 = time.monotonic()
        wait = float(self.get_parameter('image_wait_s').value)
        while time.monotonic() - t0 < wait:
            if self.latest_image and self.latest_image[1] > t0:
                break
            time.sleep(0.05)
        if self.latest_image is not None:
            self.image_pub.publish(self.latest_image[0])
        else:
            self.say('Warning: no camera image received; publishing the id only.')
        self.visited_pub.publish(Int32(data=int(tid)))
        self.say(f'Tag {tid} visited: id and image published.')
        self.abort.wait(float(self.get_parameter('hold_s').value))   # judges confirm

    def finish_run(self, reason):
        with self.run_lock:
            self.run_state = DONE
        self.finished_pub.publish(String(data=json.dumps({
            'reason': reason, 'visited': self.visited, 'skipped': self.skipped})))
        self.publish_status(DONE, reason=reason, visited=self.visited,
                            skipped=self.skipped)
        self.reply('Run finished.' if reason == 'completed' else f'Run ended: {reason}.')

    # -------------------------------------------------------- terminal loop
    def repl(self):
        self.say(MISSION_HELP)
        super().repl()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.repl()
    finally:
        node.cancel(quiet=True)           # don't leave the robot driving on exit
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
