#!/usr/bin/env python3
"""Terminal front-end that sends Nav2 goals (stand-in for the LLM).

Typed lines (or messages on /speech/transcript) become a structured command:

    {"action": "navigate", "targets": [{"type": "tag", "id": 7}, ...]}
    {"action": "cancel"}

The executor only ever sees that structure, so an LLM backend (Groq, Ollama,
llama.cpp) can later produce the same dict without touching the Nav2 code.
"""
import json
import math
import os
import re
import threading
import time

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

try:                                   # normal case: run as part of the package
    from .llm_backend import (DEFAULT_BASE_URL, DEFAULT_MODEL, LLMError,
                              LLMParser)
except ImportError:                    # running the file directly
    from llm_backend import (DEFAULT_BASE_URL, DEFAULT_MODEL, LLMError,
                             LLMParser)

# Package name taken from where this module lives, so a rename can't break it.
PKG = (__package__ or 'task_solution').split('.')[0]

HELP = """Commands:
  /pose X Y [YAW_DEG]   go to a map position (yaw defaults to 0)
  /spot NAME            go to a named spot from locations.yaml
  /tag N                go to AprilTag N's location
  /seq N N N ...        visit several tags in order
  /stop                 cancel the current goal / sequence
  /where                print the robot's pose in the map frame
  /tags                 list tags and whether their pose is known
  /help  /quit
Plain text goes to the LLM if configured, otherwise the rule-based parser:
  "go to tag 7", "visit four then nine", "stop"."""

NUMBER_WORDS = {
    'zero': 0, 'oh': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4,
    'for': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9,
}
CANCEL_WORDS = {'stop', 'cancel', 'halt', 'abort'}


# ----------------------------------------------------------------------
# Registry
# ----------------------------------------------------------------------
class Registry:
    """Named spots and tag -> pose lookup. A tag pose may be unknown (None)."""

    def __init__(self, path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        self.spots = data.get('spots') or {}
        self.tags = {int(k): (v or {}) for k, v in (data.get('tags') or {}).items()}

    @staticmethod
    def _as_pose(d):
        return (float(d['x']), float(d['y']), float(d.get('yaw_deg', 0.0)))

    def spot_pose(self, name):
        d = self.spots.get(name)
        return None if d is None else self._as_pose(d)

    def tag_pose(self, tag_id):
        """(x, y, yaw_deg) or None when the tag's location is not known yet."""
        entry = self.tags.get(tag_id)
        if entry is None:
            return None
        if entry.get('pose') is not None:
            return self._as_pose(entry['pose'])
        if entry.get('spot') is not None:
            return self.spot_pose(entry['spot'])
        return None

    def set_tag_pose(self, tag_id, pose):
        """Hook for Sub-team 1: store a tag pose seen by the detector."""
        self.tags.setdefault(tag_id, {})['pose'] = {
            'x': pose[0], 'y': pose[1], 'yaw_deg': pose[2]}


# ----------------------------------------------------------------------
# Parsing: text -> structured command (the part an LLM will replace)
# ----------------------------------------------------------------------
def extract_ids(text):
    """Pull tag numbers out of text; tolerates 'seven' and STT's 'tag for'."""
    tokens = re.findall(r'[a-z]+|\d+', text.lower())
    ids = []
    for i, tok in enumerate(tokens):
        prev = tokens[i - 1] if i else ''
        if tok.isdigit():
            ids.append(int(tok))
        elif tok in ('for', 'oh'):
            if prev in ('tag', 'tags'):      # only trust these right after 'tag'
                ids.append(NUMBER_WORDS[tok])
        elif tok in NUMBER_WORDS:
            ids.append(NUMBER_WORDS[tok])
    return ids


def parse_text(text):
    tokens = set(re.findall(r'[a-z]+', text.lower()))
    if tokens & CANCEL_WORDS:
        return {'action': 'cancel'}
    if tokens & {'tag', 'tags'}:
        ids = extract_ids(text)
        if ids:
            return {'action': 'navigate',
                    'targets': [{'type': 'tag', 'id': i} for i in ids]}
    return None


def parse_slash(line):
    parts = line.split()
    cmd, args = parts[0].lower(), parts[1:]
    if cmd == '/pose':
        x, y = float(args[0]), float(args[1])
        yaw = float(args[2]) if len(args) > 2 else 0.0
        return {'action': 'navigate',
                'targets': [{'type': 'pose', 'x': x, 'y': y, 'yaw_deg': yaw}]}
    if cmd == '/spot':
        return {'action': 'navigate',
                'targets': [{'type': 'spot', 'name': args[0]}]}
    if cmd == '/tag':
        return {'action': 'navigate',
                'targets': [{'type': 'tag', 'id': int(args[0])}]}
    if cmd == '/seq':
        return {'action': 'navigate',
                'targets': [{'type': 'tag', 'id': int(a)} for a in args]}
    if cmd == '/stop':
        return {'action': 'cancel'}
    if cmd in ('/where', '/tags', '/help', '/quit'):
        return {'action': cmd[1:]}
    raise ValueError(f'unknown command {cmd}')


# ----------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------
class TerminalNav(Node):

    def __init__(self):
        super().__init__('terminal_nav')

        default_file = os.path.join(
            get_package_share_directory(PKG), 'config', 'locations.yaml')
        self.declare_parameter('locations_file', default_file)
        self.registry = Registry(self.get_parameter('locations_file').value)

        # LLM backend (optional). Key comes from the environment, never the code.
        self.declare_parameter('use_llm', True)
        self.declare_parameter('llm_base_url', DEFAULT_BASE_URL)
        self.declare_parameter('llm_model', DEFAULT_MODEL)
        self.declare_parameter('llm_timeout', 8.0)
        self.declare_parameter('llm_reasoning_effort', '')   # try 'low' for speed
        self.llm = None
        if self.get_parameter('use_llm').value:
            base = self.get_parameter('llm_base_url').value
            model = self.get_parameter('llm_model').value
            key = os.environ.get('LLM_API_KEY') or os.environ.get('GROQ_API_KEY')
            if key or 'api.groq.com' not in base:     # local servers need no key
                self.llm = LLMParser(
                    sorted(self.registry.tags), base, model, key,
                    self.get_parameter('llm_timeout').value,
                    self.get_parameter('llm_reasoning_effort').value)
                print(f'LLM backend: {model} @ {base}')
            else:
                print('GROQ_API_KEY not set: using the rule-based parser only.')

        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/task/status', 10)
        self.say_pub = self.create_publisher(String, '/robot/say', 10)
        # Same entry point the STT node will publish to later.
        self.create_subscription(
            String, '/speech/transcript', lambda m: self.handle_line(m.data), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lock = threading.RLock()
        self.queue = []            # [(label, (x, y, yaw_deg)), ...] still to visit
        self.current = None        # label of the goal being driven to
        self.goal_handle = None
        self.goal_seq = 0          # results/feedback from older goals are ignored
        self.last_fb_print = 0.0

    # -------------------------------------------------------- output
    def say(self, text):
        print(text, flush=True)

    def reply(self, text):
        """Something the robot 'says' (printed now, TTS can subscribe later)."""
        print(f'Robot: {text}', flush=True)
        self.say_pub.publish(String(data=text))

    def handle_text(self, line):
        rule = parse_text(line)
        if rule is not None and rule['action'] == 'cancel':
            self.execute(rule)           # 'stop' never waits on a model
            return
        cmd = None
        if self.llm is not None:
            try:
                cmd = self.llm.parse(line)
                self.say(f'[llm] {json.dumps(cmd)}')
            except LLMError as e:
                self.say(f'(LLM unavailable: {e}; using the rule-based parser)')
        if cmd is None:
            cmd = rule
        if cmd is None:
            self.say("Didn't understand that. Try 'go to tag 7' or /help.")
            return
        self.execute(cmd)

    def publish_status(self, state, **extra):
        msg = String()
        msg.data = json.dumps({
            'state': state, 'target': self.current,
            'remaining_stops': len(self.queue), **extra})
        self.status_pub.publish(msg)

    # -------------------------------------------------------- input
    def handle_line(self, line):
        line = line.strip()
        if not line:
            return
        if line.startswith('/'):
            try:
                self.execute(parse_slash(line))
            except (ValueError, IndexError):
                self.say('Bad command. Type /help.')
            return
        # Free text may call a remote model: keep it off the ROS callback thread
        # so a slow reply cannot stall Nav2 feedback and results.
        threading.Thread(target=self.handle_text, args=(line,), daemon=True).start()

    def execute(self, cmd):
        action = cmd['action']
        if action == 'cancel':
            self.cancel()
        elif action == 'navigate':
            self.navigate(cmd['targets'])
        elif action == 'where':
            self.where()
        elif action == 'tags':
            for tid in sorted(self.registry.tags):
                p = self.registry.tag_pose(tid)
                self.say(f'  tag {tid}: ' + (
                    f'x={p[0]:.2f} y={p[1]:.2f} yaw={p[2]:.0f}' if p is not None
                    else 'unknown (would need to search)'))
        elif action == 'help':
            self.say(HELP)
        elif action == 'say':
            self.reply(cmd['text'])

    # -------------------------------------------------------- navigation
    def resolve(self, t):
        """Target dict -> (label, pose, error). Never guesses."""
        kind = t['type']
        if kind == 'pose':
            return (f"pose({t['x']:.2f}, {t['y']:.2f})",
                    (t['x'], t['y'], t['yaw_deg']), None)
        if kind == 'spot':
            pose = self.registry.spot_pose(t['name'])
            if pose is None:
                return None, None, f"unknown spot '{t['name']}'"
            return t['name'], pose, None
        if kind == 'tag':
            tid = t['id']
            if tid not in self.registry.tags:          # note: tag 0 is valid
                valid = sorted(self.registry.tags)
                return None, None, f'tag {tid} is not one of {valid}'
            pose = self.registry.tag_pose(tid)
            if pose is None:                           # `is None`, never `if not`
                return None, None, (f'tag {tid} has no known pose yet '
                                    '(search behaviour not implemented)')
            return f'tag {tid}', pose, None
        return None, None, f'bad target {t}'

    def navigate(self, targets):
        resolved = []
        for t in targets:
            label, pose, err = self.resolve(t)
            if err:
                self.say(f'Not moving: {err}')        # all-or-nothing validation
                return
            resolved.append((label, pose))
        with self.lock:
            self.cancel(quiet=True)                   # new command replaces old
            self.queue = resolved
            self.say('Plan: ' + ' -> '.join(l for l, _ in resolved))
            self.start_next()

    def make_pose(self, x, y, yaw_deg):
        yaw = math.radians(yaw_deg)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def start_next(self):
        with self.lock:
            if not self.queue:
                self.current = None
                self.goal_handle = None
                self.say('All stops done.')
                self.publish_status('IDLE')
                return
            if not self.client.server_is_ready():
                self.say('Nav2 NavigateToPose server is not available.')
                self.queue.clear()
                self.current = None
                self.publish_status('FAILED', reason='nav2_unavailable')
                return
            label, pose = self.queue.pop(0)
            self.current = label
            self.goal_seq += 1
            seq = self.goal_seq
            goal = NavigateToPose.Goal()
            goal.pose = self.make_pose(*pose)
            self.say(f'Going to {label}: x={pose[0]:.2f} y={pose[1]:.2f} '
                     f'yaw={pose[2]:.0f} deg')
            self.publish_status('NAVIGATING')
            fut = self.client.send_goal_async(
                goal, feedback_callback=lambda fb, s=seq: self.on_feedback(fb, s))
            fut.add_done_callback(lambda f, s=seq: self.on_goal_response(f, s))

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:                      # superseded or cancelled
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.say(f'Nav2 rejected the goal to {self.current}.')
            self.queue.clear()
            self.publish_status('FAILED', reason='rejected')
            self.current = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, s=seq: self.on_result(f, s))

    def on_feedback(self, fb, seq):
        if seq != self.goal_seq:
            return
        now = time.monotonic()
        if now - self.last_fb_print >= 2.0:
            self.last_fb_print = now
            d = fb.feedback.distance_remaining
            self.say(f'  ... {d:.1f} m to {self.current}')
            self.publish_status('NAVIGATING', distance=round(d, 2))

    def on_result(self, future, seq):
        if seq != self.goal_seq:
            return
        status = future.result().status
        with self.lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.say(f'Arrived at {self.current}.')
                self.start_next()
            else:
                self.say(f'Navigation to {self.current} ended with status '
                         f'{status}; dropping remaining stops.')
                self.queue.clear()
                self.publish_status('FAILED', status=int(status))
                self.current = None
                self.goal_handle = None

    def cancel(self, quiet=False):
        with self.lock:
            had_goal = self.goal_handle is not None or self.queue
            self.queue.clear()
            self.goal_seq += 1                        # invalidate in-flight callbacks
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.current = None
            if not quiet:
                self.say('Cancelled.' if had_goal else 'Nothing to cancel.')
                self.publish_status('IDLE')

    def where(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.say(f'No map -> base_footprint transform yet: {e}')
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.say(f'x={t.transform.translation.x:.2f} '
                 f'y={t.transform.translation.y:.2f} '
                 f'yaw={math.degrees(yaw):.0f} deg')

    # -------------------------------------------------------- terminal loop
    def repl(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.say('Warning: Nav2 action server not found yet.')
        self.say(HELP)
        while rclpy.ok():
            try:
                line = input('> ')
            except (EOFError, KeyboardInterrupt):
                break
            if line.strip().lower() in ('/quit', 'quit', 'exit'):
                break
            self.handle_line(line)


def main(args=None):
    rclpy.init(args=args)
    node = TerminalNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.repl()
    finally:
        node.cancel(quiet=True)       # don't leave the robot driving on exit
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()