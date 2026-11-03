"""Stand-in for the PARC referee, for testing only. Never launch it at the event.

    ros2 run task_solution mock_referee --ros-args -p tag_order:="[1, 4, 7, 9]"

Arms the run after arm_delay_s (publishes /parc/tag_order, state ARMED), then
follows the rules in "PARC 2026 - Referee Topics": RUNNING on StartSignal,
FINISHED on the last tag in the order or on Finish, TIMED_OUT at the time limit.
Movement on /odom while ARMED is flagged as a false start.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit

from task_solution.referee import (EVENTS, LATCHED, END_STATES, STATE_NAMES,
                                   TOPIC_FINISH, TOPIC_RUN_STATUS, TOPIC_START,
                                   TOPIC_TAG_ORDER, TOPIC_TAG_VISIT)


class MockReferee(Node):

    def __init__(self):
        super().__init__('mock_referee')
        self.declare_parameter('tag_order', [1, 4, 7, 9])
        self.declare_parameter('arm_delay_s', 3.0)
        self.declare_parameter('time_limit_s', 600.0)
        self.declare_parameter('false_start_speed', 0.02)   # m/s or rad/s on /odom

        self.order = [int(t) for t in self.get_parameter('tag_order').value]
        self.limit = float(self.get_parameter('time_limit_s').value)

        self.state = RunStatus.IDLE
        self.reported = []
        self.start_method = 0
        self.false_start = False
        self.start_time = None
        self.elapsed = 0.0
        self.created = self.get_clock().now()

        self.order_pub = self.create_publisher(TagOrder, TOPIC_TAG_ORDER, LATCHED)
        self.status_pub = self.create_publisher(RunStatus, TOPIC_RUN_STATUS, LATCHED)
        self.create_subscription(StartSignal, TOPIC_START, self.on_start, EVENTS)
        self.create_subscription(TagVisit, TOPIC_TAG_VISIT, self.on_visit, EVENTS)
        self.create_subscription(Finish, TOPIC_FINISH, self.on_finish, EVENTS)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.create_timer(0.5, self.tick)   # 2 Hz status
        self.get_logger().info(f'Mock referee up. Order will be {self.order}.')

    def set_state(self, state):
        if state == self.state:
            return
        self.get_logger().info(
            f'{STATE_NAMES[self.state]} -> {STATE_NAMES[state]}')
        if self.state == RunStatus.RUNNING:
            self.elapsed = self.now_s() - self.start_time
        self.state = state
        self.publish_status()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.state == RunStatus.IDLE:
            waited = (self.get_clock().now() - self.created).nanoseconds * 1e-9
            if waited >= float(self.get_parameter('arm_delay_s').value):
                msg = TagOrder()
                msg.stamp = self.get_clock().now().to_msg()
                msg.tag_ids = self.order
                self.order_pub.publish(msg)
                self.set_state(RunStatus.ARMED)
                self.get_logger().info(
                    'ARMED. Start with: ros2 run task_solution start_run')
        elif self.state == RunStatus.RUNNING:
            self.elapsed = self.now_s() - self.start_time
            if self.elapsed >= self.limit:
                self.set_state(RunStatus.TIMED_OUT)
        self.publish_status()

    def publish_status(self):
        msg = RunStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.elapsed_s = float(self.elapsed)
        msg.time_limit_s = self.limit
        msg.tag_order = self.order if self.state != RunStatus.IDLE else []
        msg.tags_reported = self.reported
        msg.start_method = self.start_method
        msg.false_start = self.false_start
        self.status_pub.publish(msg)

    def on_start(self, msg):
        if self.state != RunStatus.ARMED:
            self.get_logger().warn(
                f'StartSignal while {STATE_NAMES[self.state]}: ignored.')
            return
        self.start_method = msg.method
        self.start_time = self.now_s()
        how = f'voice "{msg.heard}"' if msg.method == StartSignal.METHOD_VOICE else 'command'
        self.get_logger().info(f'Start by {how}. Clock running.')
        self.set_state(RunStatus.RUNNING)

    def on_visit(self, msg):
        if self.state != RunStatus.RUNNING:
            self.get_logger().warn(f'TagVisit {msg.tag_id} while not RUNNING: ignored.')
            return
        if msg.tag_id in self.reported:
            self.get_logger().warn(f'Tag {msg.tag_id} reported twice.')
            return
        self.reported.append(int(msg.tag_id))
        self.get_logger().info(
            f'TagVisit {msg.tag_id} at {self.now_s() - self.start_time:.1f} s '
            f'(image: {msg.image.format}, {len(msg.image.data)} bytes, '
            f'frame "{msg.header.frame_id}").')
        if self.order and msg.tag_id == self.order[-1]:
            self.set_state(RunStatus.FINISHED)
        else:
            self.publish_status()

    def on_finish(self, msg):
        if self.state in END_STATES:
            return
        why = 'completed' if msg.reason == Finish.REASON_COMPLETED else 'gave up'
        self.get_logger().info(f'Finish received ({why}).')
        self.set_state(RunStatus.FINISHED)

    def on_odom(self, msg):
        if self.state != RunStatus.ARMED or self.false_start:
            return
        lim = float(self.get_parameter('false_start_speed').value)
        t = msg.twist.twist
        if abs(t.linear.x) > lim or abs(t.linear.y) > lim or abs(t.angular.z) > lim:
            self.false_start = True
            self.get_logger().error('FALSE START: robot moved while ARMED.')
            self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = MockReferee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
