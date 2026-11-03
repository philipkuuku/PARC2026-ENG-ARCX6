#!/usr/bin/env python3
"""Stand-in for the PARC organisers while we practise.

  - hears  /parc/robot_started  -> publishes the tag order on /parc/tag_order
  - optionally sends "go" on /parc/start_word (auto_go_s > 0) to test without a mic
  - logs every visit, image and the end-of-run report the robot publishes

Replace with the real status package once the organisers release it.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, Int32MultiArray, String

try:
    from .parc_mission import LATCHED, TOPICS
except ImportError:
    from parc_mission import LATCHED, TOPICS


class DummyOrganiser(Node):

    def __init__(self):
        super().__init__('dummy_organiser')
        self.declare_parameter('order', [7, 1, 9, 4])
        self.declare_parameter('order_delay_s', 1.0)   # after the robot says it started
        self.declare_parameter('auto_go_s', 0.0)       # >0: send "go" this long after the order

        self.order_pub = self.create_publisher(Int32MultiArray, TOPICS['tag_order'], LATCHED)
        self.go_pub = self.create_publisher(String, TOPICS['start_word'], 10)

        self.create_subscription(Bool, TOPICS['robot_started'], self.on_started, LATCHED)
        self.create_subscription(Bool, TOPICS['run_started'],
                                 lambda m: self.get_logger().info('RUN STARTED'), 10)
        self.create_subscription(Int32, TOPICS['tag_visited'],
                                 lambda m: self.get_logger().info(f'TAG VISITED: {m.data}'), 10)
        self.create_subscription(Image, TOPICS['tag_image'], self.on_image, 10)
        self.create_subscription(String, TOPICS['run_finished'],
                                 lambda m: self.get_logger().info(f'RUN FINISHED: {m.data}'), 10)

    def later(self, delay, fn):
        """One-shot timer (rclpy timers repeat, so cancel after the first fire)."""
        holder = {}

        def fire():
            holder['t'].cancel()
            self.destroy_timer(holder['t'])
            fn()

        holder['t'] = self.create_timer(float(delay), fire)

    def on_started(self, msg):
        if not msg.data:
            return
        self.get_logger().info('Robot reports it has started.')
        delay = self.get_parameter('order_delay_s').value
        self.later(delay, self.send_order)
        go_after = float(self.get_parameter('auto_go_s').value)
        if go_after > 0:
            self.later(delay + go_after, self.send_go)

    def send_order(self):
        order = [int(t) for t in self.get_parameter('order').value]
        self.order_pub.publish(Int32MultiArray(data=order))
        self.get_logger().info(f'Published tag order: {order}')

    def send_go(self):
        self.go_pub.publish(String(data='go'))
        self.get_logger().info('Sent "go".')

    def on_image(self, msg):
        self.get_logger().info(
            f'TAG IMAGE: {msg.width}x{msg.height} {msg.encoding}, frame {msg.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = DummyOrganiser()
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
