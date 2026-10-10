import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient


class GreenNavigator(Node):

    def __init__(self):
        super().__init__('green_navigator')

        # Receive the green object's position in the map frame
        self.subscription = self.create_subscription(
            PointStamped,
            '/green_detector/target_map',
            self.green_callback,
            10
        )

        # Client for Nav2's NavigateToPose action
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Prevent sending a new goal every time the camera publishes
        self.goal_sent = False

        self.get_logger().info('Green navigator started.')

    def green_callback(self, msg):

        # Only send the first detected target
        if self.goal_sent:
            return

        x = msg.point.x
        y = msg.point.y

        self.get_logger().info(
            f'Green target found: x={x:.2f}, y={y:.2f}'
        )

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Nav2 NavigateToPose action server not available.'
            )
            return

        # Create the navigation goal
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y


        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal: x={x:.2f}, y={y:.2f}'
        )

        self.goal_sent = True

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self.goal_sent = False
            return

        self.get_logger().info('Nav2 accepted the goal.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            f'Navigation finished with status: {result.status}'
        )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        # Nav2 gives us the remaining distance
        distance = feedback.distance_remaining

        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GreenNavigator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()