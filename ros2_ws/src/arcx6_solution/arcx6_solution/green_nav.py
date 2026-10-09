# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import PointStamped
# from nav2_msgs.action import NavigateToPose

# from rclpy.action import ActionClient


# class GreenNavigator(Node):

#     def __init__(self):
#         super().__init__('green_navigator')

#         # Receive the green object's position in the map frame
#         self.subscription = self.create_subscription(
#             PointStamped,
#             '/green_detector/target_map',
#             self.green_callback,
#             10
#         )

#         # Client for Nav2's NavigateToPose action
#         self.nav_client = ActionClient(
#             self,
#             NavigateToPose,
#             '/navigate_to_pose'
#         )

#         # Prevent sending a new goal every time the camera publishes
#         self.goal_sent = False

#         self.get_logger().info('Green navigator started.')

#     def green_callback(self, msg):

#         # Only send the first detected target
#         if self.goal_sent:
#             return

#         x = msg.point.x
#         y = msg.point.y

#         self.get_logger().info(
#             f'Green target found: x={x:.2f}, y={y:.2f}'
#         )

#         # Wait for Nav2
#         if not self.nav_client.wait_for_server(timeout_sec=2.0):
#             self.get_logger().warn(
#                 'Nav2 NavigateToPose action server not available.'
#             )
#             return

#         # Create the navigation goal
#         goal = NavigateToPose.Goal()

#         goal.pose.header.frame_id = 'map'
#         goal.pose.header.stamp = self.get_clock().now().to_msg()

#         goal.pose.pose.position.x = x
#         goal.pose.pose.position.y = y

#         # Orientation: yaw = 0 for now
#         goal.pose.pose.orientation.z = 0.0
#         goal.pose.pose.orientation.w = 1.0

#         self.get_logger().info(
#             f'Sending Nav2 goal: x={x:.2f}, y={y:.2f}'
#         )

#         self.goal_sent = True

#         future = self.nav_client.send_goal_async(
#             goal,
#             feedback_callback=self.feedback_callback
#         )

#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):

#         goal_handle = future.result()

#         if not goal_handle.accepted:
#             self.get_logger().warn('Nav2 rejected the goal.')
#             self.goal_sent = False
#             return

#         self.get_logger().info('Nav2 accepted the goal.')

#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):

#         result = future.result()

#         self.get_logger().info(
#             f'Navigation finished with status: {result.status}'
#         )

#     def feedback_callback(self, feedback_msg):

#         feedback = feedback_msg.feedback

#         # Nav2 gives us the remaining distance
#         distance = feedback.distance_remaining

#         self.get_logger().info(
#             f'Distance remaining: {distance:.2f} m'
#         )


# def main(args=None):

#     rclpy.init(args=args)

#     node = GreenNavigator()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import math
 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
 
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Twist
from nav2_msgs.action import NavigateToPose
 
SEARCHING = 'SEARCHING'      # rotating in place, looking for green
SETTLING = 'SETTLING'        # green seen: stopped, confirming the detection
NAVIGATING = 'NAVIGATING'    # goal sent to Nav2
DONE = 'DONE'
 
 
class GreenNavigator(Node):
 
    def __init__(self):
        super().__init__('green_navigator')
 
        # ---------------- Parameters ----------------
        # Rotation goes through the collision monitor input so it stays protected.
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_smoothed')
        self.declare_parameter('search_angular_speed', 0.4)   # rad/s, + = CCW
        self.declare_parameter('settle_time', 0.7)            # s stopped before trusting a detection
        self.declare_parameter('max_target_age', 0.5)         # s, detection must be this fresh
        self.declare_parameter('retry_after_fail', True)      # search again if Nav2 fails
 
        self.state = SEARCHING
        self.latest_target = None        # PointStamped in map frame
        self.latest_target_time = None   # rclpy Time
        self.state_start = self.get_clock().now()
        self.search_start = self.get_clock().now()
        self.rotations_done = 0
 
        # ---------------- ROS interfaces ----------------
        self.subscription = self.create_subscription(
            PointStamped, '/green_detector/target_map',
            self.green_callback, 10)
 
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
 
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
 
        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.control_loop)
 
        self.get_logger().info('Green navigator started: searching for green.')
 
    # ------------------------------------------------------------------
    def set_state(self, new_state):
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_start = self.get_clock().now()
        if new_state == SEARCHING:
            self.search_start = self.state_start
            self.rotations_done = 0
 
    def publish_rotation(self, wz):
        msg = Twist()
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)
 
    def stop_robot(self):
        self.publish_rotation(0.0)
 
    # ------------------------------------------------------------------
    def green_callback(self, msg):
        """Called whenever the detector sees green (target in the map frame)."""
        self.latest_target = msg
        self.latest_target_time = self.get_clock().now()
 
        if self.state == SEARCHING:
            self.get_logger().info('Green spotted, stopping to confirm.')
            self.stop_robot()
            self.set_state(SETTLING)
 
    # ------------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now()
 
        if self.state == SEARCHING:
            speed = self.get_parameter('search_angular_speed').value
            self.publish_rotation(speed)
 
            # Log once per full revolution without finding anything
            elapsed = (now - self.search_start).nanoseconds * 1e-9
            full_turn = 2.0 * math.pi / abs(speed)
            if elapsed > full_turn * (self.rotations_done + 1):
                self.rotations_done += 1
                self.get_logger().warn(
                    f'Completed {self.rotations_done} full rotation(s) '
                    f'without seeing green. Still searching.')
 
        elif self.state == SETTLING:
            self.stop_robot()
            settle = self.get_parameter('settle_time').value
            waited = (now - self.state_start).nanoseconds * 1e-9
            if waited < settle:
                return
 
            max_age = self.get_parameter('max_target_age').value
            age = (now - self.latest_target_time).nanoseconds * 1e-9
            if self.latest_target is not None and age <= max_age:
                self.send_goal(self.latest_target)
            else:
                self.get_logger().info(
                    'Detection did not hold while stopped, resuming search.')
                self.set_state(SEARCHING)
 
    # ------------------------------------------------------------------
    def send_goal(self, target):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Nav2 NavigateToPose action server not available, resuming search.')
            self.set_state(SEARCHING)
            return
 
        x = target.point.x
        y = target.point.y
        self.get_logger().info(f'Sending Nav2 goal: x={x:.2f}, y={y:.2f}')
 
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0   # yaw = 0
 
        self.set_state(NAVIGATING)
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self.after_failure()
            return
 
        self.get_logger().info('Nav2 accepted the goal.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)
 
    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached the green target.')
            self.set_state(DONE)
        else:
            self.get_logger().warn(f'Navigation ended with status {status}.')
            self.after_failure()
 
    def after_failure(self):
        self.latest_target = None
        if self.get_parameter('retry_after_fail').value:
            self.set_state(SEARCHING)
        else:
            self.set_state(DONE)
 
    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m', throttle_duration_sec=1.0)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = GreenNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
