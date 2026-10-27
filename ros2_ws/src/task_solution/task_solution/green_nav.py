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

        #Parameters 
        # Rotation goes through the collision monitor input so it stays protected.
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_smoothed')
        self.declare_parameter('search_angular_speed', 0.1)   # rad/s, + = CCW
        self.declare_parameter('settle_time', 0.7)            # s stopped before trusting a detection
        self.declare_parameter('max_target_age', 0.5)         # s, detection must be this fresh
        self.declare_parameter('retry_after_fail', True)      # search again if Nav2 fails
        self.declare_parameter('look_ahead_time', 1.5)        # s held still looking ahead before rotating
        self.declare_parameter('arrival_distance', 0.6)       # m: a failure this close counts as arrived
        self.declare_parameter('max_retries', 3)              # resend the SAME target this many times
        self.declare_parameter('lock_distance', 1.0)          # m: inside this the goal is locked, no searching/refining
        self.declare_parameter('finish_distance', 0.0)        # m: >0 cancels Nav2 when this close (0 = let Nav2 finish)
        self.declare_parameter('refine_threshold', 0.12)      # m: new detection must differ this much to update the goal
        self.declare_parameter('refine_period', 2.0)          # s: minimum time between goal updates

        self.state = SEARCHING
        self.latest_target = None        # PointStamped in map frame
        self.latest_target_time = None   # rclpy Time
        self.target = None               # target currently being navigated to
        self.last_distance = None        # last distance_remaining from Nav2
        self.goal_handle = None
        self.goal_seq = 0                # id of the newest goal; results of older goals are ignored
        self.armed = False
        self.locked = False
        self.retries = 0
        self.state_start = self.get_clock().now()
        self.search_start = self.get_clock().now()
        self.last_refine_time = self.get_clock().now()
        self.rotations_done = 0

        #ROS interfaces
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
        if new_state == self.state:
            return
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


    def green_callback(self, msg):
        """Called whenever the detector sees green (target in the map frame)."""
        if self.locked:
            return  # goal is fixed; ignore further detections

        self.latest_target = msg
        self.latest_target_time = self.get_clock().now()

        if self.state == SEARCHING:
            self.get_logger().info('Green spotted, stopping to confirm.')
            self.stop_robot()
            self.set_state(SETTLING)
        elif self.state == NAVIGATING:
            self.maybe_refine(msg)

    def maybe_refine(self, msg):
        """While still far away, correct the goal if a new detection disagrees."""
        if self.target is None:
            return
        now = self.get_clock().now()
        period = self.get_parameter('refine_period').value
        if (now - self.last_refine_time).nanoseconds * 1e-9 < period:
            return
        dx = msg.point.x - self.target.point.x
        dy = msg.point.y - self.target.point.y
        shift = math.hypot(dx, dy)
        if shift < self.get_parameter('refine_threshold').value:
            return
        self.get_logger().info(
            f'New detection is {shift:.2f} m from the current goal: updating goal.')
        self.last_refine_time = now
        self.send_goal(msg, refine=True)

    def control_loop(self):
        now = self.get_clock().now()

        if self.state == SEARCHING:
            elapsed = (now - self.search_start).nanoseconds * 1e-9
            look = self.get_parameter('look_ahead_time').value
            if elapsed < look:

                return

            speed = self.get_parameter('search_angular_speed').value
            self.publish_rotation(speed)

            # Log once per full revolution without finding anything
            full_turn = 2.0 * math.pi / abs(speed)
            if (elapsed - look) > full_turn * (self.rotations_done + 1):
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
                self.retries = 0
                self.locked = False
                self.send_goal(self.latest_target)
            else:
                self.get_logger().info(
                    'Detection did not hold while stopped, resuming search.')
                self.set_state(SEARCHING)

    def send_goal(self, target, refine=False):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 NavigateToPose action server not available.')
            if not refine:
                self.set_state(SEARCHING)
            return

        self.target = target
        self.last_distance = None
        self.armed = False
        x = target.point.x
        y = target.point.y
        self.get_logger().info(
            f'Sending Nav2 goal: x={x:.2f}, y={y:.2f}'
            + (' (refined)' if refine else ''))

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0   # yaw = 0

        self.set_state(NAVIGATING)

        self.goal_seq += 1
        seq = self.goal_seq
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb, s=seq: self.feedback_callback(fb, s))
        future.add_done_callback(lambda f, s=seq: self.goal_response_callback(f, s))

    def goal_response_callback(self, future, seq):
        if seq != self.goal_seq:
            return  # superseded by a newer goal
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self.after_failure()
            return

        self.get_logger().info('Nav2 accepted the goal.')
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f, s=seq: self.result_callback(f, s))

    def result_callback(self, future, seq):
        if seq != self.goal_seq:
            return  
        status = future.result().status
        if self.state != NAVIGATING:
            self.get_logger().info(
                f'Nav2 goal ended with status {status}; already finished.')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached the green target.')
            self.set_state(DONE)
        else:
            # 4 = succeeded, 5 = canceled, 6 = aborted
            self.get_logger().warn(
                f'Navigation ended with status {status} '
                f'(last distance remaining: {self.last_distance})')
            self.after_failure()

    def after_failure(self):
        arrival = self.get_parameter('arrival_distance').value

        # Failed while already close to the target, we will treat it like it has arrived.
        if self.last_distance is not None and self.last_distance <= arrival:
            self.get_logger().info(
                f'Within {arrival:.2f} m of the target, counting it as reached.')
            self.stop_robot()
            self.set_state(DONE)
            return

        # Far from the target: it is fixed in the map, so retry the same position instead of searching.
        limit = self.get_parameter('max_retries').value
        if self.locked:
            limit = max(limit, 10)
        if self.target is not None and self.retries < limit:
            self.retries += 1
            self.get_logger().info(f'Retrying the same target ({self.retries}/{limit}).')
            self.send_goal(self.target)
            return

       
        if self.locked:
            self.get_logger().warn(
                'Out of retries near a locked target; stopping (not searching).')
            self.stop_robot()
            self.set_state(DONE)
            return

    
        self.latest_target = None
        self.target = None
        if self.get_parameter('retry_after_fail').value:
            self.set_state(SEARCHING)
        else:
            self.set_state(DONE)

    def feedback_callback(self, feedback_msg, seq):
        if seq != self.goal_seq:
            return
        distance = feedback_msg.feedback.distance_remaining
        self.last_distance = distance
        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m', throttle_duration_sec=1.0)

        
        valid = distance > 0.01

        lock = self.get_parameter('lock_distance').value
        if self.state == NAVIGATING and valid and not self.locked and distance <= lock:
            self.locked = True
            self.get_logger().info(
                f'Within {lock:.1f} m of the target: goal locked, '
                f'searching and refining disabled.')

        finish = self.get_parameter('finish_distance').value
        if distance > max(finish * 3.0, 0.5):
            self.armed = True

        if finish > 0.0 and self.state == NAVIGATING and self.armed and distance <= finish:
            self.get_logger().info(
                f'Within {finish:.2f} m of the target: cancelling Nav2 and stopping.')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.stop_robot()
            self.set_state(DONE)


def main(args=None):
    rclpy.init(args=args)
    node = GreenNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()