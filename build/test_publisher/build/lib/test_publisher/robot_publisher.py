#!/usr/bin/env python3
"""
Script to move Robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):
    def __init__(self):
        super().__init__("move_robot")
        # Create a publisher which can "talk" to Robot and tell it to move
        self.pub = self.create_publisher(
            Twist, "/robot_base_controller/cmd_vel_unstamped", 10
        )

    def run(self):
        # Create a Twist message and add linear x and angular z values
        move_cmd = Twist()

        ######## Move Straight ########
        print("Moving Straight")
        move_cmd.linear.x = 0.5  # move in x axis at 0.5 m/s
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 4 seconds publish cmd_vel move commands
        while time.time() - now < 4:
            self.pub.publish(move_cmd)  # publish to robot

        ######## Stop ########
        print("Stopping")
        # Assigning both to 0.0 stops the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 5 seconds publish cmd_vel move commands
        while time.time() - now < 5:
            self.pub.publish(move_cmd)

        ######## Rotating Counterclockwise ########
        print("Rotating")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.7  # rotate at 0.7 rad/s

        now = time.time()
        # For the next 15 seconds publish cmd_vel move commands
        while time.time() - now < 15:
            self.pub.publish(move_cmd)

        ######## Stop ########
        print("Stopping")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        now = time.time()
        # For the next 3 seconds publish cmd_vel move commands
        while time.time() - now < 3:
            self.pub.publish(move_cmd)

        print("Exit")


def main(args=None):
    rclpy.init(args=args)

    move_robot = MoveRobot()
    move_robot.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()