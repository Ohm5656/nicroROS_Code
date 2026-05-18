#!/usr/bin/env python3

# ros lib
import os
import math
from time import sleep

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from yahboomcar_laser.common import *

print("import done")

RAD2DEG = 180.0 / math.pi


class laserAvoid(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create subscribers
        self.sub_laser = self.create_subscription(
            LaserScan, "/scan", self.registerScan, 1
        )
        self.sub_JoyState = self.create_subscription(
            Bool, "/JoyState", self.JoyStateCallback, 1
        )

        # Create publisher
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)

        # Declare parameters
        self.declare_parameter("linear", 0.3)
        self.declare_parameter("angular", 1.0)
        self.declare_parameter("LaserAngle", 45.0)
        self.declare_parameter("ResponseDist", 0.55)
        self.declare_parameter("Switch", False)

        # Read parameters
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.angular = self.get_parameter("angular").get_parameter_value().double_value
        self.LaserAngle = self.get_parameter("LaserAngle").get_parameter_value().double_value
        self.ResponseDist = self.get_parameter("ResponseDist").get_parameter_value().double_value
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value

        # State variables
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.Moving = False
        self.ros_ctrl = SinglePID()

        # Timer for refreshing parameter values
        self.timer = self.create_timer(0.01, self.on_timer)

        self.get_logger().info("laser_Avoidance node started")

    def on_timer(self):
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value
        self.angular = self.get_parameter("angular").get_parameter_value().double_value
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.LaserAngle = self.get_parameter("LaserAngle").get_parameter_value().double_value
        self.ResponseDist = self.get_parameter("ResponseDist").get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub_vel.publish(twist)

    def exit_pro(self):
        self.publish_cmd(0.0, 0.0)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return

        ranges = np.array(scan_data.ranges)

        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        # Process radar data
        for i in range(len(ranges)):
            r = ranges[i]

            # Skip invalid lidar values
            if not math.isfinite(r):
                continue

            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

            if angle > 180:
                angle = angle - 360

            # Determine whether there are obstacles in front, left, or right
            if 20 < angle < self.LaserAngle:
                if r < self.ResponseDist * 1.5:
                    self.Left_warning += 1

            if -self.LaserAngle < angle < -20:
                if r < self.ResponseDist * 1.5:
                    self.Right_warning += 1

            if abs(angle) <= 20:
                if r <= self.ResponseDist * 1.5:
                    self.front_warning += 1

        # If joystick is active or Switch is ON, do not let this node control the car
        if self.Joy_active or self.Switch is True:
            if self.Moving:
                self.publish_cmd(0.0, 0.0)
                self.Moving = False
            return

        self.Moving = True

        # According to the detected obstacles, release speed commands
        if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print("1, there are obstacles in the left and right, turn right")
            self.publish_cmd(self.linear, -self.angular)
            sleep(0.2)

        elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print("2, there is an obstacle in the middle right, turn left")
            self.publish_cmd(self.linear, self.angular)
            sleep(0.2)

            if self.Left_warning > 10 and self.Right_warning <= 10:
                self.publish_cmd(self.linear, -self.angular)
                sleep(0.5)

        elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print("4, there is an obstacle in the middle left, turn right")
            self.publish_cmd(self.linear, -self.angular)
            sleep(0.2)

            if self.Left_warning <= 10 and self.Right_warning > 10:
                self.publish_cmd(self.linear, self.angular)
                sleep(0.5)

        elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:
            print("6, there is an obstacle in the middle, turn left")
            self.publish_cmd(self.linear, self.angular)
            sleep(0.2)

        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print("7, there are obstacles on the left and right, turn right")
            self.publish_cmd(self.linear, -self.angular)
            sleep(0.4)

        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print("8, there is an obstacle on the left, turn right")
            self.publish_cmd(self.linear, -self.angular)
            sleep(0.2)

        elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print("9, there is an obstacle on the right, turn left")
            self.publish_cmd(self.linear, self.angular)
            sleep(0.2)

        elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            print("10, no obstacles, go forward")
            self.publish_cmd(self.linear, 0.0)


def main(args=None):
    rclpy.init(args=args)
    laser_avoid = laserAvoid("laser_Avoidance")

    try:
        rclpy.spin(laser_avoid)
    except KeyboardInterrupt:
        pass
    finally:
        laser_avoid.exit_pro()
        laser_avoid.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
