# ros lib
import os
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# common lib
from yahboomcar_laser.common import *

print("import done")
RAD2DEG = 180.0 / math.pi


class laserTracker(Node):
    def __init__(self, name):
        super().__init__(name)

        # create subscribers
        self.sub_laser = self.create_subscription(
            LaserScan, "/scan", self.registerScan, 1
        )
        self.sub_JoyState = self.create_subscription(
            Bool, "/JoyState", self.JoyStateCallback, 1
        )

        # create publisher
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)

        # declare params
        self.declare_parameter("priorityAngle", 10.0)
        self.priorityAngle = self.get_parameter("priorityAngle").get_parameter_value().double_value

        self.declare_parameter("LaserAngle", 15.0)
        self.LaserAngle = self.get_parameter("LaserAngle").get_parameter_value().double_value

        self.declare_parameter("ResponseDist", 0.55)
        self.ResponseDist = self.get_parameter("ResponseDist").get_parameter_value().double_value

        self.declare_parameter("Switch", False)
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value

        self.Joy_active = False
        self.Moving = False

        # PID
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)

        # refresh params
        self.timer = self.create_timer(0.01, self.on_timer)

        self.get_logger().info("laser_Tracker node started")

    def on_timer(self):
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value
        self.priorityAngle = self.get_parameter("priorityAngle").get_parameter_value().double_value
        self.LaserAngle = self.get_parameter("LaserAngle").get_parameter_value().double_value
        self.ResponseDist = self.get_parameter("ResponseDist").get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data

    def publish_zero(self):
        twist = Twist()
        self.pub_vel.publish(twist)

    def exit_pro(self):
        self.publish_zero()

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return

        ranges = np.array(scan_data.ranges)
        offset = 0.5

        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []

        # Process radar data
        for i in range(len(ranges)):
            r = ranges[i]

            # skip invalid values
            if not math.isfinite(r) or r <= 0.0:
                continue

            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

            # normalize angle to [-180, 180]
            if angle > 180.0:
                angle = angle - 360.0

            # priority zone: object in front has higher priority
            if abs(angle) < self.priorityAngle:
                if 0.0 < r < (self.ResponseDist + offset):
                    frontDistList.append(r)
                    frontDistIDList.append(angle)

            # normal tracking zone
            elif abs(angle) < self.LaserAngle:
                minDistList.append(r)
                minDistIDList.append(angle)

        # No valid object found
        if len(frontDistIDList) == 0 and len(minDistIDList) == 0:
            if self.Moving:
                self.publish_zero()
                self.Moving = False
            return

        # Find nearest object
        if len(frontDistIDList) != 0:
            minDist = min(frontDistList)
            minDistID = frontDistIDList[frontDistList.index(minDist)]
        else:
            minDist = min(minDistList)
            minDistID = minDistIDList[minDistList.index(minDist)]

        # If joystick is active or switch is on, stop this node from controlling
        if self.Joy_active or self.Switch is True:
            if self.Moving:
                self.publish_zero()
                self.Moving = False
            return

        self.Moving = True

        velocity = Twist()
        print("minDist:", minDist, "minDistID:", minDistID)

        # avoid jitter near target distance
        if abs(minDist - self.ResponseDist) < 0.1:
            minDist = self.ResponseDist

        # linear velocity
        velocity.linear.x = float(-self.lin_pid.pid_compute(self.ResponseDist, minDist))

        # angular velocity
        ang_pid_compute = float(self.ang_pid.pid_compute(minDistID / 48.0, 0.0))
        velocity.angular.z = ang_pid_compute

        if abs(ang_pid_compute) < 0.1:
            velocity.angular.z = 0.0

        self.pub_vel.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    laser_tracker = laserTracker("laser_Tracker")
    print("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        laser_tracker.exit_pro()
        laser_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
