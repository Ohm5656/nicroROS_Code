# ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt16

# common lib
import os
import math
import numpy as np
from yahboomcar_laser.common import *

print("import done")
RAD2DEG = 180.0 / math.pi


class laserWarning(Node):
    def __init__(self, name):
        super().__init__(name)

        # subscribers
        self.sub_laser = self.create_subscription(
            LaserScan, "/scan", self.registerScan, 1
        )
        self.sub_JoyState = self.create_subscription(
            Bool, "/JoyState", self.JoyStateCallback, 1
        )

        # publishers
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_Buzzer = self.create_publisher(UInt16, "/beep", 1)

        # parameters
        self.declare_parameter("LaserAngle", 10.0)
        self.declare_parameter("ResponseDist", 0.3)
        self.declare_parameter("Switch", False)

        self.LaserAngle = self.get_parameter("LaserAngle").value
        self.ResponseDist = self.get_parameter("ResponseDist").value
        self.Switch = self.get_parameter("Switch").value

        self.Joy_active = False
        self.Moving = False

        self.ang_pid = SinglePID(3.0, 0.0, 5.0)

        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        self.Switch = self.get_parameter("Switch").value
        self.LaserAngle = self.get_parameter("LaserAngle").value
        self.ResponseDist = self.get_parameter("ResponseDist").value

    def JoyStateCallback(self, msg):
        if isinstance(msg, Bool):
            self.Joy_active = msg.data

    def publish_buzzer(self, state):
        b = UInt16()
        b.data = state
        self.pub_Buzzer.publish(b)

    def exit_pro(self):
        self.pub_vel.publish(Twist())
        self.publish_buzzer(0)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return

        if self.Joy_active or self.Switch:
            if self.Moving:
                print("stop")
                self.pub_vel.publish(Twist())
                self.publish_buzzer(0)
                self.Moving = False
            return

        self.Moving = True

        ranges = np.array(scan_data.ranges)

        minDistList = []
        minDistIDList = []

        for i in range(len(ranges)):
            r = ranges[i]

            # filter invalid
            if not math.isfinite(r) or r <= 0:
                continue

            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

            if angle > 180:
                angle -= 360

            if abs(angle) < self.LaserAngle:
                minDistList.append(r)
                minDistIDList.append(angle)

        if len(minDistList) == 0:
            return

        # nearest object
        minDist = min(minDistList)
        minDistID = minDistIDList[minDistList.index(minDist)]

        velocity = Twist()

        # PID control
        angle_pid = float(self.ang_pid.pid_compute(minDistID / 48.0, 0.0))

        if abs(angle_pid) < 0.1:
            velocity.angular.z = 0.0
        else:
            velocity.angular.z = angle_pid

        self.pub_vel.publish(velocity)

        print("minDist:", minDist)
        print("minDistID:", minDistID)

        # buzzer logic
        if minDist <= self.ResponseDist:
            print("⚠️ OBSTACLE")
            self.publish_buzzer(1)
        else:
            print("no obstacles")
            self.publish_buzzer(0)


def main():
    rclpy.init()
    laser_warn = laserWarning("laser_Warning")
    print("start it")
    try:
        rclpy.spin(laser_warn)
    except KeyboardInterrupt:
        pass
    finally:
        laser_warn.exit_pro()
        laser_warn.destroy_node()
        rclpy.shutdown()
