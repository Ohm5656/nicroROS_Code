#!/usr/bin/env python3
# encoding: utf-8

import sys
import select
import termios
import tty
import time
import math
import yaml
import os
from typing import Any, Dict, List, Optional, Tuple

import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan


# =========================================================
# FIXED PATHS
# =========================================================
MAP_YAML = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/yahboomcar_nav/maps/yahboom_map.yaml"
MAP_IMAGE = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/yahboomcar_nav/maps/yahboom_map.pgm"
OUTPUT_YAML = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/waypoint_nav/waypoint_nav/nav_waypoints.yaml"

SCALE = 2


msg = """
Hybrid Check Points / Map + Keyboard
------------------------------------------------
[Map Window]
- Left click + drag : create waypoint + yaw from pixel_to_world
- i : toggle REVERSE mode for the next map point
- z : undo last waypoint
- c : clear all waypoints
- q : quit

[Keyboard in terminal]
Moving around:
    u    i    o
    j    k    l
    m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Other:
    s : save current robot pose as NEW waypoint
    v : undo last waypoint
    C : clear all waypoints
    p : print current pose
    b : toggle safety ON/OFF
    L : show lidar distances
    k / space : stop robot
    CTRL-C : save and quit

Workflow:
1) Use map window to create waypoint from pixel_to_world
2) Robot will drive to that point immediately
3) If you want an extra real waypoint (e.g. inspect point), drive by keyboard
4) Press 's' to append a NEW waypoint from real /amcl_pose
   and ask Inspect point? (y/n)
"""


moveBindings = {
    'i': (1, 0), 'o': (1, -1), 'j': (0, 1), 'l': (0, -1),
    'u': (1, 1), ',': (-1, 0), '.': (-1, 1), 'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1), 'z': (0.9, 0.9),
    'w': (1.1, 1.0), 'x': (0.9, 1.0),
    'e': (1.0, 1.1), 'c': (1.0, 0.9),
}


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return z, w


class CheckPointsNode(Node):
    def __init__(self, name: str):
        super().__init__(name)

        # ---------------- ROS interfaces ----------------
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10
        )

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # ---------------- parameters ----------------
        self.declare_parameter("linear_speed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)
        self.declare_parameter("frame_id", "map")

        # stable save pose
        self.declare_parameter("save_delay_sec", 1.2)
        self.declare_parameter("save_pose_samples", 15)

        # reverse behavior for testing map points
        self.declare_parameter("reverse_linear_speed", 0.12)
        self.declare_parameter("reverse_min_linear_speed", 0.04)
        self.declare_parameter("reverse_max_angular_speed", 0.6)
        self.declare_parameter("reverse_k_linear", 0.9)
        self.declare_parameter("reverse_k_angular", 1.0)
        self.declare_parameter("reverse_xy_tolerance", 0.06)
        self.declare_parameter("reverse_rear_safety_distance", 0.10)
        self.declare_parameter("reverse_control_dt", 0.05)
        self.declare_parameter("reverse_max_segment_time_sec", 40.0)
        self.declare_parameter("rotate_only_threshold_rad", 1.4)

        self.linear_speed_limit = (
            self.get_parameter("linear_speed_limit").get_parameter_value().double_value
        )
        self.angular_speed_limit = (
            self.get_parameter("angular_speed_limit").get_parameter_value().double_value
        )
        self.frame_id = self.get_parameter("frame_id").value

        self.save_delay_sec = (
            self.get_parameter("save_delay_sec").get_parameter_value().double_value
        )
        self.save_pose_samples = (
            self.get_parameter("save_pose_samples").get_parameter_value().integer_value
        )

        self.reverse_linear_speed = float(self.get_parameter("reverse_linear_speed").value)
        self.reverse_min_linear_speed = float(self.get_parameter("reverse_min_linear_speed").value)
        self.reverse_max_angular_speed = float(self.get_parameter("reverse_max_angular_speed").value)
        self.reverse_k_linear = float(self.get_parameter("reverse_k_linear").value)
        self.reverse_k_angular = float(self.get_parameter("reverse_k_angular").value)
        self.reverse_xy_tolerance = float(self.get_parameter("reverse_xy_tolerance").value)
        self.reverse_rear_safety_distance = float(self.get_parameter("reverse_rear_safety_distance").value)
        self.reverse_control_dt = float(self.get_parameter("reverse_control_dt").value)
        self.reverse_max_segment_time_sec = float(self.get_parameter("reverse_max_segment_time_sec").value)
        self.rotate_only_threshold_rad = float(self.get_parameter("rotate_only_threshold_rad").value)

        self.settings = termios.tcgetattr(sys.stdin)

        # ---------------- lidar safety ----------------
        self.dist_front = 10.0
        self.dist_back = 10.0
        self.dist_left = 10.0
        self.dist_right = 10.0
        self.safety_limit = 0.3
        self.last_beep_time = 0.0
        self.enable_safety = False

        # ---------------- pose ----------------
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.pose_received = False

        # ---------------- session waypoints ----------------
        self.session_waypoints: List[Dict[str, Any]] = []

        # ---------------- map UI state ----------------
        self.original = None
        self.display = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.start_point = None
        self.reverse_mode = False
        self.pending_drive_index: Optional[int] = None

        self.load_map()
        self.redraw()

    # =========================================================
    # Pose / TF helpers
    # =========================================================
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.current_x = p.x
        self.current_y = p.y
        self.current_yaw = yaw
        self.pose_received = True

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    # =========================================================
    # Lidar safety
    # =========================================================
    def scan_callback(self, msg: LaserScan):
        num_points = len(msg.ranges)

        b_idx = list(range(0, 31)) + list(range(num_points - 30, num_points))
        r_idx = list(range(60, 121))
        f_idx = list(range(150, 211))
        l_idx = list(range(240, 301))

        def get_min_dist(indices):
            ranges = [
                msg.ranges[i]
                for i in indices
                if i < len(msg.ranges) and 0.05 < msg.ranges[i] < 3.0
            ]
            return min(ranges) if ranges else 10.0

        self.dist_front = get_min_dist(f_idx)
        self.dist_left = get_min_dist(l_idx)
        self.dist_back = get_min_dist(b_idx)
        self.dist_right = get_min_dist(r_idx)

    def play_warning_sound(self):
        current_time = time.time()
        if current_time - self.last_beep_time > 0.5:
            sys.stdout.write('\a')
            sys.stdout.flush()
            self.last_beep_time = current_time

    def print_lidar(self):
        print(
            f"[LIDAR] front={self.dist_front:.2f} m | "
            f"left={self.dist_left:.2f} m | "
            f"back={self.dist_back:.2f} m | "
            f"right={self.dist_right:.2f} m"
        )

    def toggle_safety(self):
        self.enable_safety = not self.enable_safety
        print(f"[SAFETY] {'ON' if self.enable_safety else 'OFF'}")

    # =========================================================
    # Map loading / drawing
    # =========================================================
    def load_map(self):
        if not os.path.exists(MAP_YAML):
            raise FileNotFoundError(f"ไม่พบไฟล์ MAP_YAML: {MAP_YAML}")

        if not os.path.exists(MAP_IMAGE):
            raise FileNotFoundError(f"ไม่พบไฟล์ MAP_IMAGE: {MAP_IMAGE}")

        with open(MAP_YAML, "r", encoding="utf-8") as f:
            map_data = yaml.safe_load(f)

        self.resolution = map_data["resolution"]
        self.origin_x, self.origin_y, _ = map_data["origin"]

        img = cv2.imread(MAP_IMAGE)
        if img is None:
            raise RuntimeError(f"โหลดไฟล์แมพไม่ได้: {MAP_IMAGE}")

        self.original = img.copy()
        self.display = cv2.resize(
            self.original, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST
        )

    def pixel_to_world(self, x, y):
        x = x / SCALE
        y = y / SCALE

        h = self.original.shape[0]
        y = h - y

        world_x = x * self.resolution + self.origin_x
        world_y = y * self.resolution + self.origin_y
        return world_x, world_y

    def world_to_pixel(self, wx, wy):
        px = int((wx - self.origin_x) / self.resolution)
        py = int((wy - self.origin_y) / self.resolution)
        py = self.original.shape[0] - py

        px = int(px * SCALE)
        py = int(py * SCALE)
        return px, py

    @staticmethod
    def compute_yaw(x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def print_mode_status(self):
        if self.reverse_mode:
            print("[MODE] REVERSE = ON  -> จุดถัดไปจะ reverse_from_prev = True")
        else:
            print("[MODE] REVERSE = OFF -> จุดถัดไปจะ reverse_from_prev = False")

    def redraw(self):
        self.display = cv2.resize(
            self.original, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST
        )

        for i, wp in enumerate(self.session_waypoints, start=1):
            px, py = self.world_to_pixel(wp["x"], wp["y"])

            is_inspect = bool(wp.get("inspect", False))
            is_reverse = bool(wp.get("reverse_from_prev", False))

            point_color = (0, 255, 255) if is_inspect else (0, 255, 0)

            cv2.circle(self.display, (px, py), 6, point_color, -1)

            if is_inspect:
                cv2.circle(self.display, (px, py), 12, (0, 255, 255), 2)

            if is_reverse:
                cv2.circle(self.display, (px, py), 16, (255, 0, 255), 2)

            label = f"{i}"
            if is_inspect:
                label += "*"
            if is_reverse:
                label += "R"

            cv2.putText(
                self.display,
                label,
                (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1,
                cv2.LINE_AA,
            )

            yaw = wp["yaw"]
            arrow_len = 25
            end_x = int(px + arrow_len * math.cos(yaw))
            end_y = int(py - arrow_len * math.sin(yaw))
            cv2.arrowedLine(self.display, (px, py), (end_x, end_y), (0, 0, 255), 2)

    # =========================================================
    # YAML save
    # =========================================================
    def save_all_to_file(self):
        data = {"waypoints": self.session_waypoints}
        with open(OUTPUT_YAML, "w", encoding="utf-8") as f:
            yaml.dump(data, f, sort_keys=False, allow_unicode=True)
        print(f"[SAVED] {OUTPUT_YAML} ({len(self.session_waypoints)} points)")

    # =========================================================
    # Keyboard input
    # =========================================================
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def ask_yes_no(self, question: str) -> bool:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        while True:
            ans = input(question).strip().lower()
            if ans == 'y':
                return True
            if ans == 'n':
                return False
            print("กรุณาพิมพ์ y หรือ n เท่านั้น")

    # =========================================================
    # Robot motion helpers
    # =========================================================
    def stop_robot(self):
        self.pub_cmd.publish(Twist())

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub_cmd.publish(msg)

    def print_current_pose(self):
        if not self.pose_received:
            print("[POSE] waiting /amcl_pose ...")
            return

        print(
            f"[POSE] x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}"
        )

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

    def build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        z, w = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        return pose

    def wait_for_pose(self, timeout_sec: float = 3.0) -> bool:
        end_time = time.time() + timeout_sec
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose_received:
                return True
            time.sleep(0.02)
        return False

    def navigate_one_waypoint(self, wp: Dict[str, Any]) -> bool:
        self.get_logger().info(
            f"[NAV] ไป {wp['task']} -> "
            f"x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}"
        )

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("ไม่พบ action server /navigate_to_pose")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.build_pose(wp["x"], wp["y"], wp["yaw"])

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            self.get_logger().error("ส่ง goal ไม่สำเร็จ")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("goal ถูก reject")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error("ไม่ได้ผลลัพธ์จาก action")
            return False

        status = result_wrapper.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[NAV] ถึง {wp['task']} สำเร็จ")
            return True

        self.get_logger().warn(f"[NAV] {wp['task']} จบด้วย status code: {status}")
        return False

    def reverse_to_waypoint(self, wp: Dict[str, Any]) -> bool:
        if not self.wait_for_pose():
            self.get_logger().error("[REVERSE] ยังไม่ได้รับ /amcl_pose")
            return False

        self.get_logger().info(
            f"[REVERSE] ไปยัง {wp['task']} แบบถอยหลัง -> "
            f"x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}"
        )

        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < self.reverse_max_segment_time_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

            if not self.pose_received:
                continue

            dx = wp["x"] - self.current_x
            dy = wp["y"] - self.current_y
            dist = math.hypot(dx, dy)

            if dist <= self.reverse_xy_tolerance:
                self.stop_robot()
                self.get_logger().info(f"[REVERSE] ถึง {wp['task']} แล้ว")
                return True

            if self.dist_back < self.reverse_rear_safety_distance:
                self.stop_robot()
                self.get_logger().error(
                    f"[REVERSE] ด้านหลังใกล้สิ่งกีดขวางเกินไป "
                    f"({self.dist_back:.2f} m < {self.reverse_rear_safety_distance:.2f} m)"
                )
                return False

            path_heading = math.atan2(dy, dx)
            reverse_heading = self.normalize_angle(path_heading + math.pi)
            heading_error = self.normalize_angle(reverse_heading - self.current_yaw)

            angular = max(
                -self.reverse_max_angular_speed,
                min(self.reverse_max_angular_speed, self.reverse_k_angular * heading_error)
            )

            if abs(heading_error) > self.rotate_only_threshold_rad:
                linear = 0.0
            else:
                linear_mag = min(self.reverse_linear_speed, self.reverse_k_linear * dist)
                linear_mag = max(self.reverse_min_linear_speed, linear_mag)
                linear = -linear_mag

            self.publish_cmd(linear, angular)
            time.sleep(self.reverse_control_dt)

        self.stop_robot()
        self.get_logger().error(
            f"[REVERSE] timeout: ไป {wp['task']} ไม่สำเร็จภายใน {self.reverse_max_segment_time_sec:.1f} sec"
        )
        return False

    def drive_to_waypoint_by_index(self, idx: int):
        if idx < 0 or idx >= len(self.session_waypoints):
            return

        wp = self.session_waypoints[idx]

        if idx == 0 and wp.get("reverse_from_prev", False):
            self.get_logger().warn(
                f"{wp['task']} เป็น waypoint แรก จึง reverse_from_prev ไม่ได้ จะใช้ nav ปกติ"
            )
            ok = self.navigate_one_waypoint(wp)
        else:
            if wp.get("reverse_from_prev", False):
                ok = self.reverse_to_waypoint(wp)
            else:
                ok = self.navigate_one_waypoint(wp)

        if ok:
            print(f"[DRIVE DONE] {wp['task']} done")
        else:
            print(f"[DRIVE FAIL] {wp['task']} failed")

    # =========================================================
    # Stable save from keyboard: APPEND new waypoint
    # =========================================================
    def get_stable_average_pose(self, samples: int = None, delay_sec: float = None):
        if samples is None:
            samples = int(self.save_pose_samples)
        if delay_sec is None:
            delay_sec = float(self.save_delay_sec)

        if (
            not self.pose_received
            or self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            print("[ERROR] ยังไม่ได้รับ /amcl_pose จึงยังเซฟจุดไม่ได้")
            return None

        self.stop_robot()

        print(f"[SAVE] Stop robot and wait {delay_sec:.1f} sec for pose to settle...")
        settle_start = time.time()
        while rclpy.ok() and (time.time() - settle_start) < delay_sec:
            self.stop_robot()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)

        xs = []
        ys = []
        sin_sum = 0.0
        cos_sum = 0.0

        print(f"[SAVE] Collecting {samples} pose samples for averaging...")
        collected = 0
        attempts = 0
        max_attempts = max(samples * 5, 50)

        while rclpy.ok() and collected < samples and attempts < max_attempts:
            rclpy.spin_once(self, timeout_sec=0.05)
            attempts += 1

            if (
                self.pose_received
                and self.current_x is not None
                and self.current_y is not None
                and self.current_yaw is not None
            ):
                xs.append(float(self.current_x))
                ys.append(float(self.current_y))
                sin_sum += math.sin(float(self.current_yaw))
                cos_sum += math.cos(float(self.current_yaw))
                collected += 1

            self.stop_robot()
            time.sleep(0.03)

        if collected == 0:
            print("[ERROR] เก็บ pose sample ไม่สำเร็จ")
            return None

        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_yaw = math.atan2(sin_sum / collected, cos_sum / collected)
        avg_yaw = self.normalize_angle(avg_yaw)

        print(
            f"[SAVE] Averaged pose from {collected} samples -> "
            f"x={avg_x:.3f}, y={avg_y:.3f}, yaw={avg_yaw:.3f}"
        )

        return avg_x, avg_y, avg_yaw

    def save_current_pose_as_new_waypoint(self):
        stable_pose = self.get_stable_average_pose()
        if stable_pose is None:
            return

        avg_x, avg_y, avg_yaw = stable_pose
        inspect = self.ask_yes_no("Inspect point? (y/n): ")

        waypoint = {
            "task": f"waypoint_{len(self.session_waypoints) + 1}",
            "x": round(avg_x, 3),
            "y": round(avg_y, 3),
            "yaw": round(avg_yaw, 3),
            "inspect": inspect,
            "reverse_from_prev": False,
        }

        self.session_waypoints.append(waypoint)

        print("-" * 60)
        print(f"[SAVE NEW] {waypoint['task']}")
        print(f"           x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}")
        print(f"           inspect={waypoint['inspect']}")
        print(f"           reverse_from_prev={waypoint['reverse_from_prev']}")
        print("-" * 60)

        self.save_all_to_file()
        self.redraw()
        cv2.imshow("map", self.display)

    # =========================================================
    # Waypoint list operations
    # =========================================================
    def add_map_waypoint(self, wx: float, wy: float, yaw: float):
        reverse_from_prev = self.reverse_mode if len(self.session_waypoints) > 0 else False

        waypoint = {
            "task": f"waypoint_{len(self.session_waypoints) + 1}",
            "x": round(wx, 3),
            "y": round(wy, 3),
            "yaw": round(yaw, 3),
            "inspect": False,
            "reverse_from_prev": reverse_from_prev,
        }

        self.session_waypoints.append(waypoint)

        print("-" * 60)
        print(f"[ADD MAP] {waypoint['task']}")
        print(f"          x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}")
        print(f"          inspect={waypoint['inspect']}")
        print(f"          reverse_from_prev={waypoint['reverse_from_prev']}")
        print("-" * 60)

        self.save_all_to_file()
        self.redraw()
        cv2.imshow("map", self.display)

        self.pending_drive_index = len(self.session_waypoints) - 1

    def undo_last_waypoint(self):
        if not self.session_waypoints:
            print("[UNDO] ไม่มี waypoint ใน session ให้ลบ")
            return

        removed = self.session_waypoints.pop()
        print(f"[UNDO] ลบจุดล่าสุด: {removed}")
        self.save_all_to_file()
        self.redraw()
        cv2.imshow("map", self.display)

    def clear_session_waypoints(self):
        self.session_waypoints.clear()
        print("[CLEAR] ล้าง waypoint ทั้งหมดแล้ว")
        self.save_all_to_file()
        self.redraw()
        cv2.imshow("map", self.display)

    # =========================================================
    # Mouse callback for map
    # =========================================================
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.start_point = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and self.start_point is not None:
            temp = self.display.copy()
            cv2.arrowedLine(temp, self.start_point, (x, y), (0, 0, 255), 2)
            cv2.imshow("map", temp)

        elif event == cv2.EVENT_LBUTTONUP and self.start_point is not None:
            end_point = (x, y)

            wx1, wy1 = self.pixel_to_world(*self.start_point)
            wx2, wy2 = self.pixel_to_world(*end_point)
            yaw = self.compute_yaw(wx1, wy1, wx2, wy2)

            self.add_map_waypoint(wx1, wy1, yaw)

            self.start_point = None
            self.redraw()
            cv2.imshow("map", self.display)

    # =========================================================
    # Map key handling
    # =========================================================
    def handle_map_key(self, key: int) -> bool:
        if key == ord("q"):
            return False

        elif key == ord("i"):
            self.reverse_mode = not self.reverse_mode
            self.print_mode_status()
            self.redraw()
            cv2.imshow("map", self.display)

        elif key == ord("z"):
            self.undo_last_waypoint()

        elif key == ord("c"):
            self.clear_session_waypoints()

        return True


def main():
    rclpy.init()
    node = CheckPointsNode("check_points_ctrl")

    speed = 0.15
    turn = 0.8
    x = 0
    th = 0
    status = 0
    count = 0

    try:
        cv2.namedWindow("map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("map", 1400, 900)
        cv2.imshow("map", node.display)
        cv2.setMouseCallback("map", node.mouse_callback)

        print(msg)
        print(node.vels(speed, turn))
        print("[INFO] Session starts empty")
        print(f"[INFO] Save file: {OUTPUT_YAML}")
        print(f"[INFO] Safety initial state: {'ON' if node.enable_safety else 'OFF'}")
        print(
            f"[INFO] Stable save mode: delay={node.save_delay_sec:.1f}s, "
            f"samples={node.save_pose_samples}"
        )
        node.print_mode_status()

        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.01)

            # map window key
            map_key = cv2.waitKey(1) & 0xFF
            if map_key != 255:
                running = node.handle_map_key(map_key)
                if not running:
                    break

            # if new point was added by map, drive robot to that point
            if node.pending_drive_index is not None:
                idx = node.pending_drive_index
                node.pending_drive_index = None
                node.drive_to_waypoint_by_index(idx)
                node.stop_robot()

            # terminal keyboard teleop
            key = node.getKey()

            if key == 's':
                node.stop_robot()
                node.print_current_pose()
                node.save_current_pose_as_new_waypoint()
                count = 0
                x, th = 0, 0
                continue

            elif key == 'v':
                node.undo_last_waypoint()
                count = 0
                continue

            elif key == 'C':
                node.clear_session_waypoints()
                count = 0
                continue

            elif key == 'p':
                node.print_current_pose()
                count = 0
                continue

            elif key == 'b':
                node.toggle_safety()
                count = 0
                continue

            elif key == 'L':
                node.print_lidar()
                count = 0
                continue

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                speed = min(speed, node.linear_speed_limit)
                turn = min(turn, node.angular_speed_limit)

                print(node.vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
                count = 0

            elif key == ' ' or key == 'k':
                x, th = 0, 0

            elif key == '\x03':  # Ctrl+C
                break

            else:
                count += 1
                if count > 4:
                    x, th = 0, 0

            twist = Twist()
            target_linear = speed * x
            target_angular = turn * th
            is_blocked = False

            if node.enable_safety:
                if x > 0 and node.dist_front < node.safety_limit:
                    target_linear, is_blocked = 0.0, True
                    print(f"\r[STOP] Front Blocked ({node.dist_front:.2f}m)", end="")

                elif x < 0 and node.dist_back < node.safety_limit:
                    target_linear, is_blocked = 0.0, True
                    print(f"\r[STOP] Rear Blocked ({node.dist_back:.2f}m) ", end="")

                if th > 0 and node.dist_left < node.safety_limit:
                    target_linear, is_blocked = 0.0, True
                    print(f"\r[STOP] Left Blocked ({node.dist_left:.2f}m)  ", end="")

                elif th < 0 and node.dist_right < node.safety_limit:
                    target_linear, is_blocked = 0.0, True
                    print(f"\r[STOP] Right Blocked ({node.dist_right:.2f}m) ", end="")

                if is_blocked:
                    node.play_warning_sound()

            twist.linear.x = float(target_linear)
            twist.angular.z = float(target_angular)
            node.pub_cmd.publish(twist)

            sys.stdout.flush()

    except Exception as e:
        print(f"Error: {e}")

    finally:
        node.stop_robot()
        node.save_all_to_file()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
