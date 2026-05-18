#!/usr/bin/env python3

import math
import os
import time
import yaml
from collections import Counter, defaultdict
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32


WAYPOINTS_YAML = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/waypoint_nav/waypoint_nav/nav_waypoints.yaml"


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return z, w


class RunWaypointsNode(Node):
    def __init__(self):
        super().__init__("run_waypoints")

        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("waypoints_file", WAYPOINTS_YAML)

        # logic timing
        self.declare_parameter("settle_before_inspect_sec", 1.0)
        self.declare_parameter("inspect_wait_sec", 2.0)
        self.declare_parameter("inspect_spin_dt", 0.1)
        self.declare_parameter("after_drop_wait_sec", 1.0)
        self.declare_parameter("settle_after_every_waypoint_sec", 1.0)  # เพิ่มใหม่

        self.declare_parameter("person_hit_threshold", 1)
        self.declare_parameter("min_color_confidence", 1)

        # servo
        self.declare_parameter("servo_open_angle", -120)
        self.declare_parameter("servo_close_angle", 0)
        self.declare_parameter("servo_open_hold_sec", 1.0)

        # reverse behavior
        self.declare_parameter("reverse_linear_speed", 0.12)
        self.declare_parameter("reverse_min_linear_speed", 0.04)
        self.declare_parameter("reverse_max_angular_speed", 0.6)
        self.declare_parameter("reverse_k_linear", 0.9)
        self.declare_parameter("reverse_k_angular", 1.0)
        self.declare_parameter("reverse_xy_tolerance", 0.06)
        self.declare_parameter("reverse_yaw_tolerance", 0.12)
        self.declare_parameter("reverse_rear_safety_distance", 0.10)
        self.declare_parameter("reverse_control_dt", 0.05)
        self.declare_parameter("reverse_max_segment_time_sec", 40.0)
        self.declare_parameter("rotate_only_threshold_rad", 1.4)
        self.declare_parameter("use_final_yaw_after_reverse", False)

        self.action_name = self.get_parameter("action_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.waypoints_file = self.get_parameter("waypoints_file").value

        self.settle_before_inspect_sec = float(
            self.get_parameter("settle_before_inspect_sec").value
        )
        self.inspect_wait_sec = float(self.get_parameter("inspect_wait_sec").value)
        self.inspect_spin_dt = float(self.get_parameter("inspect_spin_dt").value)
        self.after_drop_wait_sec = float(
            self.get_parameter("after_drop_wait_sec").value
        )
        self.settle_after_every_waypoint_sec = float(  # เพิ่มใหม่
            self.get_parameter("settle_after_every_waypoint_sec").value
        )

        self.person_hit_threshold = int(
            self.get_parameter("person_hit_threshold").value
        )
        self.min_color_confidence = int(
            self.get_parameter("min_color_confidence").value
        )

        self.servo_open_angle = int(self.get_parameter("servo_open_angle").value)
        self.servo_close_angle = int(self.get_parameter("servo_close_angle").value)
        self.servo_open_hold_sec = float(
            self.get_parameter("servo_open_hold_sec").value
        )

        self.reverse_linear_speed = float(
            self.get_parameter("reverse_linear_speed").value
        )
        self.reverse_min_linear_speed = float(
            self.get_parameter("reverse_min_linear_speed").value
        )
        self.reverse_max_angular_speed = float(
            self.get_parameter("reverse_max_angular_speed").value
        )
        self.reverse_k_linear = float(self.get_parameter("reverse_k_linear").value)
        self.reverse_k_angular = float(self.get_parameter("reverse_k_angular").value)
        self.reverse_xy_tolerance = float(
            self.get_parameter("reverse_xy_tolerance").value
        )
        self.reverse_yaw_tolerance = float(
            self.get_parameter("reverse_yaw_tolerance").value
        )
        self.reverse_rear_safety_distance = float(
            self.get_parameter("reverse_rear_safety_distance").value
        )
        self.reverse_control_dt = float(
            self.get_parameter("reverse_control_dt").value
        )
        self.reverse_max_segment_time_sec = float(
            self.get_parameter("reverse_max_segment_time_sec").value
        )
        self.rotate_only_threshold_rad = float(
            self.get_parameter("rotate_only_threshold_rad").value
        )
        self.use_final_yaw_after_reverse = bool(
            self.get_parameter("use_final_yaw_after_reverse").value
        )

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name,
        )

        self.selected_waypoints: List[Dict[str, Any]] = []

        # latest values from vision topics
        self.latest_tag = "None"
        self.person_detected = False
        self.shirt_color = "Unknown"
        self.shirt_confidence = 0

        # pose + lidar
        self.pose_received = False
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.rear_min_dist = 10.0

        # publishers
        self.pub_servo = self.create_publisher(Int32, "servo_s1", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        # subscribers
        self.create_subscription(
            String,
            "/vision/latest_at_id",
            self.apriltag_callback,
            10,
        )
        self.create_subscription(
            String,
            "/vision/person_detected",
            self.person_callback,
            10,
        )
        self.create_subscription(
            String,
            "/vision/shirt_color",
            self.shirt_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.already_dropped = False

        self.get_logger().info("=== Run Waypoints Node Started ===")
        self.get_logger().info(f"Using action: {self.action_name}")
        self.get_logger().info(f"Using frame_id: {self.frame_id}")
        self.get_logger().info(f"Using waypoints_file: {self.waypoints_file}")
        self.get_logger().info(
            f"Settle before inspect: {self.settle_before_inspect_sec:.1f} sec"
        )
        self.get_logger().info(
            f"Inspection window: {self.inspect_wait_sec:.1f} sec"
        )
        self.get_logger().info(
            f"After drop wait: {self.after_drop_wait_sec:.1f} sec"
        )
        self.get_logger().info(
            f"Settle after every waypoint: {self.settle_after_every_waypoint_sec:.1f} sec"
        )
        self.get_logger().info(
            f"Servo open={self.servo_open_angle}, close={self.servo_close_angle}, hold={self.servo_open_hold_sec:.1f} sec"
        )
        self.get_logger().info(
            f"Reverse mode: v={self.reverse_linear_speed:.2f}, "
            f"xy_tol={self.reverse_xy_tolerance:.2f}, "
            f"rear_safe={self.reverse_rear_safety_distance:.2f}"
        )

    # =========================================================
    # Helpers
    # =========================================================
    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_x = p.x
        self.current_y = p.y
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_received = True

    def scan_callback(self, msg: LaserScan) -> None:
        num_points = len(msg.ranges)
        if num_points == 0:
            self.rear_min_dist = 10.0
            return

        back_indices = list(range(0, 31)) + list(range(max(0, num_points - 30), num_points))
        ranges = [
            msg.ranges[i]
            for i in back_indices
            if i < len(msg.ranges) and 0.05 < msg.ranges[i] < 3.0
        ]
        self.rear_min_dist = min(ranges) if ranges else 10.0

    # =========================================================
    # Vision callbacks
    # =========================================================
    def apriltag_callback(self, msg: String) -> None:
        raw = msg.data.strip()
        self.latest_tag = raw if raw else "None"

    def person_callback(self, msg: String) -> None:
        self.person_detected = msg.data.strip().lower() == "true"

    def shirt_callback(self, msg: String) -> None:
        raw = msg.data.strip()
        try:
            color, conf = raw.split("|")
            self.shirt_color = color.strip() if color.strip() else "Unknown"
            self.shirt_confidence = int(conf)
        except Exception:
            self.shirt_color = raw if raw else "Unknown"
            self.shirt_confidence = 0

    # =========================================================
    # Basic robot hold / stop
    # =========================================================
    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def hold_robot(self, hold_sec: float, reason: str) -> None:
        self.get_logger().info(f"[HOLD] {reason} {hold_sec:.1f} sec")
        end_time = time.time() + hold_sec

        while time.time() < end_time and rclpy.ok():
            self.publish_stop()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.publish_stop()

    # =========================================================
    # YAML
    # =========================================================
    def load_waypoints_from_yaml(self) -> bool:
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(f"ไม่พบไฟล์ waypoint: {self.waypoints_file}")
            return False

        try:
            with open(self.waypoints_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            waypoints_raw = data.get("waypoints", [])
            if not waypoints_raw:
                self.get_logger().error("ไม่มี waypoint ในไฟล์ YAML")
                return False

            self.selected_waypoints = []
            for i, wp in enumerate(waypoints_raw, start=1):
                item = {
                    "task": str(wp.get("task", f"waypoint_{i}")),
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp["yaw"]),
                    "inspect": bool(wp.get("inspect", False)),
                    "reverse_from_prev": bool(wp.get("reverse_from_prev", False)),
                }
                self.selected_waypoints.append(item)

                self.get_logger().info(
                    f"Waypoint {i}: task={item['task']}, "
                    f"x={item['x']:.3f}, y={item['y']:.3f}, "
                    f"yaw={item['yaw']:.3f}, inspect={item['inspect']}, "
                    f"reverse_from_prev={item['reverse_from_prev']}"
                )

            self.get_logger().info(
                f"โหลด waypoint ได้ทั้งหมด {len(self.selected_waypoints)} จุด"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"โหลดไฟล์ YAML ไม่สำเร็จ: {e}")
            return False

    # =========================================================
    # Pose builder
    # =========================================================
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

    # =========================================================
    # Normal Nav2
    # =========================================================
    def navigate_one_waypoint(self, wp: Dict[str, Any]) -> bool:
        self.get_logger().info(
            f"กำลังไป {wp['task']} -> "
            f"x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}, inspect={wp['inspect']}"
        )

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"ไม่พบ action server {self.action_name}")
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

        self.get_logger().info(f"{wp['task']} goal accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error("ไม่ได้ผลลัพธ์จาก action")
            return False

        status = result_wrapper.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"ถึง {wp['task']} สำเร็จ")
            return True

        self.get_logger().warn(f"{wp['task']} action จบด้วย status code: {status}")
        return False

    # =========================================================
    # Reverse behavior
    # =========================================================
    def wait_for_pose(self, timeout_sec: float = 3.0) -> bool:
        end_time = time.time() + timeout_sec
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose_received:
                return True
            time.sleep(0.02)
        return False

    def rotate_to_yaw(self, target_yaw: float, tolerance: float = None, timeout_sec: float = 20.0) -> bool:
        if tolerance is None:
            tolerance = self.reverse_yaw_tolerance

        if not self.wait_for_pose():
            self.get_logger().error("ยังไม่ได้รับ /amcl_pose สำหรับ rotate_to_yaw")
            return False

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) <= tolerance:
                self.publish_stop()
                return True

            angular = max(-self.reverse_max_angular_speed,
                          min(self.reverse_max_angular_speed, 1.6 * yaw_error))

            if abs(angular) < 0.15:
                angular = 0.15 if angular >= 0.0 else -0.15

            self.publish_cmd(0.0, angular)
            time.sleep(self.reverse_control_dt)

        self.publish_stop()
        return False

    def reverse_to_waypoint(self, wp: Dict[str, Any]) -> bool:
        if not self.wait_for_pose():
            self.get_logger().error("ยังไม่ได้รับ /amcl_pose จึงถอยไม่ได้")
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
                self.publish_stop()
                self.get_logger().info(
                    f"[REVERSE] ถึงตำแหน่ง {wp['task']} แล้ว (xy tolerance)"
                )
                break

            if self.rear_min_dist < self.reverse_rear_safety_distance:
                self.publish_stop()
                self.get_logger().error(
                    f"[REVERSE] ด้านหลังใกล้สิ่งกีดขวางเกินไป "
                    f"({self.rear_min_dist:.2f} m < {self.reverse_rear_safety_distance:.2f} m)"
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

        else:
            self.publish_stop()
            self.get_logger().error(
                f"[REVERSE] timeout: ใช้เวลาถอยไป {wp['task']} นานเกิน {self.reverse_max_segment_time_sec:.1f} sec"
            )
            return False

        self.publish_stop()

        if self.use_final_yaw_after_reverse:
            self.get_logger().info(
                f"[REVERSE] หมุนจัด yaw สุดท้ายให้ตรงกับ waypoint ({wp['yaw']:.3f})"
            )
            ok = self.rotate_to_yaw(wp["yaw"])
            if not ok:
                self.get_logger().warn("[REVERSE] จัด yaw สุดท้ายไม่สำเร็จ แต่จะไปต่อ")
        return True

    # =========================================================
    # Inspection helpers
    # =========================================================
    def _is_valid_tag(self, tag: str) -> bool:
        if tag is None:
            return False
        t = tag.strip().lower()
        return t not in ("", "none", "waiting for tag...", "waiting")

    def _choose_best_tag(self, tags: List[str]) -> Tuple[str, int]:
        valid_tags = [t for t in tags if self._is_valid_tag(t)]
        if not valid_tags:
            return "None", 0

        counter = Counter(valid_tags)
        best_tag, best_count = counter.most_common(1)[0]
        return best_tag, best_count

    def _choose_best_color(
        self, color_samples: List[Tuple[str, int]]
    ) -> Tuple[str, int, int]:
        valid: List[Tuple[str, int]] = []
        for color, conf in color_samples:
            c = (color or "").strip()
            if c and c.lower() != "unknown" and int(conf) >= self.min_color_confidence:
                valid.append((c, max(0, int(conf))))

        if not valid:
            return "Unknown", 0, 0

        count_map = Counter()
        conf_sum_map = defaultdict(int)
        max_conf_map = defaultdict(int)

        for color, conf in valid:
            count_map[color] += 1
            conf_sum_map[color] += conf
            if conf > max_conf_map[color]:
                max_conf_map[color] = conf

        best_color = None
        best_key = None

        for color in count_map:
            count = count_map[color]
            avg_conf = conf_sum_map[color] / count if count > 0 else 0
            max_conf = max_conf_map[color]
            key = (count, avg_conf, max_conf)

            if best_key is None or key > best_key:
                best_key = key
                best_color = color

        final_count = count_map[best_color]
        final_avg_conf = int(conf_sum_map[best_color] / final_count)
        return best_color, final_avg_conf, final_count

    def _summarize_person(self, person_samples: List[bool]) -> Tuple[bool, int]:
        person_hits = sum(1 for p in person_samples if p)
        person_found = person_hits >= self.person_hit_threshold
        return person_found, person_hits

    # =========================================================
    # Servo drop
    # =========================================================
    def drop_cube(self) -> None:
        self.get_logger().info("[DROP] เริ่มปล่อยลูกบาศก์ด้วย servo_s1")
        self.publish_stop()

        msg = Int32()

        msg.data = self.servo_open_angle
        self.pub_servo.publish(msg)
        self.get_logger().info(f"[DROP] Servo open -> {msg.data}")
        time.sleep(self.servo_open_hold_sec)

        msg.data = self.servo_close_angle
        self.pub_servo.publish(msg)
        self.get_logger().info(f"[DROP] Servo close -> {msg.data}")

        self.publish_stop()

    # =========================================================
    # Inspection
    # =========================================================
    def inspect_current_scene(self, wp: Dict[str, Any]) -> None:
        self.already_dropped = False

        self.hold_robot(
            self.settle_before_inspect_sec,
            "ถึงจุดแล้วหยุดนิ่งก่อนตรวจ",
        )

        self.get_logger().info(
            f"[INSPECT] {wp['task']} -> เก็บข้อมูล {self.inspect_wait_sec:.1f} วินาที"
        )

        end_time = time.time() + self.inspect_wait_sec

        person_samples: List[bool] = []
        tag_samples: List[str] = []
        color_samples: List[Tuple[str, int]] = []

        while time.time() < end_time and rclpy.ok():
            self.publish_stop()
            rclpy.spin_once(self, timeout_sec=self.inspect_spin_dt)

            person_samples.append(self.person_detected)
            tag_samples.append(self.latest_tag)
            color_samples.append((self.shirt_color, self.shirt_confidence))

        total_samples = len(person_samples)

        person_found, person_hits = self._summarize_person(person_samples)
        best_tag, best_tag_hits = self._choose_best_tag(tag_samples)
        best_color, best_color_conf, best_color_hits = self._choose_best_color(
            color_samples
        )

        print("\n" + "=" * 72)
        print(f"INSPECTION RESULT @ {wp['task']}")
        print("=" * 72)
        print(f"Position         : x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}")
        print(f"Settle Before    : {self.settle_before_inspect_sec:.1f} sec")
        print(f"Inspect Window   : {self.inspect_wait_sec:.1f} sec")
        print(f"Samples          : {total_samples}")
        print(
            f"Person Found     : {'Yes' if person_found else 'No'} (hits {person_hits}/{total_samples})"
        )
        print(
            f"Shirt Color      : {best_color} ({best_color_conf}%) (hits {best_color_hits})"
        )
        print(
            f"AprilTag         : {best_tag} (hits {best_tag_hits})"
        )
        print("=" * 72 + "\n")

        if person_found and not self.already_dropped:
            self.drop_cube()
            self.already_dropped = True

            self.hold_robot(
                self.after_drop_wait_sec,
                "ปล่อยลูกบาศก์แล้ว รอหลังปล่อย",
            )

    # =========================================================
    # Main run
    # =========================================================
    def run_all_waypoints(self) -> None:
        if not self.selected_waypoints:
            self.get_logger().error("ยังไม่มี waypoint ให้รัน")
            return

        print("\n" + "=" * 72)
        print("WAYPOINTS ที่จะถูกส่งให้หุ่นวิ่ง")
        print("=" * 72)
        for i, wp in enumerate(self.selected_waypoints, start=1):
            inspect_text = " [INSPECT]" if wp["inspect"] else ""
            reverse_text = " [REVERSE_FROM_PREV]" if wp["reverse_from_prev"] else ""
            print(
                f"{i}. {wp['task']}: "
                f"x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}{inspect_text}{reverse_text}"
            )
        print("=" * 72 + "\n")

        for i, wp in enumerate(self.selected_waypoints):
            if i == 0 and wp["reverse_from_prev"]:
                self.get_logger().warn(
                    f"{wp['task']} เป็น waypoint แรก จึงไม่สามารถ reverse_from_prev ได้ จะใช้ navigation ปกติแทน"
                )
                ok = self.navigate_one_waypoint(wp)
            else:
                if wp["reverse_from_prev"]:
                    ok = self.reverse_to_waypoint(wp)
                else:
                    ok = self.navigate_one_waypoint(wp)

            if not ok:
                self.get_logger().error(
                    f"หยุดการวิ่ง เพราะไป {wp['task']} ไม่สำเร็จ"
                )
                return

            self.hold_robot(  # เพิ่มใหม่
                self.settle_after_every_waypoint_sec,
                "ถึง waypoint แล้วหยุดนิ่งก่อนไปต่อ",
            )

            if wp["inspect"]:
                self.inspect_current_scene(wp)

        self.get_logger().info("สำเร็จ: ไปครบทุก waypoint แล้ว")

    def shutdown_robot(self) -> None:
        self.publish_stop()
        try:
            msg = Int32()
            msg.data = self.servo_close_angle
            self.pub_servo.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RunWaypointsNode()

    try:
        ok = node.load_waypoints_from_yaml()
        if not ok:
            node.destroy_node()
            rclpy.shutdown()
            return

        node.run_all_waypoints()

    except KeyboardInterrupt:
        print("\nหยุดโปรแกรมด้วยคีย์บอร์ด")

    finally:
        try:
            node.shutdown_robot()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
