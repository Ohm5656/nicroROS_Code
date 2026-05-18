#!/usr/bin/env python3

import math
import yaml
import os
import time
from collections import Counter, defaultdict
from typing import List, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


WAYPOINTS_YAML = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/waypoint_nav/waypoint_nav/nav_waypoints.yaml"


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    """
    แปลง yaw -> quaternion (z, w)
    """
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return z, w


class RunWaypointsNode(Node):
    def __init__(self):
        super().__init__("run_waypoints")

        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("waypoints_file", WAYPOINTS_YAML)
        self.declare_parameter("inspect_wait_sec", 5.0)
        self.declare_parameter("inspect_spin_dt", 0.1)

        self.action_name = self.get_parameter("action_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.waypoints_file = self.get_parameter("waypoints_file").value
        self.inspect_wait_sec = float(self.get_parameter("inspect_wait_sec").value)
        self.inspect_spin_dt = float(self.get_parameter("inspect_spin_dt").value)

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name,
        )

        self.selected_waypoints: List[Dict[str, Any]] = []

        # ---------------- Vision latest values ----------------
        self.latest_tag = "None"
        self.person_detected = False
        self.shirt_color = "Unknown"
        self.shirt_confidence = 0

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

        self.get_logger().info("=== Run Waypoints Node Started ===")
        self.get_logger().info(f"Using action: {self.action_name}")
        self.get_logger().info(f"Using frame_id: {self.frame_id}")
        self.get_logger().info(f"Using waypoints_file: {self.waypoints_file}")
        self.get_logger().info(
            f"Inspection window: {self.inspect_wait_sec:.1f} sec"
        )

    # =========================================================
    # Vision callbacks
    # =========================================================
    def apriltag_callback(self, msg: String):
        raw = msg.data.strip()
        self.latest_tag = raw if raw else "None"

    def person_callback(self, msg: String):
        self.person_detected = msg.data.strip().lower() == "true"

    def shirt_callback(self, msg: String):
        # format: COLOR|CONF
        raw = msg.data.strip()
        try:
            color, conf = raw.split("|")
            self.shirt_color = color.strip() if color.strip() else "Unknown"
            self.shirt_confidence = int(conf)
        except Exception:
            self.shirt_color = raw if raw else "Unknown"
            self.shirt_confidence = 0

    # =========================================================
    # YAML
    # =========================================================
    def load_waypoints_from_yaml(self) -> bool:
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(f"ไม่พบไฟล์ waypoint: {self.waypoints_file}")
            return False

        try:
            with open(self.waypoints_file, "r") as f:
                data = yaml.safe_load(f)

            waypoints_raw = data.get("waypoints", [])
            if not waypoints_raw:
                self.get_logger().error("ไม่มี waypoint ในไฟล์ YAML")
                return False

            self.selected_waypoints = []
            for i, wp in enumerate(waypoints_raw, start=1):
                x = float(wp["x"])
                y = float(wp["y"])
                yaw = float(wp["yaw"])
                inspect = bool(wp.get("inspect", False))
                task = str(wp.get("task", f"waypoint_{i}"))

                item = {
                    "task": task,
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "inspect": inspect,
                }
                self.selected_waypoints.append(item)

                self.get_logger().info(
                    f"Waypoint {i}: task={task}, x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, inspect={inspect}"
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
    # Navigation
    # =========================================================
    def navigate_one_waypoint(self, wp: Dict[str, Any]) -> bool:
        self.get_logger().info(
            f"กำลังไป {wp['task']} -> x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}, inspect={wp['inspect']}"
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
    # Inspection helpers
    # =========================================================
    def _is_valid_tag(self, tag: str) -> bool:
        if tag is None:
            return False
        t = tag.strip().lower()
        return t not in ("", "none", "waiting for tag...", "waiting")

    def _choose_best_tag(self, tags: List[str]) -> str:
        valid_tags = [t for t in tags if self._is_valid_tag(t)]
        if not valid_tags:
            return "None"

        counter = Counter(valid_tags)
        return counter.most_common(1)[0][0]

    def _choose_best_color(self, color_samples: List[Tuple[str, int]]) -> Tuple[str, int]:
        """
        เอาสีที่โผล่บ่อยสุดในช่วงตรวจ
        ถ้าคะแนนสูสี ใช้ average confidence ตัดสิน
        """
        valid = []
        for color, conf in color_samples:
            c = (color or "").strip()
            if c and c.lower() != "unknown":
                valid.append((c, max(0, int(conf))))

        if not valid:
            return "Unknown", 0

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

            # เรียงตาม:
            # 1) เจอบ่อยสุด
            # 2) avg confidence สูงสุด
            # 3) max confidence สูงสุด
            key = (count, avg_conf, max_conf)

            if best_key is None or key > best_key:
                best_key = key
                best_color = color

        final_count = count_map[best_color]
        final_avg_conf = int(conf_sum_map[best_color] / final_count)

        return best_color, final_avg_conf

    # =========================================================
    # Inspection
    # =========================================================
    def inspect_current_scene(self, wp: Dict[str, Any]):
        self.get_logger().info(
            f"[INSPECT] {wp['task']} -> หยุดตรวจ {self.inspect_wait_sec:.1f} วินาที แล้วสรุปผลจากช่วงเวลานั้น"
        )

        end_time = time.time() + self.inspect_wait_sec

        person_samples: List[bool] = []
        tag_samples: List[str] = []
        color_samples: List[Tuple[str, int]] = []

        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.inspect_spin_dt)

            person_samples.append(self.person_detected)
            tag_samples.append(self.latest_tag)
            color_samples.append((self.shirt_color, self.shirt_confidence))

        # -------- สรุปผล --------
        person_found = any(person_samples)

        best_tag = self._choose_best_tag(tag_samples)
        best_color, best_color_conf = self._choose_best_color(color_samples)

        total_frames = len(person_samples)
        person_true_count = sum(1 for x in person_samples if x)

        print("\n" + "=" * 70)
        print(f"INSPECTION RESULT @ {wp['task']}")
        print("=" * 70)
        print(f"Position        : x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}")
        print(f"Inspect Window  : {self.inspect_wait_sec:.1f} sec")
        print(f"Samples         : {total_frames}")

        print(f"Person Found    : {'Yes' if person_found else 'No'}")
        print(f"Person Hits     : {person_true_count}/{total_frames}")

        print(f"Shirt Color     : {best_color} ({best_color_conf}%)")
        print(f"AprilTag        : {best_tag}")
        print("=" * 70 + "\n")

    # =========================================================
    # Main run
    # =========================================================
    def run_all_waypoints(self):
        if not self.selected_waypoints:
            self.get_logger().error("ยังไม่มี waypoint ให้รัน")
            return

        print("\n" + "=" * 70)
        print("WAYPOINTS ที่จะถูกส่งให้หุ่นวิ่ง")
        print("=" * 70)
        for i, wp in enumerate(self.selected_waypoints, start=1):
            inspect_text = " [INSPECT]" if wp["inspect"] else ""
            print(
                f"{i}. {wp['task']}: x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f}{inspect_text}"
            )
        print("=" * 70 + "\n")

        for i, wp in enumerate(self.selected_waypoints, start=1):
            ok = self.navigate_one_waypoint(wp)
            if not ok:
                self.get_logger().error(f"หยุดการวิ่ง เพราะไป {wp['task']} ไม่สำเร็จ")
                return

            if wp["inspect"]:
                self.inspect_current_scene(wp)

        self.get_logger().info("สำเร็จ: ไปครบทุก waypoint แล้ว")


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
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
