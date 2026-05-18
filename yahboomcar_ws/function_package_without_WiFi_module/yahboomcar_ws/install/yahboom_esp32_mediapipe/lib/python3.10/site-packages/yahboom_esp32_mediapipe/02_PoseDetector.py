#!/usr/bin/env python3
# encoding: utf-8

import time
from collections import Counter, deque

import apriltag
import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from yahboomcar_msgs.msg import PointArray


class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils

        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon,
        )

        self.lmDrawSpec = self.mpDraw.DrawingSpec(
            color=(0, 0, 255), thickness=-1, circle_radius=5
        )
        self.drawSpec = self.mpDraw.DrawingSpec(
            color=(0, 255, 0), thickness=2, circle_radius=2
        )

        # เก็บประวัติสีเพื่อลดการเด้ง
        self.color_history = deque(maxlen=8)

    def classify_color_from_hsv_pixels(self, hsv_pixels):
        if hsv_pixels is None or len(hsv_pixels) == 0:
            return "Unknown", 0

        h = hsv_pixels[:, 0]
        s = hsv_pixels[:, 1]
        v = hsv_pixels[:, 2]

        total = len(hsv_pixels)
        if total == 0:
            return "Unknown", 0

        counts = {
            "Black": 0,
            "White": 0,
            "Gray": 0,
            "Red": 0,
            "Orange": 0,
            "Yellow": 0,
            "Green": 0,
            "Cyan": 0,
            "Blue": 0,
            "Purple": 0,
            "Pink": 0,
        }

        for hi, si, vi in zip(h, s, v):
            if vi < 45:
                counts["Black"] += 1
                continue

            if si < 30 and vi > 185:
                counts["White"] += 1
                continue

            if si < 45:
                counts["Gray"] += 1
                continue

            if hi < 8 or hi >= 172:
                counts["Red"] += 1
            elif hi < 18:
                counts["Orange"] += 1
            elif hi < 33:
                counts["Yellow"] += 1
            elif hi < 85:
                counts["Green"] += 1
            elif hi < 100:
                counts["Cyan"] += 1
            elif hi < 130:
                counts["Blue"] += 1
            elif hi < 155:
                counts["Purple"] += 1
            else:
                counts["Pink"] += 1

        best_color = max(counts, key=counts.get)
        best_count = counts[best_color]
        confidence = int((best_count / total) * 100)
        return best_color, confidence

    def get_shirt_roi(self, frame, landmarks):
        h, w, _ = frame.shape

        try:
            lm = landmarks.landmark

            ls = lm[self.mpPose.PoseLandmark.LEFT_SHOULDER.value]
            rs = lm[self.mpPose.PoseLandmark.RIGHT_SHOULDER.value]
            lh = lm[self.mpPose.PoseLandmark.LEFT_HIP.value]
            rh = lm[self.mpPose.PoseLandmark.RIGHT_HIP.value]

            ls_x, ls_y = int(ls.x * w), int(ls.y * h)
            rs_x, rs_y = int(rs.x * w), int(rs.y * h)
            lh_x, lh_y = int(lh.x * w), int(lh.y * h)
            rh_x, rh_y = int(rh.x * w), int(rh.y * h)

            shoulder_width = abs(rs_x - ls_x)
            if shoulder_width < 30:
                return None

            mid_shoulder_y = int((ls_y + rs_y) / 2)
            mid_hip_y = int((lh_y + rh_y) / 2)
            torso_height = max(50, mid_hip_y - mid_shoulder_y)

            center_x = int((ls_x + rs_x) / 2)

            roi_half_width = int(shoulder_width * 0.22)
            x1 = center_x - roi_half_width
            x2 = center_x + roi_half_width

            y1 = mid_shoulder_y + int(torso_height * 0.18)
            y2 = mid_shoulder_y + int(torso_height * 0.48)

            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w - 1, x2)
            y2 = min(h - 1, y2)

            if x2 <= x1 or y2 <= y1:
                return None

            roi = frame[y1:y2, x1:x2]
            if roi is None or roi.size == 0:
                return None

            hh, ww = roi.shape[:2]
            margin_x = int(ww * 0.10)
            margin_y = int(hh * 0.10)

            x1i = margin_x
            x2i = ww - margin_x
            y1i = margin_y
            y2i = hh - margin_y

            if x2i <= x1i or y2i <= y1i:
                return roi

            inner_roi = roi[y1i:y2i, x1i:x2i]
            if inner_roi is None or inner_roi.size == 0:
                return roi

            return inner_roi

        except Exception:
            return None

    def remove_skin_pixels(self, hsv_roi):
        if hsv_roi is None or hsv_roi.size == 0:
            return None

        lower_skin = np.array([0, 20, 50], dtype=np.uint8)
        upper_skin = np.array([25, 180, 255], dtype=np.uint8)

        skin_mask = cv.inRange(hsv_roi, lower_skin, upper_skin)
        non_skin_mask = cv.bitwise_not(skin_mask)

        filtered = hsv_roi[non_skin_mask > 0]
        if filtered is None or len(filtered) == 0:
            return hsv_roi.reshape(-1, 3)

        return filtered.reshape(-1, 3)

    def analyze_shirt_color(self, frame, landmarks):
        roi = self.get_shirt_roi(frame, landmarks)
        if roi is None:
            return "Unknown", 0

        hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

        pixels = hsv_roi.reshape(-1, 3)
        if len(pixels) == 0:
            return "Unknown", 0

        pixels = self.remove_skin_pixels(hsv_roi)
        if pixels is None or len(pixels) == 0:
            return "Unknown", 0

        valid = pixels[(pixels[:, 2] > 35)]
        if len(valid) < 80:
            valid = pixels

        if len(valid) == 0:
            return "Unknown", 0

        color_name, confidence = self.classify_color_from_hsv_pixels(valid)
        return color_name, confidence

    def smooth_color_result(self, raw_color, raw_confidence):
        if raw_color != "Unknown":
            self.color_history.append((raw_color, raw_confidence))

        if len(self.color_history) == 0:
            return "Unknown", 0

        color_votes = [c for c, _ in self.color_history]
        color_counter = Counter(color_votes)
        final_color, vote_count = color_counter.most_common(1)[0]

        confs = [conf for c, conf in self.color_history if c == final_color]
        final_conf = int(sum(confs) / len(confs)) if len(confs) > 0 else 0

        history_len = len(self.color_history)
        vote_ratio = vote_count / history_len if history_len > 0 else 0
        final_conf = int(0.7 * final_conf + 0.3 * (vote_ratio * 100))
        final_conf = max(0, min(99, final_conf))

        return final_color, final_conf

    def process_pose(self, frame, draw_on_frame=True, draw_on_black=False):
        point_array = PointArray()
        original_frame = frame.copy()

        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.pose.process(img_rgb)

        black_bg = None
        if draw_on_black:
            black_bg = np.zeros_like(frame)

        detected = False
        shirt_color = "Unknown"
        shirt_conf = 0

        if results.pose_landmarks:
            detected = True

            raw_color, raw_conf = self.analyze_shirt_color(
                original_frame, results.pose_landmarks
            )
            shirt_color, shirt_conf = self.smooth_color_result(raw_color, raw_conf)

            if draw_on_frame:
                self.mpDraw.draw_landmarks(
                    frame,
                    results.pose_landmarks,
                    self.mpPose.POSE_CONNECTIONS,
                    self.lmDrawSpec,
                    self.drawSpec,
                )

            if draw_on_black and black_bg is not None:
                self.mpDraw.draw_landmarks(
                    black_bg,
                    results.pose_landmarks,
                    self.mpPose.POSE_CONNECTIONS,
                    self.lmDrawSpec,
                    self.drawSpec,
                )

            for lm in results.pose_landmarks.landmark:
                point = Point()
                point.x = lm.x
                point.y = lm.y
                point.z = lm.z
                point_array.points.append(point)

        return frame, point_array, detected, black_bg, shirt_color, shirt_conf


class MultiVisionNode(Node):
    def __init__(self):
        super().__init__("multi_vision_node")

        self.bridge = CvBridge()

        self.sub_img = self.create_subscription(
            CompressedImage,
            "/espRos/esp32camera",
            self.handle_topic,
            1,
        )

        self.pub_point = self.create_publisher(PointArray, "/mediapipe/points", 1000)
        self.pub_at_id = self.create_publisher(String, "/vision/latest_at_id", 10)
        self.pub_person = self.create_publisher(String, "/vision/person_detected", 10)
        self.pub_shirt = self.create_publisher(String, "/vision/shirt_color", 10)

        self.pose_detector = PoseDetector(
            mode=False,
            smooth=True,
            detectionCon=0.65,
            trackCon=0.55,
        )

        options = apriltag.DetectorOptions(families="tag36h11")
        self.at_detector = apriltag.Detector(options)

        self.latest_at_id = "None"
        self.latest_shirt_color = "Unknown"
        self.latest_shirt_confidence = 0
        self.pose_status = "Not Found"

        self.prev_time = time.time()
        self.fps = 0.0
        self.show_pose_black_panel = False

        # ===== Hold/Latch settings =====
        # เจอแล้วให้ค้างไว้ชั่วคราว แม้เฟรมถัดไปจะหลุด
        self.person_hold_sec = 1.5
        self.shirt_hold_sec = 1.8
        self.tag_hold_sec = 1.8

        # shirt color จะอัปเดตก็ต่อเมื่อ confidence ถึงเกณฑ์
        self.min_valid_shirt_conf = 45

        self.last_person_seen_time = 0.0
        self.last_shirt_seen_time = 0.0
        self.last_tag_seen_time = 0.0

        self.held_person_detected = False
        self.held_shirt_color = "Unknown"
        self.held_shirt_confidence = 0
        self.held_tag_id = "None"

        self.get_logger().info("MultiVisionNode started")

    def draw_apriltag(self, frame, gray):
        tags = self.at_detector.detect(gray)
        found_tag = False
        current_tag = None
        now = time.time()

        for tag in tags:
            found_tag = True

            corners = tag.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv.line(frame, pt1, pt2, (255, 0, 255), 2)

            center = tuple(tag.center.astype(int))
            cv.circle(frame, center, 5, (0, 255, 255), -1)
            cv.putText(
                frame,
                f"ID:{tag.tag_id}",
                (corners[0][0], corners[0][1] - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2,
            )

            current_tag = f"ID:{tag.tag_id}"
            break

        # behavior แบบ AprilTag ที่คุณต้องการ:
        # เจอแล้วค้างไว้พักหนึ่ง ถ้าไม่เจอต่อเนื่องนานเกิน hold ค่อย reset
        if found_tag and current_tag is not None:
            self.held_tag_id = current_tag
            self.last_tag_seen_time = now
        else:
            if (now - self.last_tag_seen_time) > self.tag_hold_sec:
                self.held_tag_id = "None"

        self.latest_at_id = self.held_tag_id

        msg = String()
        msg.data = self.latest_at_id
        self.pub_at_id.publish(msg)

        return frame, found_tag

    def build_dashboard(self, pose_detected, fps):
        right_panel = np.zeros((480, 400, 3), np.uint8)

        cv.putText(
            right_panel, "VISION DASHBOARD", (20, 45),
            cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2
        )
        cv.line(right_panel, (20, 65), (360, 65), (255, 255, 255), 1)

        cv.putText(
            right_panel, "Latest AprilTag:", (20, 105),
            cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
        )
        cv.putText(
            right_panel, self.latest_at_id, (20, 140),
            cv.FONT_HERSHEY_SIMPLEX, 0.85, (255, 0, 255), 2
        )

        self.pose_status = "Detected" if pose_detected else "Not Found"
        pose_color = (0, 255, 0) if pose_detected else (0, 0, 255)

        cv.putText(
            right_panel, "Human Pose:", (20, 195),
            cv.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1
        )
        cv.putText(
            right_panel, self.pose_status, (20, 230),
            cv.FONT_HERSHEY_SIMPLEX, 0.85, pose_color, 2
        )

        color_text = f"Color: {self.latest_shirt_color} ({self.latest_shirt_confidence}%)"
        cv.putText(
            right_panel, "Shirt Analysis:", (20, 285),
            cv.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1
        )
        cv.putText(
            right_panel, color_text, (20, 320),
            cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2
        )

        cv.putText(
            right_panel, "FPS:", (20, 375),
            cv.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1
        )
        cv.putText(
            right_panel, f"{int(fps)}", (20, 410),
            cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2
        )

        cv.putText(
            right_panel, "Topic:", (20, 445),
            cv.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1
        )
        cv.putText(
            right_panel, "/espRos/esp32camera", (20, 470),
            cv.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1
        )

        return right_panel

    def handle_topic(self, msg):
        start = time.time()
        now = time.time()

        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame = cv.resize(frame, (640, 480))
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        frame, _ = self.draw_apriltag(frame, gray)

        frame, point_msg, pose_detected_raw, black_pose, shirt_color_raw, shirt_conf_raw = self.pose_detector.process_pose(
            frame,
            draw_on_frame=True,
            draw_on_black=self.show_pose_black_panel,
        )
        self.pub_point.publish(point_msg)

        # ===== Person hold =====
        # เจอแล้วให้ค้าง True ไว้พักหนึ่ง
        if pose_detected_raw:
            self.held_person_detected = True
            self.last_person_seen_time = now
        else:
            if (now - self.last_person_seen_time) > self.person_hold_sec:
                self.held_person_detected = False

        # ===== Shirt hold =====
        valid_color = (
            shirt_color_raw is not None
            and shirt_color_raw != "Unknown"
            and shirt_conf_raw >= self.min_valid_shirt_conf
        )

        # เจอสีที่น่าเชื่อถือ -> update และค้างไว้
        if valid_color:
            self.held_shirt_color = shirt_color_raw
            self.held_shirt_confidence = shirt_conf_raw
            self.last_shirt_seen_time = now
        else:
            # ไม่เจอสีในเฟรมนี้ ก็อย่าเพิ่ง reset ทันที
            if (now - self.last_shirt_seen_time) > self.shirt_hold_sec:
                self.held_shirt_color = "Unknown"
                self.held_shirt_confidence = 0

        self.latest_shirt_color = self.held_shirt_color
        self.latest_shirt_confidence = self.held_shirt_confidence

        pose_detected_for_ui = self.held_person_detected

        person_msg = String()
        person_msg.data = "True" if self.held_person_detected else "False"
        self.pub_person.publish(person_msg)

        shirt_msg = String()
        shirt_msg.data = f"{self.latest_shirt_color}|{self.latest_shirt_confidence}"
        self.pub_shirt.publish(shirt_msg)

        dt = now - self.prev_time
        if dt > 0:
            inst_fps = 1.0 / dt
            self.fps = 0.8 * self.fps + 0.2 * inst_fps if self.fps > 0 else inst_fps
        self.prev_time = now

        cv.putText(
            frame,
            f"FPS: {int(self.fps)}",
            (20, 35),
            cv.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
        )

        right_panel = self.build_dashboard(pose_detected_for_ui, self.fps)

        if self.show_pose_black_panel and black_pose is not None:
            black_pose = cv.resize(black_pose, (400, 300))
            right_panel[160:460, 0:400] = black_pose

        combined = np.hstack((frame, right_panel))
        cv.imshow("Yahboom Multi Vision", combined)
        cv.waitKey(1)

        _ = time.time() - start

    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    print("Initializing Multi-Vision System...")
    rclpy.init(args=args)

    node = MultiVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
