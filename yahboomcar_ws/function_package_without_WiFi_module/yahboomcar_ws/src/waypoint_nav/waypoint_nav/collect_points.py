#!/usr/bin/env python3

import cv2
import yaml
import math
import os

# =========================================================
# FIXED PATHS
# =========================================================
MAP_YAML = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/yahboomcar_nav/maps/yahboom_map.yaml"
MAP_IMAGE = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/yahboomcar_nav/maps/yahboom_map.pgm"
OUTPUT_YAML = "/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/waypoint_nav/waypoint_nav/nav_waypoints.yaml"

SCALE = 2  # ขยายหน้าต่างให้คลิกง่ายขึ้น

# =========================================================
# GLOBAL STATE
# =========================================================
waypoints = []
start_point = None
inspect_mode = False
display = None
original = None
resolution = None
origin_x = None
origin_y = None


# =========================================================
# LOAD MAP
# =========================================================
def load_map():
    global original, display, resolution, origin_x, origin_y

    if not os.path.exists(MAP_YAML):
        raise FileNotFoundError(f"ไม่พบไฟล์ MAP_YAML: {MAP_YAML}")

    if not os.path.exists(MAP_IMAGE):
        raise FileNotFoundError(f"ไม่พบไฟล์ MAP_IMAGE: {MAP_IMAGE}")

    with open(MAP_YAML, "r") as f:
        map_data = yaml.safe_load(f)

    resolution = map_data["resolution"]
    origin_x, origin_y, _ = map_data["origin"]

    img = cv2.imread(MAP_IMAGE)
    if img is None:
        raise RuntimeError(f"โหลดไฟล์แมพไม่ได้: {MAP_IMAGE}")

    original = img.copy()
    display = cv2.resize(
        original, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST
    )


# =========================================================
# HELPER FUNCTIONS
# =========================================================
def pixel_to_world(x, y):
    """
    แปลง pixel บนรูป -> world coordinate
    """
    x = x / SCALE
    y = y / SCALE

    h = original.shape[0]
    y = h - y

    world_x = x * resolution + origin_x
    world_y = y * resolution + origin_y
    return world_x, world_y


def world_to_pixel(wx, wy):
    """
    แปลง world -> pixel บน display
    """
    px = int((wx - origin_x) / resolution)
    py = int((wy - origin_y) / resolution)
    py = original.shape[0] - py

    px = int(px * SCALE)
    py = int(py * SCALE)
    return px, py


def compute_yaw(x1, y1, x2, y2):
    """
    คำนวณ yaw จากจุดเริ่มลาก -> จุดปลายลูกศร
    """
    return math.atan2(y2 - y1, x2 - x1)


def save_yaml():
    data = {"waypoints": waypoints}
    with open(OUTPUT_YAML, "w") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)
    print(f"[SAVED] {OUTPUT_YAML}")


def print_mode_status():
    if inspect_mode:
        print("[MODE] INSPECT = ON  -> จุดถัดไปจะเป็นจุดเปิดกล้อง")
    else:
        print("[MODE] INSPECT = OFF -> จุดถัดไปจะเป็นจุดปกติ")


def redraw():
    global display, original, waypoints

    display = cv2.resize(
        original, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST
    )

    for i, wp in enumerate(waypoints, start=1):
        px, py = world_to_pixel(wp["x"], wp["y"])

        is_inspect = bool(wp.get("inspect", False))
        point_color = (0, 255, 255) if is_inspect else (0, 255, 0)

        # วาดจุด
        cv2.circle(display, (px, py), 6, point_color, -1)

        # ถ้าเป็น inspect point วาดวงเพิ่ม
        if is_inspect:
            cv2.circle(display, (px, py), 12, (0, 255, 255), 2)

        # เขียนเลขลำดับ
        label = f"{i}*" if is_inspect else str(i)
        cv2.putText(
            display,
            label,
            (px + 8, py - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
            cv2.LINE_AA,
        )

        # วาดลูกศรตาม yaw
        yaw = wp["yaw"]
        arrow_len = 25
        end_x = int(px + arrow_len * math.cos(yaw))
        end_y = int(py - arrow_len * math.sin(yaw))
        cv2.arrowedLine(display, (px, py), (end_x, end_y), (0, 0, 255), 2)


# =========================================================
# MOUSE CALLBACK
# =========================================================
def mouse_callback(event, x, y, flags, param):
    global start_point, display, inspect_mode, waypoints

    if event == cv2.EVENT_LBUTTONDOWN:
        start_point = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and start_point is not None:
        temp = display.copy()
        cv2.arrowedLine(temp, start_point, (x, y), (0, 0, 255), 2)
        cv2.imshow("map", temp)

    elif event == cv2.EVENT_LBUTTONUP and start_point is not None:
        end_point = (x, y)

        wx1, wy1 = pixel_to_world(*start_point)
        wx2, wy2 = pixel_to_world(*end_point)
        yaw = compute_yaw(wx1, wy1, wx2, wy2)

        waypoint = {
            "task": f"waypoint_{len(waypoints) + 1}",
            "x": round(wx1, 3),
            "y": round(wy1, 3),
            "yaw": round(yaw, 3),
            "inspect": inspect_mode,
        }

        waypoints.append(waypoint)

        print("-" * 60)
        print(f"[ADD] {waypoint['task']}")
        print(f"      x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}")
        print(f"      open_camera={waypoint['inspect']}")
        if waypoint["inspect"]:
            print("      -> จุดนี้เป็นจุดเปิดกล้อง / ตรวจสอบ")
        else:
            print("      -> จุดนี้เป็นจุดปกติ")
        print("-" * 60)

        save_yaml()
        start_point = None
        redraw()
        cv2.imshow("map", display)


# =========================================================
# MAIN
# =========================================================
def main():
    global inspect_mode, waypoints

    load_map()
    redraw()

    cv2.namedWindow("map", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("map", 1400, 900)
    cv2.imshow("map", display)
    cv2.setMouseCallback("map", mouse_callback)

    print(f"""
==============================
Collect Points From Map
==============================
MAP YAML : {MAP_YAML}
MAP PGM  : {MAP_IMAGE}
SAVE TO  : {OUTPUT_YAML}

วิธีใช้:
- คลิกค้าง + ลาก = สร้าง waypoint + ทิศทาง
- กด i = สลับโหมด inspect point
- กด z = undo จุดล่าสุด
- กด c = clear ทั้งหมด
- กด q = ออก

หมายเหตุ:
- จุดปกติ = สีเขียว
- จุด inspect = สีเหลือง + วงเหลือง
- เลขที่มี * = จุด inspect / เปิดกล้อง
==============================
""")

    print_mode_status()

    while True:
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("i"):
            inspect_mode = not inspect_mode
            print_mode_status()
            redraw()
            cv2.imshow("map", display)

        elif key == ord("z"):
            if waypoints:
                removed = waypoints.pop()
                print(f"[UNDO] ลบจุดล่าสุด: {removed}")
                save_yaml()
                redraw()
                cv2.imshow("map", display)

        elif key == ord("c"):
            waypoints.clear()
            print("[CLEAR ALL] ลบ waypoint ทั้งหมดแล้ว")
            save_yaml()
            redraw()
            cv2.imshow("map", display)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
