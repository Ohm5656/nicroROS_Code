#!/bin/bash

WS="/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws"
APP="/home/natdanai/inex/start_up_robot"

# เช็ค path ก่อน
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "ไม่พบไฟล์: $WS/install/setup.bash"
    exit 1
fi

if [ ! -d "$APP" ]; then
    echo "ไม่พบโฟลเดอร์: $APP"
    exit 1
fi

# 1) เปิดหน้าดูกล้อง
terminator -u -e "bash -ic '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source \"$WS/install/setup.bash\"
cd \"$APP\"
ros2 run yahboom_esp32_camera sub_img
exec bash
'" &

# 2) เปิดควบคุมหุ่น
terminator -u -e "bash -ic '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source \"$WS/install/setup.bash\"
cd \"$APP\"
python3 ctrl_robot.py
exec bash
'" &

# 3) เปิดตรวจ vision
terminator -u -e "bash -ic '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source \"$WS/install/setup.bash\"
cd \"$APP\"
python3 Cam_Pose_AprilTag.py
exec bash
'" &
