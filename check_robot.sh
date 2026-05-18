#!/bin/bash

WS="/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws"
APP="/home/natdanai/inex/start_up_robot"
ROS_CODE="/home/natdanai/ROS_Source_Code"

echo "🚀 START CHECK ROBOT"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "❌ ไม่พบไฟล์: $WS/install/setup.bash"
    exit 1
fi

if [ ! -d "$APP" ]; then
    echo "❌ ไม่พบโฟลเดอร์: $APP"
    exit 1
fi

if [ ! -f "$ROS_CODE/start_agent_computer.sh" ]; then
    echo "❌ ไม่พบไฟล์: $ROS_CODE/start_agent_computer.sh"
    exit 1
fi

if [ ! -f "$ROS_CODE/start_Camera_computer.sh" ]; then
    echo "❌ ไม่พบไฟล์: $ROS_CODE/start_Camera_computer.sh"
    exit 1
fi

# 1) Robot Agent : 8090
terminator -u -e "bash -ic '
cd \"$ROS_CODE\"
echo \"===== START ROBOT AGENT : 8090 =====\"
sh start_agent_computer.sh
exec bash
'" &

sleep 2

# 2) Camera Agent : 9999
terminator -u -e "bash -ic '
cd \"$ROS_CODE\"
echo \"===== START CAMERA AGENT : 9999 =====\"
sh start_Camera_computer.sh
exec bash
'" &

sleep 2

# 3) Watchdog
terminator -u -e "bash -ic '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source \"$WS/install/setup.bash\"
cd \"$APP\"
echo \"===== START WATCHDOG =====\"
python3 watchdog.py
exec bash
'" &

sleep 2

# 4) Pose Detector
terminator -u -e "bash -ic '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source \"$WS/install/setup.bash\"
cd \"$APP\"
echo \"===== START POSE DETECTOR =====\"
ros2 run yahboom_esp32_mediapipe 02_PoseDetector
exec bash
'" &

sleep 2

# 5) Keyboard Control
terminator -u -e "bash -ic '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source \"$WS/install/setup.bash\"
cd \"$APP\"
echo \"===== START KEYBOARD CONTROL =====\"
ros2 run yahboomcar_ctrl yahboom_keyboard
exec bash
'" &
