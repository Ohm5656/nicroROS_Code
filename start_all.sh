#!/bin/bash

export ROS_CODE=~/ROS_Source_Code

echo "🚀 Starting all ROS2 nodes..."

# 1) watchdog (ต้องมาก่อนสุด)
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; cd /home/natdanai/inex/start_up_robot; python3 watchdog.py; exec bash"
sleep 1

# 2) micro-ROS agent
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; cd $ROS_CODE; sh start_agent_computer.sh; exec bash"
sleep 1

# 3) Camera Agent : 9999
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; cd $ROS_CODE; echo '===== START CAMERA AGENT : 9999 ====='; sh start_Camera_computer.sh; exec bash"
sleep 1

# 4) bringup
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source ~/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py; exec bash"
sleep 1

# 5) display (RViz)
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source ~/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 launch yahboomcar_nav display_launch.py; exec bash"
sleep 1

# 6) navigation
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source ~/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 launch yahboomcar_nav navigation_dwb_launch.py maps:=/home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/src/yahboomcar_nav/maps/yahboom_map.yaml; exec bash"
sleep 1

# 7) Pose Detector
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source ~/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 run yahboom_esp32_mediapipe 02_PoseDetector; exec bash"

echo "✅ All terminals launched!"
