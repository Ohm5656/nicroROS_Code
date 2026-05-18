#!/bin/bash

echo "💾 SAVE MAP..."

# source ROS 2
source /opt/ros/humble/setup.bash
source ~/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash

ros2 launch yahboomcar_nav save_map_launch.py

echo "✅ MAP SAVED"
