#!/bin/bash

echo "🚀 START CREATE MAP..."

# source ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=18
source /home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash

# 1) watchdog (ต้องมาก่อนสุด)
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; cd /home/natdanai/inex/start_up_robot; python3 watchdog.py; exec bash"

sleep 2

# 2) run micro-ROS agent
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; cd /home/natdanai/ROS_Source_Code; sh start_agent_computer.sh; exec bash"

sleep 2

# 3) bringup robot
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source /home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py; exec bash"

sleep 2

# 4) RViz
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source /home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 launch yahboomcar_nav display_launch.py; exec bash"

sleep 2

# 5) SLAM
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source /home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 launch yahboomcar_nav map_slam_toolbox_launch.py; exec bash"

sleep 2

# 6) keyboard control
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=18; source /home/natdanai/ROS_Source_Code/yahboomcar_ws/function_package_without_WiFi_module/yahboomcar_ws/install/setup.bash; ros2 run yahboomcar_ctrl yahboom_keyboard; exec bash"

echo "✅ ALL NODES STARTED"
