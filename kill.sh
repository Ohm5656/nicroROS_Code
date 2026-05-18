#!/bin/bash

echo "CLEANING ALL ROS2 / PORT / TERMINALS..."

# =========================
# 1) Kill ROS2 launch processes
# =========================
echo "Killing ROS2 launch processes..."
pkill -f "ros2 launch" 2>/dev/null
pkill -f "ros2 run" 2>/dev/null
sleep 1

# =========================
# 2) Kill ROS2 node processes (direct binaries)
# =========================
echo "Killing ROS2 node binaries..."
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "component_container_isolated" 2>/dev/null
pkill -f "component_container" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "gazebo" 2>/dev/null
pkill -f "micro_ros_agent" 2>/dev/null
pkill -f "waypoint_nav" 2>/dev/null
pkill -f "run_waypoints" 2>/dev/null
pkill -f "stop_car" 2>/dev/null
pkill -f "PoseDetector" 2>/dev/null
pkill -f "imu_filter" 2>/dev/null
pkill -f "ekf_filter_node" 2>/dev/null
sleep 1

# =========================
# 3) Kill ports 8090 and 9999 (micro-ROS agents)
# =========================
echo "Killing ports 8090 / 9999..."
PID_8090=$(sudo lsof -t -i :8090 2>/dev/null)
[ -n "$PID_8090" ] && sudo kill -9 $PID_8090 && echo "Killed port 8090: $PID_8090"

PID_9999=$(sudo lsof -t -i :9999 2>/dev/null)
[ -n "$PID_9999" ] && sudo kill -9 $PID_9999 && echo "Killed port 9999: $PID_9999"

sudo fuser -k 8888/tcp 2>/dev/null
sudo fuser -k 11311/tcp 2>/dev/null
echo "Ports cleaned"

# =========================
# 4) Force kill remaining ros2 CLI processes
#    (ข้าม claude process)
# =========================
echo "Force killing remaining ROS2 CLI processes..."
for pid in $(pgrep -f "python3.*ros2" 2>/dev/null); do
    CMD=$(ps -p $pid -o args= 2>/dev/null)
    if echo "$CMD" | grep -qi "claude"; then
        echo "Skipping Claude process: $pid"
        continue
    fi
    kill -9 $pid 2>/dev/null
done

# =========================
# 5) Stop Docker
# =========================
echo "Stopping docker containers..."
RUNNING=$(docker ps -q 2>/dev/null)
if [ -n "$RUNNING" ]; then
    docker stop $RUNNING
    echo "Docker stopped"
else
    echo "No docker running"
fi

# =========================
# 6) Close extra terminals (ข้าม Claude terminal)
# =========================
echo "Closing extra terminals..."
CURRENT_TTY=$(tty)

for pid in $(pgrep gnome-terminal 2>/dev/null); do
    CMD=$(ps -p $pid -o args= 2>/dev/null)

    if echo "$CMD" | grep -qi "claude"; then
        echo "Skipping Claude terminal: $pid"
        continue
    fi

    TTY_OF_PID=$(ps -p $pid -o tty= 2>/dev/null)
    if [ "/dev/$TTY_OF_PID" = "$CURRENT_TTY" ]; then
        echo "Skipping current terminal: $pid"
        continue
    fi

    kill -9 $pid 2>/dev/null
done
echo "Extra terminals cleaned"

# =========================
# 7) Clean shared memory (แก้ RTPS_TRANSPORT_SHM error)
# =========================
echo "Cleaning shared memory..."
sudo rm -f /dev/shm/fastrtps_* 2>/dev/null
sudo rm -f /tmp/launch_params_* 2>/dev/null
echo "Shared memory cleaned"

# =========================
# 8) Reset ROS daemon
# =========================
echo "Resetting ROS daemon..."
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
echo "ROS daemon restarted"

echo ""
echo "ALL CLEANED. Ready to run start_all.sh"
