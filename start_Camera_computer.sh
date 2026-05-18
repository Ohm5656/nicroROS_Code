#!/bin/bash

# 1. กำหนดชื่อ Container ให้เรียกง่าย
CONTAINER_NAME="uros_agent_9999"

# 2. หยุดและลบ Container เก่า (ถ้ามี) เพื่อล้างทาง
echo "Cleaning up old container..."
docker stop $CONTAINER_NAME 2>/dev/null
docker rm $CONTAINER_NAME 2>/dev/null

# 3. รัน Micro-ROS Agent ใหม่พร้อมตั้งชื่อ (--name)
echo "Starting Micro-ROS Agent..."
docker run -it --rm \
  --name $CONTAINER_NAME \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  --privileged \
  --net=host \
  microros/micro-ros-agent:humble udp4 --port 9999 -v4
