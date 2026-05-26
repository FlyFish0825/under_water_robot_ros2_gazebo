#!/bin/bash

set -eu

echo "============================================"
echo "清理 Gazebo / ROS 2 后台进程及残留..."
echo "============================================"

# 1. 按进程名 / 命令行匹配终止
echo "[1] 终止 Ignition Gazebo / 桥接 / ROS 节点..."
pkill -f "ign gazebo"      2>/dev/null || true
pkill -f "gz sim"          2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "thrust_controler_server" 2>/dev/null || true
pkill -f "pubilsh_robot_state"     2>/dev/null || true
pkill -f "ros2 daemon"     2>/dev/null || true

# 2. 清理临时文件
echo "[2] 清理 /tmp/ign-* /tmp/gz-* 临时目录..."
rm -rf /tmp/ign-* /tmp/gz-* 2>/dev/null || true

# 3. 清理共享内存
echo "[3] 清理共享内存段..."
ipcs -m 2>/dev/null | awk '/[0-9]/{print $2}' | while read -r shmid; do
  ipcrm -m "$shmid" 2>/dev/null || true
done

# 4. 停止 ROS 2 守护进程
echo "[4] 停止 ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo "============================================"
echo "清理完成"
echo "============================================"
