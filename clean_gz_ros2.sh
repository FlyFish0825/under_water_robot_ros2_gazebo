#!/bin/bash

# clean_gz_ros2.sh - 清理 Gazebo (Classic/Ignition/Gazebo) 和 ROS 2 后台进程及残留文件
# 用法: ./clean_gz_ros2.sh

set -e  # 遇到错误退出（但部分命令即使失败也无妨，所以下面会用 || true）

echo "============================================"
echo "开始清理 Gazebo 和 ROS2 后台进程..."
echo "============================================"

# 1. 强制终止已知的 Gazebo / Ignition 进程
echo "[1/6] 终止 gzserver, gzclient, gz-sim, ign 等进程..."
pkill -9 gzserver gzclient gz-sim ign gazebo 2>/dev/null || true
# 备用：killall（有些系统可能没有 pkill）
killall -9 gzserver gzclient gz-sim ign gazebo 2>/dev/null || true

# 2. 清理 Gazebo Classic 锁文件 (通常位于 /tmp 下)
echo "[2/6] 清理 /tmp/gazebo-* 锁文件..."
rm -f /tmp/gazebo-* 2>/dev/null || true

# 3. 清理 Ignition / Gazebo 临时文件夹
echo "[3/6] 清理 /tmp/ign-* 和 /tmp/gz-* 临时目录..."
rm -rf /tmp/ign-* /tmp/gz-* 2>/dev/null || true

# 4. 清理共享内存 (避免 'Address already in use' 错误)
echo "[4/6] 清理共享内存段 (ipcrm)..."
# 列出当前用户的共享内存，然后删除包含 'gazebo' 或 'ign' 或 'gz' 的段
ipcs -m | grep -E "$USER|gazebo|ign|gz" | awk '{print $2}' | while read shmid; do
    ipcrm -m $shmid 2>/dev/null || true
done

# 5. 停止 ROS 2 守护进程（清除节点缓存）
echo "[5/6] 停止 ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

# 6. 可选：清理 ROS 2 日志中的过时记录（非必须，但可减少杂乱）
echo "[6/6] 清理 ROS 2 日志（可选）..."
if [ -d ~/.ros/log ]; then
    find ~/.ros/log -name "*gz*" -type f -delete 2>/dev/null || true
fi

echo "============================================"
echo "清理完成！"
echo "现在运行 'ps aux | grep -E "gz|ign|gazebo"' 应无相关进程（除了 grep 本身）。"
echo "若需重新启动 Gazebo，请直接运行相应命令。"
echo "============================================"