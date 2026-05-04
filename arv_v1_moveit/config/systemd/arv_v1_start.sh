#!/bin/bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/huan/ros2_ws/install/setup.bash

# MuJoCo libs (move_group may need them indirectly)
for d in /home/huan/mujoco-*/lib; do
  [ -d "$d" ] && export LD_LIBRARY_PATH="${d}:${LD_LIBRARY_PATH:-}" && break
done

echo "=== ARV_V1 starting at $(date -Iseconds) ==="
echo "Serial devices: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo 'none detected')"

exec ros2 launch arv_v1_moveit hardware_deploy.launch.py \
  serial_port:=auto \
  log_dir:=/home/huan/arv_v1_logs \
  enable_rosbag:=true
