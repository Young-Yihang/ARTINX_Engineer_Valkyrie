#!/bin/bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# MuJoCo libs (move_group may need them indirectly)
for d in "$HOME"/mujoco-*/lib; do
  [ -d "$d" ] && export LD_LIBRARY_PATH="${d}:${LD_LIBRARY_PATH:-}" && break
done

echo "=== ARV_V1 starting at $(date -Iseconds) ==="
echo "Serial devices: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo 'none detected')"

# 上次断电留下的 mcap 没 metadata.yaml, 启动前批量重建索引让 'ros2 bag info <dir>' 可用
shopt -s nullglob
for d in "$HOME"/arv_v1_logs/20*/rosbag/session; do
  if [ -d "$d" ] && [ ! -f "$d/metadata.yaml" ]; then
    echo "  reindex: $d"
    ros2 bag reindex "$d" --storage mcap >/dev/null 2>&1 || true
  fi
done
shopt -u nullglob

exec ros2 launch arv_v1_moveit hardware_deploy.launch.py \
  serial_port:=auto \
  log_dir:="$HOME/arv_v1_logs" \
  enable_rosbag:=true
