#!/bin/bash
set -euo pipefail

WORKSPACE="$HOME/ros2_ws"
SYSTEMD_SRC="$WORKSPACE/src/arv_v1_moveit/config/systemd"
SYSTEMD_DEST="$HOME/.config/systemd/user"
LOG_DIR="$HOME/arv_v1_logs"

echo "=== ARV_V1 NUC Deployment ==="
echo ""

# 1. Prerequisites
echo "[1/7] Checking prerequisites..."
if ! groups | grep -q dialout; then
  echo "ERROR: User not in dialout group. Run: sudo usermod -aG dialout $USER"
  exit 1
fi
if [ ! -d "$WORKSPACE/install" ]; then
  echo "ERROR: Workspace not built. Run:"
  echo "  cd $WORKSPACE && colcon build --packages-select arv_v1_interfaces arv_v1_model arv_v1_moveit --cmake-args -DCMAKE_BUILD_TYPE=Release"
  exit 1
fi
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
  echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy/"
  exit 1
fi
echo "  OK"

# 2. Log directory
echo "[2/7] Creating log directory..."
mkdir -p "$LOG_DIR"
echo "  $LOG_DIR"

# 3. Enable linger (user services start at boot without login)
echo "[3/7] Enabling user linger..."
loginctl enable-linger "$USER"
echo "  OK"

# 4. Install systemd units
echo "[4/7] Installing systemd user services..."
mkdir -p "$SYSTEMD_DEST"
cp "$SYSTEMD_SRC/arv-v1.service" "$SYSTEMD_DEST/"
cp "$SYSTEMD_SRC/arv-v1-log-cleanup.service" "$SYSTEMD_DEST/"
cp "$SYSTEMD_SRC/arv-v1-log-cleanup.timer" "$SYSTEMD_DEST/"
echo "  Installed to $SYSTEMD_DEST/"

# 5. Permissions
echo "[5/7] Setting script permissions..."
chmod +x "$SYSTEMD_SRC/arv_v1_start.sh"
chmod +x "$SYSTEMD_SRC/cleanup_logs.sh"
echo "  OK"

# 6. Enable services
echo "[6/7] Enabling services..."
systemctl --user daemon-reload
systemctl --user enable arv-v1.service
systemctl --user enable arv-v1-log-cleanup.timer
systemctl --user start arv-v1-log-cleanup.timer
echo "  OK"

# 7. Summary
echo "[7/7] Deployment complete!"
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║  ARV_V1 Quick Reference                         ║"
echo "╠══════════════════════════════════════════════════╣"
echo "║  Start now:    systemctl --user start arv-v1    ║"
echo "║  Stop:         systemctl --user stop arv-v1     ║"
echo "║  Status:       systemctl --user status arv-v1   ║"
echo "║  Logs (live):  journalctl --user -u arv-v1 -f   ║"
echo "║  Disable boot: systemctl --user disable arv-v1  ║"
echo "║  View rosbag:  ls ~/arv_v1_logs/latest/rosbag/  ║"
echo "║  View node logs: journalctl --user -u arv-v1    ║"
echo "╠══════════════════════════════════════════════════╣"
echo "║  Manual nodes (SSH/display):                    ║"
echo "║    ros2 run arv_v1_moveit mission_executor_node ║"
echo "║    ros2 run arv_v1_moveit mujoco_interface_node ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
echo "The service will auto-start on next boot."
echo "To start immediately: systemctl --user start arv-v1"
