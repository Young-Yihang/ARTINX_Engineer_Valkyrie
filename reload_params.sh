#!/bin/bash
# 热重载控制器参数脚本
# 使用方法: ./reload_params.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SIM_MODE=$(ros2 param get /mujoco_interface sim_mode 2>/dev/null | grep -oP '(?<=value: ).*' || echo "true")
if [ "$SIM_MODE" = "True" ] || [ "$SIM_MODE" = "true" ]; then
    CONFIG_FILE="$SCRIPT_DIR/arv_v1_moveit/config/controller_params_sim.yaml"
else
    CONFIG_FILE="$SCRIPT_DIR/arv_v1_moveit/config/controller_params_hw.yaml"
fi

echo "=== 热重载控制器参数 ==="
echo "模式: $([ "$SIM_MODE" = "true" ] || [ "$SIM_MODE" = "True" ] && echo '仿真' || echo '真机')"
echo "配置文件: $CONFIG_FILE"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

# 检查节点是否运行
if ! ros2 node list 2>/dev/null | grep -q "torque_controller_action_server"; then
    echo "[ERROR] 节点未运行: /torque_controller_action_server"
    echo "请先启动系统: ./start_mujoco_system.sh"
    exit 1
fi

# 加载参数
echo "正在加载参数..."
ros2 param load /torque_controller_action_server "$CONFIG_FILE"

if [ $? -eq 0 ]; then
    echo "[OK] 参数热重载成功!"
else
    echo "[ERROR] 参数加载失败"
    exit 1
fi
