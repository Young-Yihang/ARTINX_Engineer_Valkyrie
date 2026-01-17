#!/bin/bash
# 停止所有ARV_V1节点

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=========================================="
echo "  停止所有 MuJoCo 系统节点"
echo "==========================================${NC}"
echo ""

# 查找并杀死相关进程
echo -e "${GREEN}查找运行中的节点...${NC}"

# 节点列表 (覆盖所有模式: SIM/HARDWARE)
nodes=(
    "mujoco_interface_node"
    "hardware_interface_node"
    "torque_controller_node"
    "dynamics_solver_node"
    "move_group"
    "rviz2"
    "robot_state_publisher"
    "static_transform_publisher"
    "ros2_control_node"
    "controller_manager"
)

for node in "${nodes[@]}"; do
    pids=$(pgrep -f "$node")
    if [ -n "$pids" ]; then
        echo -e "${YELLOW}停止: $node (PIDs: $pids)${NC}"
        kill $pids 2>/dev/null
        sleep 0.5
        # 强制杀死（如果还在运行）
        pids=$(pgrep -f "$node")
        if [ -n "$pids" ]; then
            echo -e "${RED}强制停止: $node${NC}"
            kill -9 $pids 2>/dev/null
        fi
    else
        echo -e "未找到: $node"
    fi
done

echo ""
echo -e "${GREEN}=========================================="
echo "  所有节点已停止"
echo "==========================================${NC}"
