#!/bin/bash
# 停止所有ARV_V1节点

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ==================== 可爱的停止画面 ====================
show_mascot() {
    echo -e "${CYAN}"
    cat << 'MASCOT'
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡀⣄⢄⣠⡄⢠⠎⠀⠀⠈⠳⣄⠀⠀⢀⣀⡐⠲⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠠⣶⢰⡧⠈⠟⠈⠉⠈⢠⠏⠀⠀⠀⠀⠀⠈⠳⡀⠀⠀⠉⠳⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣠⣈⢿⠇⠉⠀⠀⠀⠀⠀⢀⣀⠾⠀⠀⠀⠀⠀⠀⠀⠀⠙⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⡀⣀⠰⣮⠌⠋⣁⡤⠤⠶⠒⠒⠒⠒⠒⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⢦⡀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀
⠀⠀⠀⢀⡄⠙⠿⠃⠀⠀⢸⡃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⣀⣀⡀⠀⠀⠀⠙⢦⠀⠀⠀⠀⠠⡖⠋⠙⠦⡄⠀⠀⠀
⠀⠀⢺⡆⠉⠁⠀⠀⠀⠀⠈⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣶⠿⠛⠉⠉⠉⠉⠉⠓⢦⣄⣄⠀⢳⡀⠀⠀⠀⢧⣤⣄⡞⠁⠀⠀⠀
⠀⠀⠀⠀⠐⢒⣶⠆⠀⠀⠀⢹⣆⠀⠀⠀⠀⠀⠀⠀⠀⢠⡾⠋⠀⠀⠀⠀⠀⠀⠀⢠⣄⠘⢿⣿⡷⣤⢷⡀⠀⠀⢀⡇⠈⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢰⣿⠥⠄⠀⠀⠀⠀⠻⡄⡀⠀⠀⠀⠀⠀⢠⣿⡇⠀⠀⢸⡆⠀⠀⡰⣆⠚⡏⠳⣔⣦⠈⣇⠀⢷⡀⠀⣸⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡄⠀⠀⣿⠂⠀⠀⠀⠀⠀⡾⢸⡇⠀⠀⣼⠿⡄⠀⡧⠟⠀⢿⡟⢿⣻⣢⣽⠀⢸⠓⠒⡯⣄⠀⠀⠀⠰⠆⠀
⠀⠀⠀⠀⠰⣺⣇⠀⠀⠀⠀⠀⢠⡇⠀⠀⠀⠀⠀⢠⡇⠘⣧⠀⣿⣼⣴⢿⡀⠀⠀⡀⠀⠉⠈⠈⣿⢻⠀⡸⠀⢰⠗⠈⢳⡀⠀⠀⠀⠀
⠀⠀⠀⠀⠈⠉⠁⠀⢀⣀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠘⡇⠀⠹⣦⣹⣿⣿⣩⣇⣀⣘⡋⠀⠀⣀⡴⢛⡯⠞⠁⠀⠉⠀⠀⠀⣧⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠉⣿⠤⠀⣾⠀⠀⠀⠀⠀⠀⠀⣇⠀⢀⠈⣿⠽⠚⠉⠀⠀⠈⠉⢳⡿⣯⠞⠉⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠀⢸⡀⢸⡷⠞⠀⠀⠀⠀⠀⠀⠀⠈⡇⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠳⣸⡇⠀⠀⢀⠂⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⢀⡇⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠴⠄⠀⠀⠀⣰⠋⡇⠀⠀⠀⠀⠀⠀⠀⠀⠈⠃⠀⢀⠎⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⡰⠀⠀⠀⠀⣸⠁⠀⠀⠀⠀
⠀⠀⡀⢠⠆⠀⠀⠀⠀⠀⣼⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠎⠀⠀⠀⢀⠀⠀⠀⠃⠀⠀⠀⠀⢠⠃⠀⠀⠀⢠⠏⠀⠀⠀⠀⠀
⠀⢸⠁⢟⠀⠀⠀⠀⠀⠀⢻⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠃⠀⠀⠀⠀⠄⠀⠀⢸⠀⠀⠀⠀⢀⠇⠀⠀⠀⢀⡾⣄⠀⠀⠀⠀⠀
⠀⠀⠀⠈⠀⠀⠀⠀⣀⣀⡼⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡰⠃⠀⠀⠀⠀⠀⠀⠀⠀⡟⠀⠀⠀⠀⠈⠀⠀⠀⠀⠈⠀⠀⠙⡆⠀⠀⠀
⠀⠀⠀⠀⢰⠊⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⣼⣧⣀⠀⠀⠀⣀⣀⣠⣤⡤⠤⠤⠴⠚⠁⠀⠀⠀
⠀⠀⠀⠀⠈⠓⠦⠤⢤⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣰⡭⠼⠛⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  よくねむれますように。~ Good Night
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
MASCOT
    echo -e "${NC}"
}

clear
show_mascot
echo -e "${YELLOW}=========================================="
echo "  停止所有 MuJoCo 系统节点"
echo "==========================================${NC}"
echo ""

# 查找并杀死相关进程
echo -e "${GREEN}查找运行中的节点...${NC}"
echo -e "${YELLOW}停止 ros2 launch 进程（关键步骤）...${NC}"
pids=$(pgrep -f "ros2 launch" | grep -v grep)
if [ -n "$pids" ]; then
    echo -e "${YELLOW}  找到 ros2 launch PID: $pids${NC}"
    echo $pids | xargs -r kill 2>/dev/null
    sleep 1.5
    pids=$(pgrep -f "ros2 launch" | grep -v grep)
    if [ -n "$pids" ]; then
        echo -e "${RED}  强制停止 ros2 launch${NC}"
        echo $pids | xargs -r kill -9 2>/dev/null
        sleep 0.5
    fi
    echo -e "${GREEN}  ✓ ros2 launch 已停止，bash 应自动退出${NC}"
else
    echo -e "未找到 ros2 launch 进程"
fi
echo -e "${YELLOW}清理残留节点...${NC}"

# 节点列表 (覆盖所有模式: SIM/HARDWARE)
nodes=(
    "mujoco_interface_node"
    "hardware_interface_node"
    "torque_controller_node"
    "trajectory_manager_node"
    "mission_executor_node"
    "move_group"
    "rviz2"
    "robot_state_publisher"
    "static_transform_publisher"
    "ros2_control_node"
    "controller_manager"
    "cartesian_controller_node"
)

for node in "${nodes[@]}"; do
    pids=$(pgrep -f "$node")
    if [ -n "$pids" ]; then
        echo -e "${YELLOW}停止: $node (PIDs: $pids)${NC}"
        kill $pids 2>/dev/null
        sleep 0.5
        pids=$(pgrep -f "$node")
        if [ -n "$pids" ]; then
            echo -e "${RED}强制停止: $node${NC}"
            kill -9 $pids 2>/dev/null
        fi
    else
        echo -e "未找到: $node"
    fi
done

echo -e "${GREEN}终端窗口应已随 ros2 launch 进程自动关闭${NC}"

echo ""
echo -e "${GREEN}=========================================="
echo "  所有节点已停止"
echo "==========================================${NC}"
