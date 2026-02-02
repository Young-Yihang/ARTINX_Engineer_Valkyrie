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

# 关闭由启动脚本创建的终端窗口
echo -e "${GREEN}关闭相关终端窗口...${NC}"
terminal_titles=(
    "MoveIt+RViz"
    "TorqueController"
    "MuJoCo(仿真)"
    "MuJoCo(孪生)"
    "SerialInterface"
    "TrajectoryManager"
    "MissionExecutor"
)

for title in "${terminal_titles[@]}"; do
    # 使用 wmctrl 关闭指定标题的窗口
    if command -v wmctrl &> /dev/null; then
        wmctrl -c "$title" 2>/dev/null && echo -e "关闭窗口: $title"
    else
        # 备用方案: 使用 xdotool
        if command -v xdotool &> /dev/null; then
            wid=$(xdotool search --name "$title" 2>/dev/null | head -1)
            if [ -n "$wid" ]; then
                xdotool windowclose "$wid" 2>/dev/null && echo -e "关闭窗口: $title"
            fi
        fi
    fi
done

# 如果没有 wmctrl 或 xdotool，提示用户安装
if ! command -v wmctrl &> /dev/null && ! command -v xdotool &> /dev/null; then
    echo -e "${YELLOW}提示: 安装 wmctrl 或 xdotool 可自动关闭终端窗口${NC}"
    echo -e "${YELLOW}  sudo apt install wmctrl${NC}"
fi

echo ""
echo -e "${GREEN}=========================================="
echo "  所有节点已停止"
echo "==========================================${NC}"
