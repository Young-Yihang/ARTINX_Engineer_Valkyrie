#!/bin/bash
# ARV_V1 停止脚本 (Game UI版)

# 捕捉 Ctrl+C 恢复光标
trap 'tput cnorm; exit' INT TERM

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ==================== 猫猫画面 ====================
show_mascot() {
    clear
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
MASCOT
    echo -e "${NC}"
    echo -e "  ${CYAN}:: ARV_V1 SYSTEM SHUTDOWN ::${NC}"
    echo "  ──────────────────────────────────────────"
    echo ""
    echo ""
}

# 隐藏/显示光标
cursor_hide() { tput civis; }
cursor_show() { tput cnorm; }

# 进度更新（固定位置刷新）
update_status() {
    local text="$1"
    local percent="$2"
    local bar_len=40
    local filled=$((percent * bar_len / 100))
    local empty=$((bar_len - filled))
    local bar_str=$(printf "%${filled}s" | tr ' ' '█')
    local empty_str=$(printf "%${empty}s" | tr ' ' '░')
    echo -e "\033[2A\033[K  ${BOLD}STATUS:${NC} $text"
    echo -e "\033[K  ${RED}[${bar_str}${empty_str}]${NC} ${percent}%"
}

# ==================== 主流程 ====================
cursor_hide
show_mascot

# 节点列表（按层级顺序: 应用层→控制层→接口层→基础设施）
nodes=(
    "mission_executor_node"
    "cartesian_controller_node"
    "trajectory_manager_node"
    "torque_controller_node"
    "mujoco_interface_node"
    "hardware_interface_node"
    "move_group"
    "rviz2"
    "robot_state_publisher"
    "static_transform_publisher"
    "ros2_control_node"
    "controller_manager"
)
total=${#nodes[@]}
stopped=0

# 先处理 ros2 launch
update_status "停止 ros2 launch 进程..." 5
pids=$(pgrep -f "ros2 launch" | grep -v grep)
if [ -n "$pids" ]; then
    echo $pids | xargs -r kill 2>/dev/null
    sleep 1.5
    pids=$(pgrep -f "ros2 launch" | grep -v grep)
    if [ -n "$pids" ]; then
        echo $pids | xargs -r kill -9 2>/dev/null
        sleep 0.5
    fi
fi

# 逐个停止节点
for i in "${!nodes[@]}"; do
    node="${nodes[$i]}"
    percent=$(( (i + 1) * 90 / total + 10 ))
    update_status "停止: $node" "$percent"

    pids=$(pgrep -f "$node")
    if [ -n "$pids" ]; then
        kill $pids 2>/dev/null
        sleep 0.3
        pids=$(pgrep -f "$node")
        if [ -n "$pids" ]; then
            kill -9 $pids 2>/dev/null
        fi
        ((stopped++))
    fi
done

update_status "清理完成！" 100
sleep 0.5

# 最终报告
echo ""
echo ""
if [ $stopped -gt 0 ]; then
    echo -e "  ${GREEN}${BOLD}[完成]${NC} 已停止 $stopped 个节点"
else
    echo -e "  ${YELLOW}${BOLD}[完成]${NC} 未发现运行中的节点"
fi
echo ""
cursor_show
