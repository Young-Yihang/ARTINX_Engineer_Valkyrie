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
# 暖色日落渐变: 粉(255,100,150) → 橙(255,180,50) → 金(255,220,100)
sunset_rgb() {
    local row=$1 total=$2
    local max=$((total - 1))
    (( max < 1 )) && max=1
    local r=255
    local g=$(( 100 + 120 * row / max ))
    local b=$(( 150 - 100 * row / max + 50 * row / max ))
    # 简化: 粉→橙→金
    if (( row * 2 < total )); then
        local t=$(( row * 2 ))
        r=255; g=$(( 100 + 80 * t / total )); b=$(( 150 - 100 * t / total ))
    else
        local t=$(( (row * 2 - total) ))
        r=255; g=$(( 180 + 40 * t / total )); b=$(( 50 + 50 * t / total ))
    fi
    printf "%d;%d;%d" $r $g $b
}

show_mascot() {
    clear
    printf "\033[?25l"

    local -a mlines=()
    while IFS= read -r line; do
        mlines+=("$line")
    done << 'MASCOT'
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
    local total=${#mlines[@]}

    # 从底部往上逐行浮现 (暖色日落)
    for (( i=total-1; i>=0; i-- )); do
        local rgb
        rgb=$(sunset_rgb $i $total)
        printf "\033[%d;1H\033[2;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
        sleep 0.015
    done
    sleep 0.06

    # 亮起
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(sunset_rgb $i $total)
        printf "\033[%d;1H\033[1;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
    done
    sleep 0.15

    # 暖色脉冲 3 帧 (亮→暗→亮)
    for dim in 2 1 0; do
        local bold=$((1 - dim % 2))
        for (( i=0; i<total; i++ )); do
            local rgb
            rgb=$(sunset_rgb $i $total)
            printf "\033[%d;1H\033[${bold};38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
        done
        sleep 0.12
    done

    # 定格
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(sunset_rgb $i $total)
        printf "\033[%d;1H\033[38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
    done

    echo ""
    echo -e "  \033[38;2;255;180;100m:: ARV_V1 SYSTEM SHUTDOWN ::\033[0m"
    echo "  ──────────────────────────────────────────"
    echo ""
}

# 隐藏/显示光标
cursor_hide() { tput civis; }
cursor_show() { tput cnorm; }

# 步骤计数器
STEP_CURRENT=0
STEP_TOTAL=1

# 进度: 逐行 spinner
update_status() {
    local text="$1"
    STEP_CURRENT=$((STEP_CURRENT + 1))
    local frames=("⠋" "⠙" "⠹" "⠸" "⠼" "⠴" "⠦" "⠧" "⠇" "⠏")
    for i in $(seq 0 3); do
        printf "\r  ${CYAN}${frames[$((i % ${#frames[@]}))]}${NC} $text"
        sleep 0.08
    done
    printf "\r  ${RED}✗${NC} $text\n"
}

# ==================== 主流程 ====================
cursor_hide
show_mascot

# 节点列表（按层级顺序: 应用层→控制层→接口层→基础设施）
nodes=(
    "mission_panel.py"
    "move_to_pose.py"
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
update_status "停止 ros2 launch 进程"
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
    update_status "停止: $node"

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

# 最终报告
echo ""
if [ $stopped -gt 0 ]; then
    echo -e "  ${GREEN}${BOLD}✓ 已停止 $stopped 个节点${NC}"
else
    echo -e "  ${YELLOW}${BOLD}✓ 未发现运行中的节点${NC}"
fi
echo ""
cursor_show
