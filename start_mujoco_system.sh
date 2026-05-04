#!/bin/bash
# ARV_V1启动脚本: 仿真模式 / 串口真机+孪生模式

set -e

# 捕捉 Ctrl+C 恢复光标
trap 'tput cnorm; exit' INT TERM

# ========== 颜色定义 ==========
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ========== ARTINX Splash ==========
get_artinx_art() {
    echo '⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣦⣤⡄⠀⠀⠀⢀⣀⣤⣤⣀⠀⠀⠀⢀⣀⣀⣠⡄⠀⣀⣤⣄⠀⠀⢀⣀⠀⠀⠀⢀⣀⡀⢀⣀⡀⠤⠤⠒⢂⣀⠀⠀⠀⠀⠀⠀⠀⠀'
    echo '⠀⠀⠀⠀⠀⠠⠐⣢⣽⣿⡿⠋⠙⣿⡄⠀⣾⡿⢛⣫⣿⠿⠁⠘⠛⠛⣿⠟⠋⠉⢀⣿⠏⠁⢀⣴⣿⣿⡆⣠⣾⡿⠋⠙⣿⣶⣶⣤⣶⣾⠿⠟⠁⠀⠀⠀⠀⠀⠀⠀'
    echo '⠀⠀⠀⠀⠀⣠⣾⣿⣿⣧⣶⣶⣻⣿⠃⣸⣿⣿⣿⡛⠁⡆⠀⠀⠀⣼⡿⠀⣠⣤⣾⣏⠀⣰⣿⠟⠁⣿⣷⡿⠛⢁⣤⣾⣷⠿⠛⠛⠛⠿⠿⠶⣶⣦⣤⣀⡀⠀⠀⠀'
    echo '⠀⠀⢀⣴⣿⣿⣋⣭⠿⠛⠋⠙⠻⡟⠀⠿⠃⠈⠙⠻⢿⣿⠄⠀⠀⠛⠃⠘⠛⠛⣋⣁⣀⣘⣁⣠⣤⣬⣯⣤⣤⣿⣿⣿⣶⡶⠶⠶⠶⠶⠶⢶⣶⣾⣿⣿⣿⣷⡦⠀'
    echo '⠀⢴⣿⡿⠟⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠒⠒⠒⠛⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠈⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀'
}

# 紫→绿渐变
splash_row_color() {
    local row=$1 total=$2
    local max=$((total - 1))
    (( max < 1 )) && max=1
    local r=$(( 180 - 180 * row / max ))
    local g=$(( 255 * row / max ))
    local b=$(( 255 - 155 * row / max ))
    printf "%d;%d;%d" $r $g $b
}

show_splash() {
    local -a lines=()
    while IFS= read -r line; do
        lines+=("$line")
    done < <(get_artinx_art)
    local num_lines=${#lines[@]}

    local term_cols term_rows
    term_cols=$(tput cols)
    term_rows=$(tput lines)
    local display_w=60
    local pad_left=$(( (term_cols - display_w) / 2 ))
    local pad_top=$(( (term_rows - num_lines) / 2 - 2 ))
    (( pad_left < 0 )) && pad_left=0
    (( pad_top < 0 )) && pad_top=0

    printf "\033[?25l"
    clear

    # 逐行暗色浮现 (紫→绿)
    for (( row=0; row<num_lines; row++ )); do
        local rgb
        rgb=$(splash_row_color $row $num_lines)
        printf "\033[%d;%dH\033[2;38;2;${rgb}m%s\033[0m" \
            $((pad_top + row + 1)) $((pad_left + 1)) "${lines[$row]}"
        sleep 0.08
    done
    sleep 0.08

    # 亮起
    for (( row=0; row<num_lines; row++ )); do
        local rgb
        rgb=$(splash_row_color $row $num_lines)
        printf "\033[%d;%dH\033[1;38;2;${rgb}m%s\033[0m" \
            $((pad_top + row + 1)) $((pad_left + 1)) "${lines[$row]}"
    done
    sleep 0.15

    # 霓虹呼吸 12 帧
    for (( frame=0; frame<12; frame++ )); do
        for (( row=0; row<num_lines; row++ )); do
            local phase=$(( (frame * 40 + row * 50) % 360 ))
            local r g b
            if (( phase < 120 )); then
                r=$(( 180 - 180 * phase / 120 ))
                g=$(( 100 * phase / 120 ))
                b=$(( 255 - 55 * phase / 120 ))
            elif (( phase < 240 )); then
                local p2=$(( phase - 120 ))
                r=$(( 50 * p2 / 120 ))
                g=$(( 100 + 155 * p2 / 120 ))
                b=$(( 200 - 100 * p2 / 120 ))
            else
                local p3=$(( phase - 240 ))
                r=$(( 50 + 130 * p3 / 120 ))
                g=$(( 255 - 155 * p3 / 120 ))
                b=$(( 100 + 155 * p3 / 120 ))
            fi
            printf "\033[%d;%dH\033[1;38;2;%d;%d;%dm%s\033[0m" \
                $((pad_top + row + 1)) $((pad_left + 1)) $r $g $b "${lines[$row]}"
        done
        sleep 0.08
    done

    # 稳定渐变
    for (( row=0; row<num_lines; row++ )); do
        local rgb
        rgb=$(splash_row_color $row $num_lines)
        printf "\033[%d;%dH\033[1;38;2;${rgb}m%s\033[0m" \
            $((pad_top + row + 1)) $((pad_left + 1)) "${lines[$row]}"
    done

    # 副标题
    local subtitle="R O B O M A S T E R  2 0 2 6"
    local sub_pad=$(( (term_cols - ${#subtitle}) / 2 ))
    printf "\033[%d;%dH\033[38;2;100;255;180m%s\033[0m" \
        $((pad_top + num_lines + 2)) $((sub_pad + 1)) "$subtitle"

    sleep 0.8
}

# ========== 彩虹猫 ==========
rainbow_rgb() {
    local row=$1 total=$2
    local phase=$(( row * 360 / total ))
    local r g b
    if (( phase < 60 )); then
        r=255; g=$(( 255 * phase / 60 )); b=0
    elif (( phase < 120 )); then
        r=$(( 255 - 255 * (phase - 60) / 60 )); g=255; b=0
    elif (( phase < 180 )); then
        r=0; g=255; b=$(( 255 * (phase - 120) / 60 ))
    elif (( phase < 240 )); then
        r=0; g=$(( 255 - 255 * (phase - 180) / 60 )); b=255
    elif (( phase < 300 )); then
        r=$(( 255 * (phase - 240) / 60 )); g=0; b=255
    else
        r=255; g=0; b=$(( 255 - 255 * (phase - 300) / 60 ))
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
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⢋⣷⡄⠈⠉⠓⠶⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡞⢡⣿⣿⣇⠀⠀⠀⠀⠀⠹⠷⠶⠦⠤⢤⣄⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡼⢡⣿⣿⣿⠿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠲⢤⣤⣤⣀⣀⣀⠀⠀⠀  OK~
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠃⣼⡿⠛⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠛⠓⠶⢤⣄⡀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡿⠀⠈⠀⠀⠀⠀⠀⠀⢀⡼⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⣤⠝⡆⠀
⠀⠀⢠⣄⠀⠀⠀⠀⠀⠀⠀⢰⡇⠀⠀⠀⠀⠀⠀⠀⣠⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣞⣯⣽⣶⡟⢠⠇⠀
⠀⢠⢿⣿⡄⠀⠀⠀⠀⠀⢠⠎⠀⠀⠀⠀⠀⠀⠀⣴⣿⢇⠀⠀⠀⠀⢀⠀⠀⠀⠀⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⢨⣷⣿⣿⡟⢀⡞⠀⠀
⠀⠘⠚⠃⠋⠀⠀⠀⠀⢀⠎⠀⠀⠀⠀⠀⣰⠆⡼⠃⣿⠀⠀⠀⠀⠀⡟⠀⠀⠀⢸⣿⡄⠀⠀⠀⣴⣶⣤⣤⣤⣤⠘⣿⠏⢀⡾⠀⠀⠀
⠀⠀⢠⡄⠀⠀⠀⠀⣠⠋⠀⠀⢠⣆⢀⡴⠻⣼⠃⠀⢿⢠⡀⠀⠀⢸⣧⠀⠀⢀⡟⠈⠙⢦⣀⢰⣿⣿⣿⣿⣿⣿⡄⠀⢠⡾⠁⠀⠀⠀
⠀⠀⢸⡇⠀⠀⠰⣏⣑⡤⠀⢠⡟⢹⣿⣿⣿⣿⣷⣶⡌⠋⠓⠲⢤⡼⠿⠴⣆⡾⠀⠀⠀⠀⠈⢻⣿⣿⣿⣿⣿⣿⡇⠰⣿⠁⠀⠀⠀⠀
⠀⠀⠸⡇⠀⠀⠀⠀⢸⠁⠀⢸⡇⠀⠀⠀⠀⠀⠀⠈⠁⠀⠀⠀⠀⠀⠀⠀⢋⣤⣤⣄⡀⠀⠀⠈⢹⡟⠻⠿⠟⠋⡟⠀⣿⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣾⢰⡀⢸⡁⠀⠀⠀⠀⢀⣠⣤⣤⣤⣤⣤⣀⡀⠀⠀⠈⠙⠛⠿⣿⣿⣦⡀⣾⠀⠀⠀⠀⢀⡇⠀⣿⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢿⠸⣿⡮⠇⠀⠀⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⡀⠀⠀⠀⠀⠉⠻⢻⡏⠀⠀⠀⠀⢸⡇⠄⡏⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢸⡸⣝⢷⡄⠀⢰⣿⣿⣿⣿⣿⡿⠟⠛⠻⡟⣿⣿⣿⣿⣦⠀⠀⠀⠀⣠⠟⠀⠀⠀⠀⠀⢸⢇⢰⠇⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢸⣧⣛⣧⠽⠟⢹⣿⣿⣿⣿⣿⣿⠀⠀⠀⢿⢹⣿⣾⣿⣿⣇⠀⢠⣞⣃⣤⡤⠀⠀⠀⠀⣿⡈⡾⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢸⡟⡁⠀⠀⠀⠈⢿⣿⣿⠿⠛⠁⠀⠀⠀⠸⡟⣿⣿⣿⢸⡟⠀⢀⣹⡿⡋⠀⠀⠀⠀⣸⢧⢱⠇⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣾⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⠆⣰⣯⣿⣭⣭⣾⣴⣾⣿⠃⢘⣃⣤⣤⣤⢠⡟⡞⣼⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢀⣼⣿⣿⡖⢢⣀⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⣿⡟⣻⡿⣡⣷⣾⣿⣿⣿⢣⡿⣽⡇⣯⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢀⡴⢫⣾⣿⣿⡇⠀⠏⠀⠗⠉⡆⠀⡄⠐⢯⠔⣴⣿⣦⣿⠟⣛⣵⠞⠿⠾⣻⣿⣿⣿⣿⣫⣿⡿⣿⢿⣹⡄⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠰⢯⣼⣿⣿⣿⣿⣷⡀⠀⠀⠀⠈⢉⣇⠈⠀⢀⣾⣿⣿⣿⣿⣧⣝⣛⢻⣿⣿⣿⣾⣿⣟⣵⣿⣿⡇⣟⠷⠧⠷⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⠦⣤⣀⣀⣸⡿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣏⣿⣿⡿⣯⢾⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠑⠀⠀⠉⠀⠀⠀⠈⠉⠉⠉⠉⠙⠋⠙⠉⠋⠘⠛⠉⠉⠈⠃⠈⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀
MASCOT
    local total=${#mlines[@]}

    # 逐行暗色彩虹浮现
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(rainbow_rgb $i $total)
        printf "\033[%d;1H\033[2;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
        sleep 0.012
    done
    sleep 0.05

    # 亮起彩虹
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(rainbow_rgb $i $total)
        printf "\033[%d;1H\033[1;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
    done
    sleep 0.15

    # 彩虹滚动 8 帧
    for (( frame=0; frame<8; frame++ )); do
        for (( i=0; i<total; i++ )); do
            local shifted=$(( (i + frame * 3) % total ))
            local rgb
            rgb=$(rainbow_rgb $shifted $total)
            printf "\033[%d;1H\033[1;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
        done
        sleep 0.07
    done

    # 定格彩虹
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(rainbow_rgb $i $total)
        printf "\033[%d;1H\033[38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
    done

    echo ""
    echo -e "  \033[38;2;100;255;180m:: ARV_V1 SYSTEM LAUNCHER ::\033[0m"
    echo "  ──────────────────────────────────────────"
    echo ""
    echo ""
}

# 步骤计数器
STEP_CURRENT=0
STEP_TOTAL=6

# 进度: 逐行输出, 每步一行带 spinner
update_status() {
    local text="$1"
    STEP_CURRENT=$((STEP_CURRENT + 1))

    # spinner 动画 (0.6s)
    local frames=("⠋" "⠙" "⠹" "⠸" "⠼" "⠴" "⠦" "⠧" "⠇" "⠏")
    for i in $(seq 0 5); do
        printf "\r  ${CYAN}${frames[$((i % ${#frames[@]}))]}${NC} ${BOLD}[${STEP_CURRENT}/${STEP_TOTAL}]${NC} $text"
        sleep 0.1
    done
    printf "\r  ${GREEN}✓${NC} ${BOLD}[${STEP_CURRENT}/${STEP_TOTAL}]${NC} $text\n"
}

# ========== 工作空间路径 ==========
WORKSPACE_DIR="$HOME/ros2_ws"

# ========== 日志函数 ==========
log_info()    { echo -e "  ${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "  ${GREEN}[OK]${NC} $1"; }
log_warning() { echo -e "  ${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "  ${RED}[ERROR]${NC} $1"; }

# ========== 显示菜单 ==========
show_menu() {
    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║     ARV_V1 机械臂控制系统             ║${NC}"
    echo -e "${GREEN}╠══════════════════════════════════════╣${NC}"
    echo -e "${GREEN}║${NC}  [1] 纯仿真模式 (MuJoCo物理仿真)      ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  [2] 串口真机 + 数字孪生              ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  [0] 退出                            ${GREEN}║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════╝${NC}"
    echo -n "请选择 [0-2]: "
}

# ========== 智能探测串口设备 ==========
detect_serial_device() {
    log_info "自动探测串口设备..."
    local devices=()
    for pattern in /dev/ttyACM* /dev/ttyUSB*; do
        for dev in $pattern; do
            [ -e "$dev" ] && devices+=("$dev")
        done
    done

    if [ ${#devices[@]} -eq 0 ]; then
        log_error "未找到任何串口设备！"
        return 1
    fi

    for dev in "${devices[@]}"; do
        if [ -r "$dev" ] && [ -w "$dev" ]; then
            export DETECTED_SERIAL_DEVICE="$dev"
            log_success "找到可用串口: $dev"
            return 0
        fi
    done

    log_warning "找到设备但权限不足: ${devices[0]}"
    log_info "修复: sudo chmod 666 ${devices[0]}"
    export DETECTED_SERIAL_DEVICE="${devices[0]}"
    return 1
}



# ========== 智能探测 MuJoCo 安装路径 ==========
detect_mujoco_path() {
    log_info "自动探测 MuJoCo 安装路径..."
    local candidates=(
        "$MUJOCO_PATH"
        $(ls -d $HOME/mujoco-* 2>/dev/null | sort -V -r)
        $(ls -d $HOME/.mujoco/mujoco-* 2>/dev/null | sort -V -r)
        "$HOME/.mujoco/mujoco-3.3.7"
        "$HOME/mujoco-3.3.7"
    )

    for path in "${candidates[@]}"; do
        [ -z "$path" ] && continue
        if [ -d "$path" ] && [ -f "$path/lib/libmujoco.so" ]; then
            export MUJOCO_PATH="$path"
            log_success "找到 MuJoCo: $MUJOCO_PATH"
            return 0
        fi
    done

    log_error "未找到 MuJoCo 安装！"
    return 1
}

# ========== 清理残留 DDS 共享内存（防止 FastDDS discovery 死锁）==========
cleanup_dds() {
    local stale
    stale=$(ls /dev/shm/ 2>/dev/null | grep -cE "fast|rtps|ros2" || true)
    if [ "$stale" -gt 0 ]; then
        log_warning "发现 $stale 个残留 DDS shm 文件，清理中..."
        rm -f /dev/shm/fastrtps_* /dev/shm/*rtps* /tmp/fastrtps_* 2>/dev/null || true
        log_success "DDS shm 已清理"
    fi

    # 杀掉同名僵尸进程（静默，不影响正常启动）
    pkill -f "ros2 run arv_v1_moveit" 2>/dev/null || true
    sleep 0.3
}

# ========== 设置环境 ==========
setup_environment() {
    log_info "设置环境变量..."
    if [ ! -d "$WORKSPACE_DIR" ]; then
        log_error "工作空间不存在: $WORKSPACE_DIR"
        exit 1
    fi

    cd "$WORKSPACE_DIR"
    source /opt/ros/jazzy/setup.bash
    [ -f "install/setup.bash" ] && source install/setup.bash

    detect_mujoco_path || exit 1
    log_success "环境设置完成"
}

# ========== 编译项目 ==========
build_workspace() {
    log_info "编译项目..."
    cd "$WORKSPACE_DIR"

    # 注意: 不传 --cmake-args，复用已有 cmake 缓存，避免触发全量重编
    colcon build --packages-select arv_v1_interfaces arv_v1_model arv_v1_moveit

    if [ $? -eq 0 ]; then
        log_success "编译成功！"
        source install/setup.bash
    else
        log_error "编译失败，请检查错误信息"
        exit 1
    fi
}

# ========== 启动节点（在新终端中） ==========
start_node() {
    local node_name=$1
    local command=$2
    local delay=${3:-0}

    log_info "启动: $node_name"

    local mujoco_lib_path=""
    [ -d "$MUJOCO_PATH/lib" ] && mujoco_lib_path="$MUJOCO_PATH/lib"

    gnome-terminal --title="$node_name" -- bash -c "
        cd $WORKSPACE_DIR
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        [ -n '$mujoco_lib_path' ] && export LD_LIBRARY_PATH='$mujoco_lib_path':\"\$LD_LIBRARY_PATH\"
        sleep $delay
        echo -e '${GREEN}========== $node_name ==========${NC}'
        $command
        exec bash
    " &
}

# ========== 模式1: 纯仿真 ==========
start_sim_mode() {
    local config_path="$WORKSPACE_DIR/src/arv_v1_moveit/config/controller_params.yaml"
    local cartesian_config="$WORKSPACE_DIR/src/arv_v1_moveit/config/cartesian_controller_param.yaml"
    STEP_CURRENT=0; STEP_TOTAL=6

    update_status "MoveIt + RViz"
    start_node "MoveIt+RViz" "ros2 launch arv_v1_moveit mujoco_demo.launch.py" 0

    update_status "TorqueController"
    start_node "TorqueController" "ros2 run arv_v1_moveit torque_controller_node --ros-args --params-file $config_path" 0
    sleep 3

    update_status "MissionExecutor (headless)"
    start_node "MissionExecutor" "ros2 run arv_v1_moveit mission_executor_node" 0

    update_status "MuJoCo (仿真)"
    start_node "MuJoCo(仿真)" "ros2 run arv_v1_moveit mujoco_interface_node" 2

    update_status "TrajectoryManager"
    start_node "TrajectoryManager" "ros2 run arv_v1_moveit trajectory_manager_node --ros-args -p trajectory_dir:=$WORKSPACE_DIR/src/arv_v1_moveit/config/trajectories" 1

    update_status "CartesianController (IK)"
    start_node "CartesianController" "ros2 run arv_v1_moveit cartesian_controller_node --ros-args --params-file $cartesian_config" 0

    # Python GUI 工具 (独立窗口)
    log_info "启动 Mission Panel GUI..."
    ros2 run arv_v1_moveit mission_panel.py &
    log_info "启动 Motion Planning Tool..."
    ros2 run arv_v1_moveit move_to_pose.py &
}

# ========== 模式2: 串口 + 数字孪生 ==========
start_serial_mode() {
    local config_path="$WORKSPACE_DIR/src/arv_v1_moveit/config/controller_params.yaml"
    local cartesian_config="$WORKSPACE_DIR/src/arv_v1_moveit/config/cartesian_controller_param.yaml"
    STEP_CURRENT=0; STEP_TOTAL=7

    update_status "探测串口设备"
    if ! detect_serial_device; then
        export DETECTED_SERIAL_DEVICE="/dev/ttyACM0"
    fi

    update_status "MoveIt + RViz"
    start_node "MoveIt+RViz" "ros2 launch arv_v1_moveit mujoco_demo.launch.py" 0

    update_status "TorqueController"
    start_node "TorqueController" "ros2 run arv_v1_moveit torque_controller_node --ros-args --params-file $config_path" 0
    sleep 3

    update_status "MissionExecutor (headless)"
    start_node "MissionExecutor" "ros2 run arv_v1_moveit mission_executor_node" 0

    update_status "SerialInterface ($DETECTED_SERIAL_DEVICE)"
    start_node "SerialInterface" "ros2 run arv_v1_moveit hardware_interface_node --ros-args -p serial_port:=$DETECTED_SERIAL_DEVICE" 2

    update_status "MuJoCo (数字孪生)"
    start_node "MuJoCo(孪生)" "ros2 run arv_v1_moveit mujoco_interface_node --ros-args -p visualization_only:=true" 2

    update_status "TrajectoryManager"
    start_node "TrajectoryManager" "ros2 run arv_v1_moveit trajectory_manager_node --ros-args -p trajectory_dir:=$WORKSPACE_DIR/src/arv_v1_moveit/config/trajectories" 1

    update_status "CartesianController (IK)"
    start_node "CartesianController" "ros2 run arv_v1_moveit cartesian_controller_node --ros-args --params-file $cartesian_config" 0

    # Python GUI 工具 (独立窗口)
    log_info "启动 Mission Panel GUI..."
    ros2 run arv_v1_moveit mission_panel.py &
    log_info "启动 Motion Planning Tool..."
    ros2 run arv_v1_moveit move_to_pose.py &
}



# ========== 主函数 ==========
main() {
    # 阶段1: 环境设置（猫猫出现前完成，避免输出打乱画面）
    cleanup_dds
    setup_environment
    build_workspace

    # 阶段2: ARTINX splash → 猫猫 + 菜单
    show_splash
    show_mascot

    # 菜单选择（覆盖进度条预留行）
    while true; do
        echo -e "\033[2A\033[K  ${GREEN}[1]${NC} 仿真  ${GREEN}[2]${NC} 串口真机+孪生  ${GREEN}[0]${NC} 退出"
        echo -ne "\033[K  请选择 [0-2]: "
        tput cnorm
        read choice
        tput civis

        case $choice in
            1) selected_mode=sim; break ;;
            2) selected_mode=serial; break ;;
            0) tput cnorm; exit 0 ;;
            *) ;;
        esac
    done

    # 阶段3: 猫猫 + 逐步启动
    show_mascot

    if [ "$selected_mode" = "sim" ]; then
        echo -e "  ${CYAN}>> 纯仿真模式${NC}"
        echo ""
        start_sim_mode
    else
        echo -e "  ${CYAN}>> 串口真机 + 数字孪生${NC}"
        echo ""
        start_serial_mode
    fi

    echo ""
    echo -e "  ${GREEN}${BOLD}✓ 所有节点已启动！${NC}"
    echo ""
    echo -e "  ${YELLOW}停止:${NC} ./stop_all_nodes.sh"
    echo -e "  ${YELLOW}诊断:${NC} ./scripts/check_system.sh"
    echo ""
    tput cnorm
    log_info "按 Ctrl+C 退出此脚本（节点继续运行）"
    tail -f /dev/null
}

main
