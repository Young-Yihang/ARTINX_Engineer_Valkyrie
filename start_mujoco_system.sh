#!/bin/bash

################################################################################
# ARV_V1 机械臂控制系统 - 交互式启动脚本
#
# 功能：
#   1. 纯仿真模式 - MuJoCo物理仿真
#   2. 串口真机 + 数字孪生
#   3. SocketCAN真机 + 数字孪生
#
# 使用方法：
#   chmod +x start_mujoco_system.sh
#   ./start_mujoco_system.sh
#
# Author: ARV V1 Team
# Date: 2026-01-08
################################################################################

set -e

# ========== 颜色定义 ==========
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ========== 工作空间路径 ==========
WORKSPACE_DIR="$HOME/ros2_ws"

# ========== 日志函数 ==========
log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

# ========== 显示菜单 ==========
show_menu() {
    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║     ARV_V1 机械臂控制系统             ║${NC}"
    echo -e "${GREEN}╠══════════════════════════════════════╣${NC}"
    echo -e "${GREEN}║${NC}  [1] 纯仿真模式 (MuJoCo物理仿真)      ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  [2] 串口真机 + 数字孪生              ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  [3] SocketCAN真机 + 数字孪生         ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  [0] 退出                            ${GREEN}║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════╝${NC}"
    echo -n "请选择 [0-3]: "
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

# ========== 智能探测CAN设备 ==========
detect_can_device() {
    log_info "自动探测CAN设备..."
    local can_devices=()
    for i in 0 1 2 3; do
        if ip link show "can$i" &>/dev/null; then
            can_devices+=("can$i")
        fi
    done

    if [ ${#can_devices[@]} -eq 0 ]; then
        log_error "未找到任何CAN设备！"
        log_info "请检查USB-CAN适配器并运行:"
        log_info "  sudo ip link set can0 up type can bitrate 1000000"
        return 1
    fi

    for dev in "${can_devices[@]}"; do
        local state=$(ip link show "$dev" | grep -oP 'state \K\w+')
        if [ "$state" = "UP" ]; then
            export DETECTED_CAN_DEVICE="$dev"
            log_success "找到可用CAN接口: $dev (状态: UP)"
            return 0
        fi
    done

    export DETECTED_CAN_DEVICE="${can_devices[0]}"
    log_warning "CAN接口 ${can_devices[0]} 未启动"
    log_info "启动命令: sudo ip link set ${can_devices[0]} up type can bitrate 1000000"
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
    log_info "启动纯仿真模式..."
    start_node "MoveIt+RViz" "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" 0
    start_node "TorqueController" "ros2 run ARV_V1_MOVEIT torque_controller_node" 0
    sleep 3
    start_node "MuJoCo(仿真)" "ros2 run ARV_V1_MOVEIT mujoco_interface_node" 0
}

# ========== 模式2: 串口 + 数字孪生 ==========
start_serial_mode() {
    detect_serial_device || exit 1
    log_info "启动串口真机模式 (设备: $DETECTED_SERIAL_DEVICE)..."
    start_node "MoveIt+RViz" "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" 0
    start_node "TorqueController" "ros2 run ARV_V1_MOVEIT torque_controller_node" 0
    sleep 3
    start_node "SerialInterface" "ros2 run ARV_V1_MOVEIT hardware_interface_node --ros-args -p serial_port:=$DETECTED_SERIAL_DEVICE -p baud_rate:=921600" 0
    start_node "MuJoCo(孪生)" "ros2 run ARV_V1_MOVEIT mujoco_interface_node --ros-args -p visualization_only:=true" 0
}

# ========== 模式3: CAN + 数字孪生 ==========
start_can_mode() {
    detect_can_device || exit 1
    log_info "启动CAN真机模式 (接口: $DETECTED_CAN_DEVICE)..."
    start_node "MoveIt+RViz" "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" 0
    start_node "TorqueController" "ros2 run ARV_V1_MOVEIT torque_controller_node" 0
    sleep 3
    start_node "CANInterface" "ros2 run ARV_V1_MOVEIT can_interface_node --ros-args -p can_interface:=$DETECTED_CAN_DEVICE" 0
    start_node "MuJoCo(孪生)" "ros2 run ARV_V1_MOVEIT mujoco_interface_node --ros-args -p visualization_only:=true" 0
}

# ========== 主函数 ==========
main() {
    setup_environment

    while true; do
        show_menu
        read choice
        case $choice in
            1) start_sim_mode; break ;;
            2) start_serial_mode; break ;;
            3) start_can_mode; break ;;
            0) log_info "退出"; exit 0 ;;
            *) log_warning "无效选择，请重试" ;;
        esac
    done

    echo ""
    log_success "所有节点已启动！"
    echo ""
    echo -e "${YELLOW}提示:${NC}"
    echo "  - 停止所有节点: ./stop_all_nodes.sh"
    echo "  - 查看话题: ros2 topic list"
    echo "  - 查看关节状态: ros2 topic echo /joint_states"
    echo ""

    log_info "按 Ctrl+C 退出此脚本（节点继续运行）"
    tail -f /dev/null
}

main
