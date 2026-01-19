#!/bin/bash
# ARV_V1启动脚本: 仿真模式 / 串口真机+孪生模式

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

    colcon build --packages-select arv_v1_interfaces ARV_V1_MODEL ARV_V1_MOVEIT --cmake-args -DCMAKE_BUILD_TYPE=Release
    
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
    log_info "启动纯仿真模式..."
    local config_path="$WORKSPACE_DIR/src/ARV_V1_MOVEIT/config/controller_params.yaml"
    start_node "MoveIt+RViz" "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" 0
    start_node "TorqueController" "ros2 run ARV_V1_MOVEIT torque_controller_node --ros-args --params-file $config_path" 0
    sleep 3
    start_node "MuJoCo(仿真)" "ros2 run ARV_V1_MOVEIT mujoco_interface_node" 0
    start_node "TrajectoryManager" "ros2 run ARV_V1_MOVEIT trajectory_manager_node" 1
}

# ========== 模式2: 串口 + 数字孪生 ==========
start_serial_mode() {
    # 尝试检测串口，但失败也不退出
    if detect_serial_device; then
        log_success "使用检测到的串口: $DETECTED_SERIAL_DEVICE"
    else
        log_warning "串口检测失败，使用默认设备 /dev/ttyACM0（节点会自动重连）"
        export DETECTED_SERIAL_DEVICE="/dev/ttyACM0"
    fi

    log_info "启动串口真机模式 (设备: $DETECTED_SERIAL_DEVICE)..."
    local config_path="$WORKSPACE_DIR/src/ARV_V1_MOVEIT/config/controller_params.yaml"
    start_node "MoveIt+RViz" "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" 0
    start_node "TorqueController" "ros2 run ARV_V1_MOVEIT torque_controller_node --ros-args --params-file $config_path" 0
    sleep 3
    start_node "SerialInterface" "ros2 run ARV_V1_MOVEIT hardware_interface_node --ros-args -p serial_port:=$DETECTED_SERIAL_DEVICE -p baud_rate:=921600" 0
    start_node "MuJoCo(孪生)" "ros2 run ARV_V1_MOVEIT mujoco_interface_node --ros-args -p visualization_only:=true" 0
    start_node "TrajectoryManager" "ros2 run ARV_V1_MOVEIT trajectory_manager_node" 1
}



# ========== 主函数 ==========
main() {
    setup_environment
    build_workspace

    while true; do
        show_menu
        read choice
        case $choice in
            1) start_sim_mode; break ;;
            2) start_serial_mode; break ;;
            0) log_info "退出"; exit 0 ;;
            *) log_warning "无效选择，请重试" ;;
        esac
    done

    echo ""
    log_success "所有节点已启动！"
    echo ""
    echo -e "${YELLOW}常用命令:${NC}"
    echo "  - 停止所有节点: ./stop_all_nodes.sh"
    echo "  - 查看话题: ros2 topic list"
    echo "  - 查看关节状态: ros2 topic echo /joint_states"
    echo ""
    echo -e "${YELLOW}轨迹管理服务:${NC}"
    echo "  列出轨迹: ros2 service call /list_trajectories arv_v1_interfaces/srv/ListTrajectories"
    echo ""
    echo "  保存最近执行的轨迹 (先在RViz中Plan&Execute):"
    echo "    ros2 service call /save_last_trajectory arv_v1_interfaces/srv/SaveLastTrajectory \\"
    echo "        \"{name: 'my_traj', description: '我的轨迹'}\""
    echo ""
    echo "  加载并执行轨迹:"
    echo "    ros2 service call /load_trajectory arv_v1_interfaces/srv/LoadTrajectory \\"
    echo "        \"{name: 'my_traj', execute: true}\""
    echo ""

    log_info "按 Ctrl+C 退出此脚本（节点继续运行）"
    tail -f /dev/null
}

main
