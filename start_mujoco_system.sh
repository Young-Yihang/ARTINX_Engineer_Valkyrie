#!/bin/bash

################################################################################
# MuJoCo 力矩控制系统 - 一键启动脚本
#
# 功能：
#   1. 编译项目
#   2. 设置环境变量
#   3. 按顺序启动所有节点（在独立终端中）
#
# 使用方法：
#   chmod +x start_mujoco_system.sh
#   ./start_mujoco_system.sh
#
# 作者：自动生成
# 日期：2025-10-29
################################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作空间路径
WORKSPACE_DIR="$HOME/ros2_ws"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 打印标题
print_header() {
    echo -e "${GREEN}"
    echo "=========================================="
    echo "  MuJoCo 力矩控制系统 - 启动脚本"
    echo "=========================================="
    echo -e "${NC}"
}

# 检查是否在正确的目录
check_directory() {
    if [ ! -d "$WORKSPACE_DIR" ]; then
        log_error "工作空间不存在: $WORKSPACE_DIR"
        exit 1
    fi
    log_success "工作空间检查通过"
}

# 编译项目
build_workspace() {
    log_info "开始编译项目..."
    cd "$WORKSPACE_DIR"

    # 清理旧的编译文件（可选）
    # log_warning "清理旧的编译文件..."
    # rm -rf build/ install/ log/

    # 编译
    log_info "编译 ARV_V1_MODEL 和 ARV_V1_MOVEIT..."
    colcon build --packages-select ARV_V1_MODEL ARV_V1_MOVEIT --cmake-args -DCMAKE_BUILD_TYPE=Release

    if [ $? -eq 0 ]; then
        log_success "编译成功！"
    else
        log_error "编译失败，请检查错误信息"
        exit 1
    fi
}

# 智能探测 MuJoCo 安装路径
detect_mujoco_path() {
    log_info "自动探测 MuJoCo 安装路径..."

    # 候选路径列表（按优先级排序）
    local candidates=(
        "$MUJOCO_PATH"                                          # 用户预设环境变量
        "$HOME/.mujoco/mujoco-3.3.7"                            # 标准安装位置
        "$HOME/mujoco-3.3.7"
        "$HOME/mujoco-3.4.0"
        "$HOME/.mujoco/mujoco-3.4.0"
        "/usr/local/mujoco"                                      # 系统级安装
        $(ls -d $HOME/.local/lib/python*/site-packages/mujoco 2>/dev/null | head -1)  # Python包
    )

    for path in "${candidates[@]}"; do
        if [ -z "$path" ]; then
            continue
        fi

        # 检查路径存在且包含必要文件
        if [ -d "$path" ]; then
            # 检查库文件（支持带版本号和不带版本号）
            local lib_found=false
            if [ -f "$path/lib/libmujoco.so" ] || ls "$path/lib/libmujoco.so"* >/dev/null 2>&1; then
                lib_found=true
            elif [ -f "$path/libmujoco.so" ] || ls "$path/libmujoco.so"* >/dev/null 2>&1; then
                lib_found=true
            fi

            # 检查头文件
            local header_found=false
            if [ -f "$path/include/mujoco/mujoco.h" ]; then
                header_found=true
            elif [ -f "$path/mujoco.h" ]; then
                header_found=true
            fi

            if [ "$lib_found" = true ] && [ "$header_found" = true ]; then
                export MUJOCO_PATH="$path"
                log_success "找到 MuJoCo: $MUJOCO_PATH"
                check_mujoco_version "$path"
                return 0
            fi
        fi
    done

    log_error "未找到有效的 MuJoCo 安装！"
    log_error "请检查以下候选路径是否包含 libmujoco.so 和 mujoco.h:"
    for path in "${candidates[@]}"; do
        [ -n "$path" ] && echo "  - $path"
    done
    exit 1
}

# 检查 MuJoCo 版本兼容性
check_mujoco_version() {
    local mujoco_path=$1
    local version_file=""

    # 查找版本信息文件
    if [ -f "$mujoco_path/include/mujoco/mjversion.h" ]; then
        version_file="$mujoco_path/include/mujoco/mjversion.h"
    elif [ -f "$mujoco_path/mjversion.h" ]; then
        version_file="$mujoco_path/mjversion.h"
    fi

    if [ -n "$version_file" ] && [ -f "$version_file" ]; then
        local version=$(grep -oP '(?<=#define mjVERSION )\d+' "$version_file" 2>/dev/null | head -1)
        if [ -n "$version" ]; then
            log_info "MuJoCo 版本: $(echo "$version" | sed 's/\([0-9]\)\([0-9]\)\([0-9]\)/\1.\2.\3/')" 
            
            # 警告低版本
            if [ "$version" -lt 330 ]; then
                log_warning "检测到 MuJoCo 版本 < 3.3.0，建议升级以获得最佳兼容性"
            fi
        fi
    else
        log_warning "无法确定 MuJoCo 版本（未找到 mjversion.h）"
    fi
}

# 设置环境变量
setup_environment() {
    log_info "设置 ROS2 环境变量..."
    cd "$WORKSPACE_DIR"

    source /opt/ros/jazzy/setup.bash
    # 如果本地已安装工作区（install），再 source 本地 setup
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi

    # 智能探测 MuJoCo 路径（仅用于编译时头文件查找）
    detect_mujoco_path

    # 注意：运行时库查找由系统 ldconfig 处理（/etc/ld.so.conf.d/mujoco.conf）
    # 如果未配置 ldconfig，请运行：sudo bash -c 'echo "$MUJOCO_PATH/lib" > /etc/ld.so.conf.d/mujoco.conf' && sudo ldconfig

    log_success "环境变量设置完成（MUJOCO_PATH=$MUJOCO_PATH）"
}

# 启动节点（在新终端中）
start_node() {
    local node_name=$1
    local command=$2
    local delay=$3

    log_info "启动节点: $node_name (延迟 ${delay}s)"

    # 注意：MuJoCo 库路径由系统 ldconfig 管理，无需设置 LD_LIBRARY_PATH
    gnome-terminal --title="$node_name" -- bash -c "
        cd $WORKSPACE_DIR
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        
        sleep $delay
        echo -e '${GREEN}=========================================='
        echo ' 节点: $node_name'
        echo '==========================================${NC}'
        $command
        exec bash
    " &

    log_success "$node_name 终端已启动"
}

# 主函数
main() {
    print_header

    # 步骤1：检查目录
    log_info "步骤 1/5: 检查工作空间"
    check_directory
    echo ""

    # 步骤2：设置环境（在编译前设置 MUJOCO_PATH/ROS 环境）
    log_info "步骤 2/5: 设置环境变量"
    setup_environment
    echo ""

    # 步骤3：编译项目
    log_info "步骤 3/5: 编译项目"
    build_workspace
    echo ""

    # 步骤4：启动节点
    log_info "步骤 4/5: 启动所有节点"
    echo ""


    # 节点3：MoveIt + RViz
    start_node \
        "3. MoveIt + RViz" \
        "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" \
        0
        
    # 节点1：力矩控制器（先启动，确保重力补偿就绪）
    start_node \
        "1. Torque Controller" \
        "ros2 run ARV_V1_MOVEIT torque_controller_node" \
        0

    sleep 3  # 等待控制器完全启动并开始发布重力补偿

    # 节点2：MuJoCo 仿真节点（后启动，避免机械臂下落）
    start_node \
        "2. MuJoCo Interface" \
        "ros2 run ARV_V1_MOVEIT mujoco_interface_node" \
        0

    sleep 2  # 等待 MuJoCo 节点启动

    echo ""
    log_success "所有节点已启动！"
    echo ""

    # 步骤5：显示系统信息
    log_info "步骤 5/5: 系统信息"
    echo ""
    echo -e "${YELLOW}=========================================="
    echo "  系统已启动，节点信息："
    echo "==========================================${NC}"
    echo ""
    echo -e "${GREEN}终端 1:${NC} Torque Controller Node"
    echo "  - 功能: 动力学计算 + PD控制"
    echo "  - 发布: /effort_controller/commands"
    echo "  - 订阅: /joint_states"
    echo "  - Action: /ARM_controller/follow_joint_trajectory"
    echo "  - 启动时立即发送重力补偿（防止机械臂下落）"
    echo ""
    echo -e "${GREEN}终端 2:${NC} MuJoCo Interface Node"
    echo "  - 功能: MuJoCo 物理仿真 + 3D可视化"
    echo "  - 发布: /joint_states (200 Hz)"
    echo "  - 订阅: /effort_controller/commands"
    echo ""
    echo -e "${GREEN}终端 3:${NC} MoveIt + RViz"
    echo "  - 功能: 轨迹规划 + 可视化"
    echo "  - 使用: 在 RViz 中拖动机械臂并执行"
    echo ""
    echo -e "${YELLOW}=========================================="
    echo "  有用的调试命令："
    echo "==========================================${NC}"
    echo ""
    echo "# 查看话题列表"
    echo "ros2 topic list"
    echo ""
    echo "# 查看关节状态"
    echo "ros2 topic echo /joint_states"
    echo ""
    echo "# 查看力矩命令"
    echo "ros2 topic echo /effort_controller/commands"
    echo ""
    echo "# 查看节点图"
    echo "rqt_graph"
    echo ""
    echo "# 查看 Action 列表"
    echo "ros2 action list"
    echo ""
    echo -e "${GREEN}=========================================="
    echo "  测试力矩控制（可选）："
    echo "==========================================${NC}"
    echo ""
    echo "# 在新终端中手动发布测试力矩"
    echo "ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \\"
    echo "  \"data: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]\" --rate 100"
    echo ""
    echo -e "${YELLOW}按 Ctrl+C 可以在各个终端中停止节点${NC}"
    echo ""
}

# 运行主函数
main

# 保持脚本运行，方便查看日志
log_info "脚本执行完成，按 Ctrl+C 退出"
tail -f /dev/null
