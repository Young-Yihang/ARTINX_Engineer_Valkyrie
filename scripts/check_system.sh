#!/bin/bash
################################################################################
# ARV_V1 系统链路检查工具
# 快速诊断: 节点状态、话题频率、参数完整性、通信链路
# 用法: ./check_system.sh
################################################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
fail() { echo -e "${RED}[✗]${NC} $1"; }

echo ""
echo "========================================"
echo "  ARV_V1 系统链路检查"
echo "========================================"
echo ""

# ========== 1. 节点检查 ==========
echo "--- 节点状态 ---"
NODES=$(ros2 node list 2>/dev/null)

check_node() {
    local name=$1
    local desc=$2
    if echo "$NODES" | grep -q "$name"; then
        ok "$desc ($name)"
        return 0
    else
        fail "$desc ($name) 未运行"
        return 1
    fi
}

check_node "torque_controller" "力矩控制器"
CTRL_OK=$?
check_node "move_group" "MoveIt规划器"
check_node "mujoco_interface" "MuJoCo仿真" || check_node "hardware_interface" "硬件接口"
check_node "robot_state_publisher" "TF发布器"

echo ""

# ========== 2. 话题频率检查 ==========
echo "--- 话题频率 (采样2秒) ---"

check_hz() {
    local topic=$1
    local expected=$2
    local name=$3

    # 采样2秒获取频率
    local hz=$(timeout 2.5 ros2 topic hz "$topic" 2>/dev/null | grep "average rate:" | head -1 | awk '{print $3}')

    if [ -z "$hz" ]; then
        fail "$name: 无数据"
        return 1
    fi

    # 转换为整数比较
    local hz_int=${hz%.*}
    local exp_low=$((expected * 8 / 10))  # 80%阈值

    if [ "$hz_int" -ge "$exp_low" ]; then
        ok "$name: ${hz} Hz (期望 ${expected} Hz)"
        return 0
    else
        warn "$name: ${hz} Hz (期望 ${expected} Hz) - 频率偏低"
        return 1
    fi
}

check_hz "/joint_states" 200 "关节状态"
check_hz "/effort_controller/commands" 200 "力矩指令"

echo ""

# ========== 3. 话题连通性 ==========
echo "--- 话题连通性 ---"

check_topic() {
    local topic=$1
    local desc=$2
    if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        ok "$desc ($topic) 有发布者"
    else
        fail "$desc ($topic) 无发布者"
    fi
}

check_topic "/joint_states" "关节状态"
check_topic "/effort_controller/commands" "力矩指令"
check_topic "/ARM_controller/follow_joint_trajectory/_action/status" "Action状态"

echo ""

# ========== 4. 参数检查 (需要控制器运行) ==========
if [ $CTRL_OK -eq 0 ]; then
    echo "--- 控制器参数 ---"

    # 检查关键参数是否存在
    params=$(ros2 param list /torque_controller_action_server 2>/dev/null)

    if echo "$params" | grep -q "Kp.joint_1"; then
        ok "PD增益参数已加载"
    else
        fail "PD增益参数未加载"
    fi

    if echo "$params" | grep -q "kalman.enabled"; then
        # 获取kalman状态
        kalman=$(ros2 param get /torque_controller_action_server kalman.enabled 2>/dev/null | grep -o "True\|False")
        if [ "$kalman" = "True" ]; then
            ok "Kalman滤波: 启用"
        else
            warn "Kalman滤波: 禁用"
        fi
    fi

    if echo "$params" | grep -q "use_cascade_pid"; then
        pid=$(ros2 param get /torque_controller_action_server use_cascade_pid 2>/dev/null | grep -o "True\|False")
        if [ "$pid" = "True" ]; then
            ok "控制模式: 级联PID"
        else
            ok "控制模式: PD"
        fi
    fi
    echo ""
fi

# ========== 5. TF检查 ==========
echo "--- TF坐标系 ---"
tf_frames=$(ros2 run tf2_ros tf2_echo base_link link6_2006roll 2>&1 | head -5)
if echo "$tf_frames" | grep -q "Translation"; then
    ok "TF链完整 (base_link -> link6_2006roll)"
else
    fail "TF链不完整，检查URDF和robot_state_publisher"
fi

echo ""

# ========== 6. 系统资源 ==========
echo "--- 系统资源 ---"
cpu=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}')
mem=$(free -m | awk 'NR==2{printf "%.1f", $3*100/$2}')
echo "  CPU使用: ${cpu}%"
echo "  内存使用: ${mem}%"

# 检查控制器进程CPU
if [ $CTRL_OK -eq 0 ]; then
    ctrl_cpu=$(ps aux | grep torque_controller | grep -v grep | awk '{print $3}')
    if [ -n "$ctrl_cpu" ]; then
        echo "  控制器CPU: ${ctrl_cpu}%"
    fi
fi

echo ""
echo "========================================"
echo "  检查完成"
echo "========================================"
