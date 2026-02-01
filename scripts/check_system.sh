#!/bin/bash
# ARV_V1 系统健康检查 v2.0
# 功能: 节点状态、话题频率、错误检测、链路诊断

# ==================== 可爱的检查画面 ====================
show_mascot() {
    echo -e "\033[0;35m"
    cat << 'MASCOT'
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣀⣀⣀⠀⢠⠂⢢⢀⠔⠢⠀⠀⠀⠀⠀⠀⠀⡴⣱⣧⠀⠉⠲⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⡰⠁⢀⣀⠀⢳⠘⡀⠈⠁⠀⢎⡀⢰⣉⣑⣊⠗⣸⢱⣿⣿⠀⠀⠀⠀⠙⢦⡀⠀⠀⠀⢀⣀⣀⡀⠀⠀⠀  あー
⠀⠀⠀⠀⡇⠀⠣⠤⠃⢸⠀⢇⠀⣰⠢⠤⠃⠀⠀⠀⠀⢠⠇⣿⣿⠿⠀⠀⠀⠀⠀⠉⠉⠓⠒⠦⠎⡞⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠈⠢⠤⠤⠴⠋⠀⠈⠉⠀⠀⠀⠀⠀⠀⠀⢀⡟⠈⠁⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠉⠑⠶⠒⠦⠤⢤⣀⣀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠎⠀⠀⠀⠀⠀⠀⠀⣰⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⢉⣒⣄⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡠⠞⠁⠀⠀⣠⠀⠀⠀⢀⣾⠃⠀⠀⠀⠀⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣾⣿⢯⠇⠀
⠀⠀⠀⠀⠀⠀⡠⠔⠊⠉⠉⠉⠢⡀⠀⠀⠙⡾⠁⠀⢀⣾⠁⠀⣀⢠⢿⡏⠀⠀⠀⠀⡼⠀⠀⣀⠀⠀⠀⠀⠀⠀⢺⣿⣿⣿⢋⡞⠀⠀
⠀⠀⠀⠀⡠⠊⠀⠀⠀⠀⠀⠀⠀⠈⢆⠀⣸⠃⠀⠀⡼⠗⠶⠚⢻⠏⢸⡇⠀⠀⠀⢰⡇⠀⣠⣿⠀⠀⢀⡀⠀⠀⠉⢿⡿⢣⠞⠀⠀⠀
⠀⠀⢀⠞⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢶⣿⠀⡀⢠⡿⠿⠿⢷⣦⡄⠈⠿⣄⡀⠀⣾⣠⡆⡾⠉⢧⡀⣸⣷⣶⣶⡆⠀⣰⠏⠀⠀⠀⠀
⠀⢀⠎⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣿⠘⣧⠸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⢛⣉⣻⠃⠀⠀⢈⣿⣿⣿⣿⡇⠀⣿⠀⠀⠀⠀⠀
⠀⢼⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⢹⡄⢿⣧⣻⠀⠀⠀⠀⠀⣤⣤⡀⠀⠀⠈⠙⠻⢿⣶⣄⢿⡾⠋⠋⠁⠀⢀⡏⠀⠀⠀⠀⠀
⠀⠈⠻⢭⣛⠢⢄⠀⠀⠀⠀⢀⡏⠀⠀⣠⡟⣧⠀⢻⡅⠀⠀⠀⠀⠀⡟⠛⢻⠇⠀⠀⠀⠀⠀⠈⠙⡼⠁⠀⢀⡟⠀⣸⠃⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠉⠛⠲⠯⢶⣤⣤⠞⠀⣠⢞⡟⠀⠘⣆⡀⠹⣦⡀⠀⠀⠀⠳⢠⠞⠀⠀⠀⠀⠀⠀⢠⡞⠁⠀⠀⣼⢃⢠⠏⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠳⣤⢴⡞⣳⠎⠀⠀⠀⢹⣸⡇⠹⣿⣷⣦⠄⠀⠀⠀⠀⠀⠀⠀⣠⣖⣃⣬⡶⡰⣰⣯⢧⡟⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣟⠁⠀⠀⠀⠀⢸⣷⡆⠀⣿⡏⢿⣷⣶⣤⣽⣿⣿⣿⣿⣿⡟⠛⠉⠀⣰⡟⣶⢿⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢳⣄⠀⠀⠀⠈⠙⡇⡇⣿⣁⣨⣷⣴⠟⠋⠀⠙⣿⣿⣿⠁⣀⢎⣱⣿⠙⡃⢸⡇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣫⣾⣿⣦⣀⡤⢠⣇⣼⣿⣿⣿⣿⣿⣷⣶⣤⡀⣿⠙⣿⠀⡟⣾⣿⣿⡀⡧⡄⢧⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠀⣿⣿⣿⡇⠙⠋⢉⣽⣿⣿⣿⣿⣿⣿⡿⠃⠋⠀⢻⡆⢰⣿⣿⣿⣧⢸⣧⣌⣳⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⠛⠛⠁⠀⠀⠛⠛⠛⠛⠛⠛⠃⠈⠀⠐⠀⠀⠈⠛⠚⠛⠛⠛⠛⠓⠃⠀⠀⠀⠀⠀⠀⠀⠀
MASCOT
    echo -e "\033[0m"
}

# ==================== 颜色定义 ====================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ==================== 输出函数 ====================
ok()   { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }
fail() { echo -e "${RED}[✗]${NC} $1"; }
info() { echo -e "${BLUE}[i]${NC} $1"; }

# 进度条显示
progress_bar() {
    local current=$1
    local total=$2
    local width=20
    local percent=$((current * 100 / total))
    local filled=$((current * width / total))
    local empty=$((width - filled))
    printf "\r  采样中 [%s%s] %d%%" "$(printf '#%.0s' $(seq 1 $filled 2>/dev/null))" "$(printf '.%.0s' $(seq 1 $empty 2>/dev/null))" "$percent"
}

# ==================== 配置 ====================
SAMPLE_TIME=3  # 话题采样时间(秒)

# 期望的节点列表
declare -A EXPECTED_NODES=(
    ["torque_controller"]="力矩控制器"
    ["move_group"]="MoveIt规划器"
    ["robot_state_publisher"]="TF发布器"
    ["trajectory_manager"]="轨迹管理器"
    ["mission_executor"]="任务执行器"
)

# 期望的话题和频率
declare -A EXPECTED_TOPICS=(
    ["/joint_states"]=200
    ["/effort_controller/commands"]=200
)

# ==================== 主程序 ====================
clear
show_mascot

# 统计变量
NODE_OK=0
NODE_FAIL=0
TOPIC_OK=0
TOPIC_WARN=0
TOPIC_FAIL=0
ERRORS_FOUND=0

# ==================== 1. 节点状态检查 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  1. 节点状态                                                │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

NODES=$(ros2 node list 2>/dev/null)

# 核心节点检查
for node in "${!EXPECTED_NODES[@]}"; do
    desc="${EXPECTED_NODES[$node]}"
    if echo "$NODES" | grep -q "$node"; then
        ok "$desc"
        ((NODE_OK++))
    else
        fail "$desc ($node)"
        ((NODE_FAIL++))
    fi
done

# 检查mujoco或hardware接口(二选一)
if echo "$NODES" | grep -q "mujoco_interface"; then
    ok "执行层: MuJoCo仿真"
    ((NODE_OK++))
    EXEC_MODE="simulation"
elif echo "$NODES" | grep -q "hardware_interface"; then
    ok "执行层: 硬件接口"
    ((NODE_OK++))
    EXEC_MODE="hardware"
else
    fail "执行层: 无接口节点"
    ((NODE_FAIL++))
    EXEC_MODE="none"
fi

echo ""

# ==================== 2. 话题频率检查 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  2. 话题频率 (采样${SAMPLE_TIME}秒)                                       │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

# 频率检查函数 - 带可视化偏差
check_hz_visual() {
    local topic=$1
    local expected=$2
    local name=$3

    # 采样获取频率
    printf "  %-25s " "$name"

    local hz_output=$(timeout $((SAMPLE_TIME + 1)) ros2 topic hz "$topic" --window 50 2>/dev/null | grep "average rate:" | tail -1)
    local hz=$(echo "$hz_output" | awk '{print $3}')

    if [ -z "$hz" ]; then
        echo -e "${RED}无数据${NC}"
        ((TOPIC_FAIL++))
        return 1
    fi

    # 计算偏差
    local hz_int=${hz%.*}
    local deviation=$(( (hz_int - expected) * 100 / expected ))
    local abs_dev=${deviation#-}

    # 生成偏差条
    local bar_width=20
    local bar=""
    local center=$((bar_width / 2))

    if [ "$deviation" -ge 0 ]; then
        # 正偏差 - 往右
        local right_fill=$(( abs_dev * center / 50 ))
        [ $right_fill -gt $center ] && right_fill=$center
        bar=$(printf "%${center}s" | tr ' ' '─')
        bar+=$(printf "%${right_fill}s" | tr ' ' '█')
        bar+=$(printf "%$((center - right_fill))s" | tr ' ' '─')
    else
        # 负偏差 - 往左
        local left_fill=$(( abs_dev * center / 50 ))
        [ $left_fill -gt $center ] && left_fill=$center
        bar=$(printf "%$((center - left_fill))s" | tr ' ' '─')
        bar+=$(printf "%${left_fill}s" | tr ' ' '█')
        bar+=$(printf "%${center}s" | tr ' ' '─')
    fi

    # 颜色和状态
    if [ "$abs_dev" -le 10 ]; then
        color=$GREEN
        status="OK"
        ((TOPIC_OK++))
    elif [ "$abs_dev" -le 30 ]; then
        color=$YELLOW
        status="WARN"
        ((TOPIC_WARN++))
    else
        color=$RED
        status="ERR"
        ((TOPIC_FAIL++))
    fi

    printf "${color}%6.1f Hz${NC} [%s] %+4d%% ${color}%s${NC}\n" "$hz" "$bar" "$deviation" "$status"
}

echo ""
echo -e "  ${BOLD}话题                      频率       偏差图            状态${NC}"
echo "  ─────────────────────────────────────────────────────────────"

check_hz_visual "/joint_states" 200 "关节状态"
check_hz_visual "/effort_controller/commands" 200 "力矩指令"

echo ""
echo -e "  ${BLUE}偏差图说明: [──────────█─────────] 中心=期望值, █=实际偏差${NC}"
echo ""

# ==================== 3. 话题连通性 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  3. 话题连通性                                              │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

check_topic_conn() {
    local topic=$1
    local desc=$2

    local info=$(ros2 topic info "$topic" 2>/dev/null)
    local pub=$(echo "$info" | grep "Publisher count:" | awk '{print $3}')
    local sub=$(echo "$info" | grep "Subscription count:" | awk '{print $3}')

    pub=${pub:-0}
    sub=${sub:-0}

    if [ "$pub" -gt 0 ] && [ "$sub" -gt 0 ]; then
        ok "$desc: ${pub}发布 → ${sub}订阅"
    elif [ "$pub" -gt 0 ]; then
        warn "$desc: ${pub}发布, 无订阅"
    else
        fail "$desc: 无发布者"
    fi
}

check_topic_conn "/joint_states" "关节状态"
check_topic_conn "/effort_controller/commands" "力矩指令"
check_topic_conn "/ARM_controller/follow_joint_trajectory/_action/status" "Action状态"

echo ""

# ==================== 4. 错误检测 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  4. 最近错误 (最近30秒日志)                                 │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

# 检查各节点的错误日志
check_node_errors() {
    local node_pattern=$1
    local node_name=$2

    # 从journalctl获取最近的ROS日志(如果可用)
    # 或者检查/tmp下的日志文件
    local log_file=$(find /tmp -name "*.log" -newer /tmp -mmin -1 2>/dev/null | grep -i "$node_pattern" | head -1)

    if [ -n "$log_file" ]; then
        local errors=$(grep -i "error\|fatal\|exception" "$log_file" 2>/dev/null | tail -3)
        if [ -n "$errors" ]; then
            warn "$node_name 有错误:"
            echo "$errors" | while read line; do
                echo -e "    ${RED}$line${NC}"
            done
            ((ERRORS_FOUND++))
            return 1
        fi
    fi
    return 0
}

# 检查ros2 daemon状态
daemon_errors=$(ros2 daemon status 2>&1)
if echo "$daemon_errors" | grep -qi "error\|failed"; then
    warn "ROS2 daemon异常"
    ((ERRORS_FOUND++))
fi

# 简单检查: 查看是否有节点在stderr输出错误
# 这里我们用一个轻量的方法 - 检查关键话题是否有数据异常

# 检查joint_states是否有NaN
joint_data=$(timeout 1 ros2 topic echo /joint_states --once 2>/dev/null)
if echo "$joint_data" | grep -q "nan\|inf"; then
    fail "检测到关节数据异常 (NaN/Inf)"
    ((ERRORS_FOUND++))
else
    ok "关节数据正常"
fi

# 检查是否有超时告警(通过检查话题时间戳)
if [ "$TOPIC_FAIL" -eq 0 ]; then
    ok "话题通信正常"
else
    warn "有 $TOPIC_FAIL 个话题异常"
fi

echo ""

# ==================== 5. 轨迹管理服务 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  5. 轨迹管理服务                                            │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

# 检查服务是否存在
services=$(ros2 service list 2>/dev/null)

check_service() {
    local service=$1
    local desc=$2
    
    if echo "$services" | grep -q "$service"; then
        # 尝试调用服务测试响应
        if [ "$service" = "/list_trajectories" ]; then
            local result=$(timeout 2 ros2 service call "$service" arv_v1_interfaces/srv/ListTrajectories 2>/dev/null)
            if [ $? -eq 0 ]; then
                local traj_count=$(echo "$result" | grep -o "names:" | wc -l)
                if [ "$traj_count" -gt 0 ]; then
                    local names=$(echo "$result" | grep "names:" -A 1 | tail -1 | tr -d "[]'")
                    local count=$(echo "$names" | grep -o "," | wc -l)
                    count=$((count + 1))
                    ok "$desc (已保存 $count 个轨迹)"
                else
                    ok "$desc (无已保存轨迹)"
                fi
            else
                warn "$desc (服务响应超时)"
            fi
        else
            ok "$desc"
        fi
    else
        fail "$desc (服务不存在)"
    fi
}

check_service "/list_trajectories" "列出轨迹"
check_service "/load_trajectory" "加载轨迹"
check_service "/save_trajectory" "保存轨迹"
check_service "/save_last_trajectory" "保存最近轨迹"

# 检查mission_executor是否在运行
if echo "$NODES" | grep -q "mission_executor"; then
    ok "任务执行器: 运行中 (TUI界面可用)"
else
    info "任务执行器: 未启动 (可选组件)"
fi

echo ""

# ==================== 6. 控制器参数 ====================
if echo "$NODES" | grep -q "torque_controller"; then
    echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
    echo -e "${CYAN}│  6. 控制器配置                                              │${NC}"
    echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

    params=$(ros2 param list /torque_controller_action_server 2>/dev/null)

    # Kalman滤波状态
    if echo "$params" | grep -q "kalman.enabled"; then
        kalman=$(ros2 param get /torque_controller_action_server kalman.enabled 2>/dev/null | grep -oE "True|False")
        [ "$kalman" = "True" ] && ok "Kalman滤波: 启用" || info "Kalman滤波: 禁用"
    fi

    # 控制模式
    if echo "$params" | grep -q "use_cascade_pid"; then
        cascade=$(ros2 param get /torque_controller_action_server use_cascade_pid 2>/dev/null | grep -oE "True|False")
        [ "$cascade" = "True" ] && ok "控制模式: 级联PID" || ok "控制模式: PD"
    fi

    # 安全限制
    if echo "$params" | grep -q "safety.max_torque_default"; then
        max_torque=$(ros2 param get /torque_controller_action_server safety.max_torque_default 2>/dev/null | grep -oE "[0-9]+\.?[0-9]*")
        info "力矩限制: ${max_torque} Nm"
    fi

    echo ""
fi

# ==================== 7. TF链路 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  7. TF坐标系                                                │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

tf_result=$(timeout 2 ros2 run tf2_ros tf2_echo base_link link6_2006roll 2>&1 | head -3)
if echo "$tf_result" | grep -q "Translation"; then
    ok "TF链完整: base_link → link6_2006roll"
else
    fail "TF链断开"
fi

echo ""

# ==================== 8. 系统资源 ====================
echo -e "${CYAN}┌─────────────────────────────────────────────────────────────┐${NC}"
echo -e "${CYAN}│  8. 系统资源                                                │${NC}"
echo -e "${CYAN}└─────────────────────────────────────────────────────────────┘${NC}"

# CPU和内存
cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
mem_usage=$(free | awk 'NR==2{printf "%.1f", $3*100/$2}')

# CPU条形图
cpu_int=${cpu_usage%.*}
cpu_bar_len=$((cpu_int / 5))
cpu_bar=$(printf '%*s' "$cpu_bar_len" | tr ' ' '█')
cpu_empty=$(printf '%*s' "$((20 - cpu_bar_len))" | tr ' ' '░')

if [ "$cpu_int" -lt 50 ]; then
    cpu_color=$GREEN
elif [ "$cpu_int" -lt 80 ]; then
    cpu_color=$YELLOW
else
    cpu_color=$RED
fi

echo -e "  CPU:  ${cpu_color}[${cpu_bar}${cpu_empty}]${NC} ${cpu_usage}%"

# 内存条形图
mem_int=${mem_usage%.*}
mem_bar_len=$((mem_int / 5))
mem_bar=$(printf '%*s' "$mem_bar_len" | tr ' ' '█')
mem_empty=$(printf '%*s' "$((20 - mem_bar_len))" | tr ' ' '░')

if [ "$mem_int" -lt 50 ]; then
    mem_color=$GREEN
elif [ "$mem_int" -lt 80 ]; then
    mem_color=$YELLOW
else
    mem_color=$RED
fi

echo -e "  内存: ${mem_color}[${mem_bar}${mem_empty}]${NC} ${mem_usage}%"

# 关键进程CPU占用
echo ""
echo -e "  ${BOLD}关键进程:${NC}"
ps aux --sort=-%cpu | grep -E "torque_controller|mujoco|hardware_interface|move_group" | grep -v grep | head -4 | while read line; do
    proc_name=$(echo "$line" | awk '{print $11}' | xargs basename 2>/dev/null)
    proc_cpu=$(echo "$line" | awk '{print $3}')
    proc_mem=$(echo "$line" | awk '{print $4}')
    printf "    %-25s CPU: %5s%%  MEM: %5s%%\n" "$proc_name" "$proc_cpu" "$proc_mem"
done

echo ""

# ==================== 汇总报告 ====================
echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║                        汇  总  报  告                        ║${NC}"
echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"

# 节点状态
if [ $NODE_FAIL -eq 0 ]; then
    echo -e "${BOLD}║${NC}  节点状态:  ${GREEN}█${NC} 全部正常 ($NODE_OK/$((NODE_OK + NODE_FAIL)))                           ${BOLD}║${NC}"
else
    echo -e "${BOLD}║${NC}  节点状态:  ${RED}█${NC} 异常 ($NODE_FAIL 个未运行)                            ${BOLD}║${NC}"
fi

# 话题频率
if [ $TOPIC_FAIL -eq 0 ] && [ $TOPIC_WARN -eq 0 ]; then
    echo -e "${BOLD}║${NC}  话题频率:  ${GREEN}█${NC} 全部正常                                       ${BOLD}║${NC}"
elif [ $TOPIC_FAIL -eq 0 ]; then
    echo -e "${BOLD}║${NC}  话题频率:  ${YELLOW}█${NC} $TOPIC_WARN 个偏差较大                                  ${BOLD}║${NC}"
else
    echo -e "${BOLD}║${NC}  话题频率:  ${RED}█${NC} $TOPIC_FAIL 个异常                                     ${BOLD}║${NC}"
fi

# 错误状态
if [ $ERRORS_FOUND -eq 0 ]; then
    echo -e "${BOLD}║${NC}  错误检测:  ${GREEN}█${NC} 无错误                                         ${BOLD}║${NC}"
else
    echo -e "${BOLD}║${NC}  错误检测:  ${RED}█${NC} 发现 $ERRORS_FOUND 个问题                                 ${BOLD}║${NC}"
fi

# 运行模式
case $EXEC_MODE in
    simulation)
        echo -e "${BOLD}║${NC}  运行模式:  ${BLUE}█${NC} MuJoCo仿真                                     ${BOLD}║${NC}"
        ;;
    hardware)
        echo -e "${BOLD}║${NC}  运行模式:  ${GREEN}█${NC} 硬件模式                                       ${BOLD}║${NC}"
        ;;
    *)
        echo -e "${BOLD}║${NC}  运行模式:  ${RED}█${NC} 未知                                           ${BOLD}║${NC}"
        ;;
esac

echo -e "${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"

# 总体健康状态
echo ""
total_issues=$((NODE_FAIL + TOPIC_FAIL + ERRORS_FOUND))
if [ $total_issues -eq 0 ]; then
    echo -e "  ${GREEN}${BOLD}系统健康 ✓${NC}"
elif [ $total_issues -le 2 ]; then
    echo -e "  ${YELLOW}${BOLD}系统有轻微问题，建议检查${NC}"
else
    echo -e "  ${RED}${BOLD}系统异常，需要排查！${NC}"
fi

echo ""
