#!/bin/bash
# ARV_V1 系统健康检查 v3.0 (Game UI版)
# 功能: 节点状态、话题频率、错误检测、链路诊断

# ==================== 0. 基础设置 ====================
# 捕捉 Ctrl+C 恢复光标
trap 'tput cnorm; exit' INT TERM

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# 变量初始化
SAMPLE_TIME=3
NODE_OK=0; NODE_FAIL=0
TOPIC_OK=0; TOPIC_WARN=0; TOPIC_FAIL=0
ERRORS_FOUND=0
declare -a REPORT_BUFFER  # 用于存储详细日志，最后显示

# ==================== 1. UI 渲染函数 ====================

# 隐藏/显示光标
cursor_hide() { tput civis; }
cursor_show() { tput cnorm; }

show_mascot() {
    clear
    echo -e "${PURPLE}"
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
    echo -e "${NC}"
    echo -e "${CYAN}  :: ARV_V1 SYSTEM DIAGNOSTICS ::${NC}"
    echo "  ──────────────────────────────────────────"
    echo "" # 留一行空行作为状态显示区
    echo "" # 留一行空行作为进度条区
}

# 核心魔法：在固定位置刷新文字
update_loading() {
    local text="$1"
    local percent="$2"
    local bar_len=40
    local filled=$((percent * bar_len / 100))
    local empty=$((bar_len - filled))
    
    # 构造进度条
    local bar_str=$(printf "%${filled}s" | tr ' ' '█')
    local empty_str=$(printf "%${empty}s" | tr ' ' '░')
    
    # 移动光标：向上2行 (\033[2A) -> 清除行 (\033[K) -> 打印
    # 这里的逻辑是：假设当前光标在进度条下方，我们需要往回跳
    
    echo -e "\033[2A\033[K  ${BOLD}STATUS:${NC} $text"
    echo -e "\033[K  ${BLUE}[${bar_str}${empty_str}]${NC} ${percent}%"
}

# 将日志写入缓冲区，不直接打印
log_ok()   { REPORT_BUFFER+=("${GREEN}[✓]${NC} $1"); }
log_warn() { REPORT_BUFFER+=("${YELLOW}[!]${NC} $1"); }
log_fail() { REPORT_BUFFER+=("${RED}[✗]${NC} $1"); }
log_info() { REPORT_BUFFER+=("${BLUE}[i]${NC} $1"); }
log_section() { REPORT_BUFFER+=(""); REPORT_BUFFER+=("${CYAN}>> $1${NC}"); }

# ==================== 2. 检测逻辑 (静默版) ====================

# 配置
declare -A EXPECTED_NODES=(
    ["torque_controller"]="力矩控制器"
    ["move_group"]="MoveIt规划器"
    ["robot_state_publisher"]="TF发布器"
    ["trajectory_manager"]="轨迹管理器"
    ["mission_executor"]="任务执行器"
)

# 阶段1: 节点检查
check_nodes_silently() {
    log_section "1. 节点状态检查"
    local nodes_list=$(ros2 node list 2>/dev/null)
    
    for node in "${!EXPECTED_NODES[@]}"; do
        if echo "$nodes_list" | grep -q "$node"; then
            log_ok "${EXPECTED_NODES[$node]}"
            ((NODE_OK++))
        else
            log_fail "${EXPECTED_NODES[$node]} (未运行)"
            ((NODE_FAIL++))
        fi
    done

    # 检查运行模式
    if echo "$nodes_list" | grep -q "mujoco_interface"; then
        log_ok "执行层: MuJoCo仿真"
        EXEC_MODE="simulation"
    elif echo "$nodes_list" | grep -q "hardware_interface"; then
        log_ok "执行层: 硬件接口"
        EXEC_MODE="hardware"
    else
        log_fail "执行层: 未检测到接口"
        EXEC_MODE="none"
    fi
}

# 阶段2: 话题频率 (这个最耗时)
check_hz_silently() {
    log_section "2. 话题与通信质量"
    local topic=$1
    local expected=$2
    local name=$3

    # 实际执行 ros2 topic hz
    local hz_output=$(timeout $((SAMPLE_TIME + 1)) ros2 topic hz "$topic" --window 50 2>/dev/null | grep "average rate:" | tail -1)
    local hz=$(echo "$hz_output" | awk '{print $3}')

    if [ -z "$hz" ]; then
        log_fail "$name: 无数据 (期望 ${expected}Hz)"
        ((TOPIC_FAIL++))
        return
    fi

    # 计算偏差
    local hz_int=${hz%.*}
    local deviation=$(( (hz_int - expected) * 100 / expected ))
    local abs_dev=${deviation#-}

    if [ "$abs_dev" -le 10 ]; then
        log_ok "$name: ${hz} Hz (偏差 ${deviation}%)"
        ((TOPIC_OK++))
    elif [ "$abs_dev" -le 30 ]; then
        log_warn "$name: ${hz} Hz (偏差 ${deviation}%)"
        ((TOPIC_WARN++))
    else
        log_fail "$name: ${hz} Hz (严重偏差 ${deviation}%)"
        ((TOPIC_FAIL++))
    fi
}

# 阶段3: 错误日志扫描
check_errors_silently() {
    log_section "3. 系统日志扫描"
    # 检查 Daemon
    if ros2 daemon status 2>&1 | grep -qi "error\|failed"; then
        log_warn "ROS2 Daemon 状态异常"
        ((ERRORS_FOUND++))
    fi

    # 检查 Joint States NaN
    local joint_data=$(timeout 1 ros2 topic echo /joint_states --once 2>/dev/null)
    if echo "$joint_data" | grep -q "nan\|inf"; then
        log_fail "关节数据包含 NaN/Inf"
        ((ERRORS_FOUND++))
    else
        log_ok "关节数据完整性校验通过"
    fi
}

# 阶段4: 系统资源
check_resources_silently() {
    log_section "4. 资源占用"
    local cpu=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
    local mem=$(free | awk 'NR==2{printf "%.1f", $3*100/$2}')
    
    log_info "CPU负载: ${cpu}%"
    log_info "内存占用: ${mem}%"
}

# ==================== 主程序流 ====================

cursor_hide
show_mascot

# 初始化进度条位置
# 此时光标在进度条下方，我们需要保持它在这里

# --- 步骤 1: 节点检查 ---
update_loading "正在连接 ROS2 上下文..." 10
sleep 0.5
update_loading "正在扫描活动节点..." 20
check_nodes_silently
sleep 0.5

# --- 步骤 2: 话题频率 (耗时) ---
update_loading "采样关节数据 (/joint_states)..." 40
check_hz_silently "/joint_states" 200 "关节状态"

update_loading "采样控制指令 (/effort_controller/commands)..." 60
check_hz_silently "/effort_controller/commands" 200 "力矩指令"

# --- 步骤 3: 错误与资源 ---
update_loading "分析系统日志与错误..." 80
check_errors_silently
sleep 0.5

update_loading "读取硬件资源统计..." 90
check_resources_silently
sleep 0.5

# --- 完成 ---
update_loading "诊断完成！生成报告中..." 100
sleep 0.8

# ==================== 最终报告渲染 ====================
clear
show_mascot # 重新画头
echo -e "\033[2A\033[K" # 擦除上面 show_mascot留下的空行
echo -e "\033[K" 

# 打印详细列表
echo -e "  ${BOLD}📋 详细诊断报告:${NC}"
for line in "${REPORT_BUFFER[@]}"; do
    echo -e "  $line"
done

echo ""
echo -e "  ${BOLD}════════════════ 结 论 ════════════════${NC}"

# 总体判断逻辑
total_issues=$((NODE_FAIL + TOPIC_FAIL + ERRORS_FOUND))

if [ $total_issues -eq 0 ]; then
    if [ $TOPIC_WARN -gt 0 ]; then
         echo -e "  ${YELLOW}${BOLD}[系统就绪] 但存在性能警告${NC}"
         echo -e "  虽然所有组件都在运行，但部分话题频率不稳定。"
    else
         echo -e "  ${GREEN}${BOLD}[系统完美] 所有指标正常${NC}"
         echo -e "  ARV 系统已准备好执行任务。"
    fi
else
    echo -e "  ${RED}${BOLD}[系统异常] 检测到 $total_issues 个关键问题${NC}"
    echo -e "  请检查上方的 [✗] 标记项目。"
fi

echo ""
cursor_show