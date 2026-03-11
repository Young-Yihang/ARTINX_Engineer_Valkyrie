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

# --- 可配置参数 (改频率只改这里) ---
CONTROL_RATE_HZ=1000      # /joint_states 期望频率 (Hz)
EFFORT_RATE_HZ=1000       # /effort_controller/commands 期望频率 (Hz)
JITTER_WARN_MS=0.2        # 抖动警告阈值 (ms), 1kHz周期=1ms
RT_CORE=3                 # RT-PREEMPT 隔离核心编号

NODE_OK=0; NODE_FAIL=0
TOPIC_OK=0; TOPIC_WARN=0; TOPIC_FAIL=0
ERRORS_FOUND=0
declare -a REPORT_BUFFER  # 用于存储详细日志，最后显示

# ==================== 1. UI 渲染函数 ====================

# 隐藏/显示光标
cursor_hide() { tput civis; }
cursor_show() { tput cnorm; }

# 极光渐变: 蓝(50,100,255) → 紫(180,50,255) → 品红(255,50,200)
aurora_rgb() {
    local row=$1 total=$2
    local max=$((total - 1))
    (( max < 1 )) && max=1
    local r g b
    if (( row * 2 < total )); then
        local t=$(( row * 2 ))
        r=$(( 50 + 130 * t / total ))
        g=$(( 100 - 50 * t / total ))
        b=255
    else
        local t=$(( row * 2 - total ))
        r=$(( 180 + 75 * t / total ))
        g=$(( 50 ))
        b=$(( 255 - 55 * t / total ))
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
    local total=${#mlines[@]}

    # 逐行极光浮现
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(aurora_rgb $i $total)
        printf "\033[%d;1H\033[2;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
        sleep 0.012
    done
    sleep 0.05

    # 极光波动 6 帧 (色带滚动)
    for (( frame=0; frame<6; frame++ )); do
        for (( i=0; i<total; i++ )); do
            local shifted=$(( (i + frame * 4) % total ))
            local rgb
            rgb=$(aurora_rgb $shifted $total)
            printf "\033[%d;1H\033[1;38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
        done
        sleep 0.07
    done

    # 定格
    for (( i=0; i<total; i++ )); do
        local rgb
        rgb=$(aurora_rgb $i $total)
        printf "\033[%d;1H\033[38;2;${rgb}m%s\033[0m" $((i + 1)) "${mlines[$i]}"
    done

    echo ""
    echo -e "  \033[38;2;180;100;255m:: ARV_V1 SYSTEM DIAGNOSTICS ::\033[0m"
    echo "  ──────────────────────────────────────────"
    echo ""
}

# 步骤计数器
CHECK_STEP=0
CHECK_TOTAL=6

# 进度: 逐行 spinner
update_loading() {
    local text="$1"
    CHECK_STEP=$((CHECK_STEP + 1))
    local frames=("⠋" "⠙" "⠹" "⠸" "⠼" "⠴" "⠦" "⠧" "⠇" "⠏")
    for i in $(seq 0 5); do
        printf "\r  ${CYAN}${frames[$((i % ${#frames[@]}))]}${NC} ${BOLD}[${CHECK_STEP}/${CHECK_TOTAL}]${NC} $text"
        sleep 0.1
    done
    printf "\r  ${GREEN}✓${NC} ${BOLD}[${CHECK_STEP}/${CHECK_TOTAL}]${NC} $text\n"
}

# 将日志写入缓冲区，不直接打印
log_ok()   { REPORT_BUFFER+=("${GREEN}[✓]${NC} $1"); }
log_warn() { REPORT_BUFFER+=("${YELLOW}[!]${NC} $1"); }
log_fail() { REPORT_BUFFER+=("${RED}[✗]${NC} $1"); }
log_info() { REPORT_BUFFER+=("${BLUE}[i]${NC} $1"); }
log_section() { REPORT_BUFFER+=(""); REPORT_BUFFER+=("${CYAN}>> $1${NC}"); }

# ==================== 2. 检测逻辑 (静默版) ====================

# 配置: 通用节点 (两种模式都需要)
declare -A COMMON_NODES=(
    ["torque_controller"]="力矩控制器"
    ["move_group"]="MoveIt规划器"
    ["robot_state_publisher"]="TF发布器"
    ["trajectory_manager"]="轨迹管理器"
    ["mission_executor"]="任务执行器"
    ["cartesian_controller"]="笛卡尔控制器"
)

# 阶段1: 节点检查 (模式感知)
check_nodes_silently() {
    log_section "1. 节点状态检查"
    local nodes_list=$(ros2 node list 2>/dev/null)

    # 先检测运行模式
    local has_mujoco=false has_hardware=false
    echo "$nodes_list" | grep -q "mujoco_interface" && has_mujoco=true
    echo "$nodes_list" | grep -q "hardware_interface" && has_hardware=true

    if $has_hardware; then
        EXEC_MODE="hardware"
        log_ok "执行层: 硬件接口"
    elif $has_mujoco; then
        EXEC_MODE="simulation"
        log_ok "执行层: MuJoCo仿真"
    else
        EXEC_MODE="none"
        log_fail "执行层: 未检测到接口"
    fi

    # 通用节点
    for node in "${!COMMON_NODES[@]}"; do
        if echo "$nodes_list" | grep -q "$node"; then
            log_ok "${COMMON_NODES[$node]}"
            ((NODE_OK++))
        else
            log_fail "${COMMON_NODES[$node]} (未运行)"
            ((NODE_FAIL++))
        fi
    done

    # 模式专属节点
    if [ "$EXEC_MODE" = "hardware" ]; then
        # 硬件模式: 数字孪生可选
        if $has_mujoco; then
            log_ok "数字孪生: MuJoCo (visualization_only)"
            ((NODE_OK++))
        else
            log_warn "数字孪生: MuJoCo 未启动 (可选)"
        fi
    fi
}

# 阶段2: 话题频率 + 抖动 (log_section 由主流程调用, 不在函数内)
check_hz_silently() {
    local topic=$1
    local expected=$2
    local name=$3

    # 采样: 同时捕获 average rate 和 std dev
    local raw_output=$(timeout $((SAMPLE_TIME + 1)) ros2 topic hz "$topic" --window 50 2>/dev/null | tail -2)
    local hz=$(echo "$raw_output" | grep "average rate:" | awk '{print $3}')
    # std dev 行格式: "    min: 0.004s max: 0.006s std dev: 0.00035s window: 50"
    local std_dev_s=$(echo "$raw_output" | grep "std dev:" | sed 's/.*std dev: \([0-9.]*\)s.*/\1/')

    if [ -z "$hz" ]; then
        log_fail "$name: 无数据 (期望 ${expected}Hz)"
        ((TOPIC_FAIL++))
        return
    fi

    # 频率偏差
    local hz_int=${hz%.*}
    local deviation=$(( (hz_int - expected) * 100 / expected ))
    local abs_dev=${deviation#-}

    # 抖动评估 (std_dev 秒 → 毫秒)
    local jitter_info=""
    if [ -n "$std_dev_s" ]; then
        local std_dev_ms=$(awk "BEGIN{printf \"%.2f\", $std_dev_s * 1000}")
        local jitter_bad=$(awk "BEGIN{print ($std_dev_ms > $JITTER_WARN_MS) ? 1 : 0}")
        if [ "$jitter_bad" -eq 1 ]; then
            jitter_info=", 抖动 ${std_dev_ms}ms ${YELLOW}[高]${NC}"
        else
            jitter_info=", 抖动 ${std_dev_ms}ms"
        fi
    fi

    if [ "$abs_dev" -le 10 ]; then
        log_ok "$name: ${hz} Hz (偏差 ${deviation}%${jitter_info})"
        ((TOPIC_OK++))
    elif [ "$abs_dev" -le 30 ]; then
        log_warn "$name: ${hz} Hz (偏差 ${deviation}%${jitter_info})"
        ((TOPIC_WARN++))
    else
        log_fail "$name: ${hz} Hz (严重偏差 ${deviation}%${jitter_info})"
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
    if echo "$joint_data" | grep -iE "^- nan$|^- inf$|^- -inf$" | grep -q .; then
        log_fail "关节数据包含 NaN/Inf"
        ((ERRORS_FOUND++))
    else
        log_ok "关节数据完整性校验通过"
    fi
}

# 阶段4: 各节点 CPU% / 内存
check_node_perf_silently() {
    log_section "4. 节点性能"
    local ps_output=$(ps -eo pid,comm,%cpu,rss --no-headers 2>/dev/null)
    local node_patterns="torque_control|mujoco_interfa|hardware_inter|move_group|trajectory_man|cartesian_cont|mission_execut"
    local matched=$(echo "$ps_output" | grep -E "$node_patterns")

    if [ -z "$matched" ]; then
        log_warn "未检测到 ROS2 节点进程"
        return
    fi

    local total_cpu=0 total_rss=0
    while IFS= read -r line; do
        local pname=$(echo "$line" | awk '{print $2}')
        local pcpu=$(echo "$line" | awk '{print $3}')
        local prss_kb=$(echo "$line" | awk '{print $4}')
        local prss_mb=$(awk "BEGIN{printf \"%.1f\", $prss_kb / 1024}")
        log_info "${pname}: CPU ${pcpu}%  RSS ${prss_mb} MB"
        total_cpu=$(awk "BEGIN{printf \"%.1f\", $total_cpu + $pcpu}")
        total_rss=$(awk "BEGIN{printf \"%.1f\", $total_rss + $prss_kb / 1024}")
    done <<< "$matched"

    # 汇总 (对比 <65% 约束)
    local cpu_bad=$(awk "BEGIN{print ($total_cpu > 65) ? 1 : 0}")
    if [ "$cpu_bad" -eq 1 ]; then
        log_warn "节点合计: CPU ${total_cpu}% (超过65%阈值)  RSS ${total_rss} MB"
    else
        log_ok "节点合计: CPU ${total_cpu}%  RSS ${total_rss} MB"
    fi
}

# 阶段5: 系统资源 + RT核心
check_resources_silently() {
    log_section "5. 系统资源"
    local cpu=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
    local mem=$(free | awk 'NR==2{printf "%.1f", $3*100/$2}')

    log_info "全局 CPU: ${cpu}%"
    log_info "内存占用: ${mem}%"

    # RT 隔离核心检测
    if command -v mpstat &>/dev/null; then
        local idle=$(mpstat -P "$RT_CORE" 1 1 2>/dev/null | tail -1 | awk '{print $NF}')
        if [ -n "$idle" ]; then
            local rt_use=$(awk "BEGIN{printf \"%.1f\", 100 - $idle}")
            local rt_bad=$(awk "BEGIN{print ($rt_use > 80) ? 1 : 0}")
            if [ "$rt_bad" -eq 1 ]; then
                log_warn "RT核心(#${RT_CORE}): ${rt_use}% (超过80%)"
            else
                log_ok "RT核心(#${RT_CORE}): ${rt_use}%"
            fi
        fi
    fi
}

# ==================== 主程序流 ====================

cursor_hide
show_mascot

# --- 步骤 1: 节点检查 ---
update_loading "扫描活动节点"
check_nodes_silently

# --- 步骤 2: 话题频率 + 抖动 (耗时) ---
log_section "2. 话题与通信质量"
update_loading "采样 /joint_states"
check_hz_silently "/joint_states" $CONTROL_RATE_HZ "关节状态"

update_loading "采样 /effort_controller/commands"
check_hz_silently "/effort_controller/commands" $EFFORT_RATE_HZ "力矩指令"

# --- 步骤 3: 错误扫描 ---
update_loading "分析系统日志"
check_errors_silently

# --- 步骤 4: 节点性能 ---
update_loading "采集节点 CPU/内存"
check_node_perf_silently

# --- 步骤 5: 系统资源 + RT核心 ---
update_loading "检测系统资源与RT核心"
check_resources_silently

# ==================== 最终报告渲染 ====================
echo ""
echo -e "  ${BOLD}详细诊断报告:${NC}"
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