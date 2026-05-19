#!/usr/bin/env bash
# Minimal real-hardware latency check for hardware_interface_node.
#
# This script does not modify code or switch RMW. It launches only the hardware
# node with zero torque enabled, then collects timing logs and topic diagnostics.

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
DURATION_SEC=30
SERIAL_PORT="auto"
SEND_HZ=1000
GRIPPER_HZ=50
KILL_EXISTING=0
OBSERVE_EXISTING=0
OUT_DIR=""

usage() {
  cat <<USAGE
Usage: $0 [options]

Options:
  --duration SEC        Capture duration. Default: ${DURATION_SEC}
  --serial-port DEV     Serial device, or auto. Default: auto
  --send-hz HZ          hardware_interface_node send_rate_hz. Default: ${SEND_HZ}
  --gripper-hz HZ       gripper_rate_hz. Default: ${GRIPPER_HZ}
  --out DIR             Output directory. Default: ~/ros2_ws/hw_latency_<timestamp>
  --observe-existing    Do not start/stop nodes; observe an already running hardware node
  --kill-existing       Kill existing hardware_interface_node before starting
  -h, --help            Show this help

The node is launched with force_zero_torque:=true, so it sends zero torque only.
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --duration)
      DURATION_SEC="$2"; shift 2 ;;
    --serial-port)
      SERIAL_PORT="$2"; shift 2 ;;
    --send-hz)
      SEND_HZ="$2"; shift 2 ;;
    --gripper-hz)
      GRIPPER_HZ="$2"; shift 2 ;;
    --out)
      OUT_DIR="$2"; shift 2 ;;
    --kill-existing)
      KILL_EXISTING=1; shift ;;
    --observe-existing)
      OBSERVE_EXISTING=1; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      printf '[ERROR] Unknown option: %s\n' "$1" >&2
      usage
      exit 2 ;;
  esac
done

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  printf '[ERROR] /opt/ros/jazzy/setup.bash not found\n' >&2
  exit 1
fi

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  printf '[ERROR] workspace setup not found: %s/install/setup.bash\n' "$WORKSPACE_DIR" >&2
  exit 1
fi

if [ "$OBSERVE_EXISTING" -eq 0 ] && [ "$SERIAL_PORT" = "auto" ]; then
  SERIAL_PORT="$(find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) 2>/dev/null | sort | head -n 1 || true)"
fi

if [ "$OBSERVE_EXISTING" -eq 0 ] && { [ -z "$SERIAL_PORT" ] || [ ! -e "$SERIAL_PORT" ]; }; then
  printf '[ERROR] No serial device found. Use --serial-port /dev/ttyACM0\n' >&2
  exit 1
fi

if [ "$OBSERVE_EXISTING" -eq 0 ] && { [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; }; then
  printf '[ERROR] Serial device is not readable/writable: %s\n' "$SERIAL_PORT" >&2
  printf '        Try: sudo usermod -aG dialout $USER  # then relogin\n' >&2
  printf '        Or temporary: sudo chmod 666 %s\n' "$SERIAL_PORT" >&2
  exit 1
fi

if [ -z "$OUT_DIR" ]; then
  OUT_DIR="$WORKSPACE_DIR/hw_latency_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

if ros2 node list 2>/dev/null | grep -qx '/hardware_interface_node'; then
  if [ "$KILL_EXISTING" -eq 1 ]; then
    pkill -f 'hardware_interface_node' 2>/dev/null || true
    sleep 1
  elif [ "$OBSERVE_EXISTING" -eq 1 ]; then
    :
  else
    printf '[ERROR] /hardware_interface_node is already running. Stop it or pass --kill-existing.\n' >&2
    exit 1
  fi
elif [ "$OBSERVE_EXISTING" -eq 1 ]; then
  printf '[ERROR] --observe-existing requested but /hardware_interface_node is not running.\n' >&2
  exit 1
fi

SUMMARY="$OUT_DIR/summary.txt"
NODE_LOG="$OUT_DIR/hardware_node.log"
ROS_LOG_COPY="$OUT_DIR/ros_hardware_timing.log"
HZ_LOG="$OUT_DIR/joint_states_hz.log"
DIAG_LOG="$OUT_DIR/hardware_link_diag.log"
TOP_LOG="$OUT_DIR/top.log"
ENV_LOG="$OUT_DIR/environment.txt"

cleanup() {
  local pids=("${MONITOR_PIDS[@]:-}")
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  if [ "$OBSERVE_EXISTING" -eq 0 ] && [ -n "${NODE_PID:-}" ]; then
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

{
  printf 'date=%s\n' "$(date -Iseconds)"
  printf 'workspace=%s\n' "$WORKSPACE_DIR"
  printf 'serial_port=%s\n' "$SERIAL_PORT"
  printf 'observe_existing=%s\n' "$OBSERVE_EXISTING"
  printf 'duration_sec=%s\n' "$DURATION_SEC"
  printf 'send_hz=%s\n' "$SEND_HZ"
  printf 'gripper_hz=%s\n' "$GRIPPER_HZ"
  printf 'kernel=%s\n' "$(uname -a)"
  printf 'rmw=%s\n' "${RMW_IMPLEMENTATION:-default}"
  printf '\n--- serial device ---\n'
  [ -n "$SERIAL_PORT" ] && [ -e "$SERIAL_PORT" ] && ls -l "$SERIAL_PORT" || true
  printf '\n--- cpu governor ---\n'
  cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c || true
  printf '\n--- load ---\n'
  uptime
} > "$ENV_LOG"

printf '[INFO] Output: %s\n' "$OUT_DIR"

if [ "$OBSERVE_EXISTING" -eq 0 ]; then
  printf '[INFO] Starting hardware node on %s for %ss\n' "$SERIAL_PORT" "$DURATION_SEC"
  stdbuf -oL -eL timeout "$DURATION_SEC" \
    ros2 run arv_v1_moveit hardware_interface_node --ros-args \
      -p serial_port:="$SERIAL_PORT" \
      -p send_rate_hz:="$SEND_HZ" \
      -p gripper_rate_hz:="$GRIPPER_HZ" \
      -p force_zero_torque:=true \
      -p link_diag_enabled:=true \
      -p link_diag_period_ms:=1000 \
    >"$NODE_LOG" 2>&1 &
  NODE_PID=$!
  sleep 2
else
  printf '[INFO] Observing existing /hardware_interface_node for %ss\n' "$DURATION_SEC"
  touch "$NODE_LOG"
fi

MONITOR_PIDS=()
MONITOR_DURATION=$((OBSERVE_EXISTING == 0 && DURATION_SEC > 3 ? DURATION_SEC - 2 : DURATION_SEC))

timeout "$MONITOR_DURATION" ros2 topic hz /joint_states >"$HZ_LOG" 2>&1 &
MONITOR_PIDS+=("$!")

timeout "$MONITOR_DURATION" ros2 topic echo /hardware_link_diag >"$DIAG_LOG" 2>&1 &
MONITOR_PIDS+=("$!")

HW_PID="$(pgrep -n -f 'hardware_interface_node' || true)"
if [ -n "$HW_PID" ]; then
  timeout "$MONITOR_DURATION" top -b -d 1 -p "$HW_PID" >"$TOP_LOG" 2>&1 &
  MONITOR_PIDS+=("$!")
fi

if [ "$OBSERVE_EXISTING" -eq 0 ]; then
  wait "$NODE_PID" 2>/dev/null || true
  NODE_PID=""
else
  sleep "$MONITOR_DURATION"
fi

for pid in "${MONITOR_PIDS[@]}"; do
  wait "$pid" 2>/dev/null || true
done

if [ "$OBSERVE_EXISTING" -eq 1 ]; then
  find "$HOME/.ros/log" -type f -name '*.log' -mmin -30 -print0 2>/dev/null \
    | xargs -0 grep -hE '\[SEND/TIMING|hardware_interface_node|\[LINK\]|\[HEALTH\]' \
    >"$ROS_LOG_COPY" 2>/dev/null || true
  if [ -s "$ROS_LOG_COPY" ]; then
    cp "$ROS_LOG_COPY" "$NODE_LOG"
  fi
fi

max_us_for_field() {
  local field="$1"
  local value
  value="$(
    {
      grep -oE "${field}=[0-9]+us" "$NODE_LOG" 2>/dev/null \
        | sed -E 's/.*=([0-9]+)us/\1/' || true
      grep -oE "${field}: avg=[0-9]+us max=[0-9]+us" "$NODE_LOG" 2>/dev/null \
        | sed -E 's/.*max=([0-9]+)us/\1/' || true
    } | sort -nr | head -n 1
  )"
  printf '%s\n' "${value:-0}"
}

max_percentile_field() {
  local field="$1"
  local value
  value="$(grep -oE "${field}=[0-9]+us" "$NODE_LOG" 2>/dev/null \
    | sed -E 's/.*=([0-9]+)us/\1/' \
    | sort -nr \
    | head -n 1 || true)"
  printf '%s\n' "${value:-0}"
}

overrun_count="$(grep -c '\[SEND/TIMING\] OVERRUN' "$NODE_LOG" || true)"
max_overrun_us="$(grep -oE 'OVERRUN total=[0-9]+us' "$NODE_LOG" 2>/dev/null \
  | sed -E 's/.*total=([0-9]+)us/\1/' \
  | sort -nr \
  | head -n 1 || true)"
max_overrun_us="${max_overrun_us:-0}"
max_1k_us="$(grep '\[SEND/TIMING/1k\]' "$NODE_LOG" 2>/dev/null \
  | grep -oE 'total: min=[0-9]+us avg=[0-9]+us max=[0-9]+us' \
  | sed -E 's/.*max=([0-9]+)us/\1/' \
  | sort -nr \
  | head -n 1 || true)"
max_1k_us="${max_1k_us:-0}"
max_gap_us="$(max_us_for_field 'gap')"
max_param_us="$(max_us_for_field 'param')"
max_cache_us="$(max_us_for_field 'cache')"
max_build_torque_us="$(max_us_for_field 'build_torque')"
max_send_torque_us="$(max_us_for_field 'send_torque')"
max_build_gripper_us="$(max_us_for_field 'build_gripper')"
max_send_gripper_us="$(max_us_for_field 'send_gripper')"
max_build_status_us="$(max_us_for_field 'build_status')"
max_send_status_us="$(max_us_for_field 'send_status')"
max_serial_lock_us="$(max_us_for_field 'serial_lock')"
total_p50_us="$(max_percentile_field 'total_p50')"
total_p90_us="$(max_percentile_field 'total_p90')"
total_p99_us="$(max_percentile_field 'total_p99')"
total_p999_us="$(max_percentile_field 'total_p999')"
gap_p99_us="$(max_percentile_field 'gap_p99')"
gap_p999_us="$(max_percentile_field 'gap_p999')"
send_torque_p99_us="$(max_percentile_field 'send_torque_p99')"
send_torque_p999_us="$(max_percentile_field 'send_torque_p999')"
timing_windows="$(grep -c '\[SEND/TIMING/1k\]' "$NODE_LOG" || true)"
link_lines="$(grep -c '\[LINK\]' "$NODE_LOG" || true)"
health_lines="$(grep -c '\[HEALTH\]' "$NODE_LOG" || true)"

{
  printf 'HW_VERIFY_STATUS=%s\n' "$([ "$overrun_count" -eq 0 ] && printf OK || printf SUSPECT)"
  printf 'HW_VERIFY_OVERRUN_COUNT=%s\n' "$overrun_count"
  printf 'HW_VERIFY_MAX_OVERRUN_US=%s\n' "$max_overrun_us"
  printf 'HW_VERIFY_MAX_1K_WINDOW_US=%s\n' "$max_1k_us"
  printf 'HW_VERIFY_TOTAL_P50_US=%s\n' "$total_p50_us"
  printf 'HW_VERIFY_TOTAL_P90_US=%s\n' "$total_p90_us"
  printf 'HW_VERIFY_TOTAL_P99_US=%s\n' "$total_p99_us"
  printf 'HW_VERIFY_TOTAL_P999_US=%s\n' "$total_p999_us"
  printf 'HW_VERIFY_GAP_P99_US=%s\n' "$gap_p99_us"
  printf 'HW_VERIFY_GAP_P999_US=%s\n' "$gap_p999_us"
  printf 'HW_VERIFY_SEND_TORQUE_P99_US=%s\n' "$send_torque_p99_us"
  printf 'HW_VERIFY_SEND_TORQUE_P999_US=%s\n' "$send_torque_p999_us"
  printf 'HW_VERIFY_MAX_GAP_US=%s\n' "$max_gap_us"
  printf 'HW_VERIFY_MAX_PARAM_US=%s\n' "$max_param_us"
  printf 'HW_VERIFY_MAX_CACHE_US=%s\n' "$max_cache_us"
  printf 'HW_VERIFY_MAX_BUILD_TORQUE_US=%s\n' "$max_build_torque_us"
  printf 'HW_VERIFY_MAX_SEND_TORQUE_US=%s\n' "$max_send_torque_us"
  printf 'HW_VERIFY_MAX_BUILD_GRIPPER_US=%s\n' "$max_build_gripper_us"
  printf 'HW_VERIFY_MAX_SEND_GRIPPER_US=%s\n' "$max_send_gripper_us"
  printf 'HW_VERIFY_MAX_BUILD_STATUS_US=%s\n' "$max_build_status_us"
  printf 'HW_VERIFY_MAX_SEND_STATUS_US=%s\n' "$max_send_status_us"
  printf 'HW_VERIFY_MAX_SERIAL_LOCK_US=%s\n' "$max_serial_lock_us"
  printf 'HW_VERIFY_TIMING_WINDOWS=%s\n' "$timing_windows"
  printf 'HW_VERIFY_LINK_LINES=%s\n' "$link_lines"
  printf 'HW_VERIFY_HEALTH_LINES=%s\n' "$health_lines"
  printf 'HW_VERIFY_SERIAL_PORT=%s\n' "$SERIAL_PORT"
  printf 'HW_VERIFY_OBSERVE_EXISTING=%s\n' "$OBSERVE_EXISTING"
  printf 'HW_VERIFY_OUT_DIR=%s\n' "$OUT_DIR"
  printf '\n--- latest timing lines ---\n'
  grep '\[SEND/TIMING' "$NODE_LOG" | tail -40 || true
  printf '\n--- joint_states hz tail ---\n'
  tail -40 "$HZ_LOG" || true
} > "$SUMMARY"

cat "$SUMMARY"
printf '\n[INFO] Logs saved in %s\n' "$OUT_DIR"
