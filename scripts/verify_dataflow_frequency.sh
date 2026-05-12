#!/usr/bin/env bash
# Observe ARV dataflow topic frequency without starting or stopping nodes.
#
# Focus:
#   /joint_states                 hardware -> controller feedback
#   /effort_controller/commands   controller -> hardware torque command
#   /hardware_link_diag           hardware TX/RX link counters/rates

set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
DURATION_SEC=60
WINDOW=10000
OUT_DIR=""

JOINT_TOPIC="/joint_states"
EFFORT_TOPIC="/effort_controller/commands"
LINK_TOPIC="/hardware_link_diag"

usage() {
  cat <<USAGE
Usage: $0 [options]

Options:
  --duration SEC     Capture duration. Default: ${DURATION_SEC}
  --window N         ros2 topic hz window. Default: ${WINDOW}
  --out DIR          Output directory. Default: ~/ros2_ws/dataflow_freq_<timestamp>
  -h, --help         Show this help

This script only observes an already running ARV system. It does not launch or
kill ROS nodes.
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --duration)
      DURATION_SEC="$2"; shift 2 ;;
    --window)
      WINDOW="$2"; shift 2 ;;
    --out)
      OUT_DIR="$2"; shift 2 ;;
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

if [ -z "$OUT_DIR" ]; then
  OUT_DIR="$WORKSPACE_DIR/dataflow_freq_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u

SUMMARY="$OUT_DIR/summary.txt"
ENV_LOG="$OUT_DIR/environment.txt"
NODE_LOG="$OUT_DIR/nodes.txt"
TOPIC_LOG="$OUT_DIR/topics.txt"
JOINT_HZ_LOG="$OUT_DIR/joint_states_hz.log"
EFFORT_HZ_LOG="$OUT_DIR/effort_commands_hz.log"
LINK_ECHO_LOG="$OUT_DIR/hardware_link_diag.log"
TOP_LOG="$OUT_DIR/top.log"

MONITOR_PIDS=()

cleanup() {
  local pids=("${MONITOR_PIDS[@]:-}")
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -qx "$topic"
}

capture_hz() {
  local topic="$1"
  local out="$2"
  timeout "$DURATION_SEC" ros2 topic hz "$topic" --window "$WINDOW" >"$out" 2>&1
}

summarize_hz_log() {
  local label="$1"
  local file="$2"
  local expected_hz="$3"

  awk -v label="$label" -v expected="$expected_hz" '
    /average rate:/ {
      rate = $3 + 0
      if (rate_count == 0 || rate < min_rate) min_rate = rate
      if (rate_count == 0 || rate > max_rate) max_rate = rate
      last_rate = rate
      sum_rate += rate
      rate_count++
    }
    /min:/ && /max:/ && /std dev:/ {
      for (i = 1; i <= NF; ++i) {
        if ($i == "max:") {
          v = $(i + 1)
          sub(/s$/, "", v)
          if (interval_count == 0 || v + 0 > max_interval) max_interval = v + 0
        }
        if ($i == "dev:") {
          v = $(i + 1)
          sub(/s$/, "", v)
          if (interval_count == 0 || v + 0 > max_stddev) max_stddev = v + 0
        }
      }
      interval_count++
    }
    END {
      prefix = toupper(label)
      gsub(/[^A-Z0-9]+/, "_", prefix)
      if (rate_count == 0) {
        printf "%s_STATUS=NO_DATA\n", prefix
        printf "%s_SAMPLES=0\n", prefix
        exit
      }
      avg_rate = sum_rate / rate_count
      dev_pct = (avg_rate - expected) * 100.0 / expected
      abs_dev_pct = dev_pct < 0 ? -dev_pct : dev_pct
      max_interval_ms = max_interval * 1000.0
      max_stddev_ms = max_stddev * 1000.0

      status = "OK"
      if (abs_dev_pct > 10.0 || max_interval_ms > 10.0) status = "SUSPECT"
      if (abs_dev_pct > 30.0 || max_interval_ms > 50.0) status = "BAD"

      printf "%s_STATUS=%s\n", prefix, status
      printf "%s_SAMPLES=%d\n", prefix, rate_count
      printf "%s_RATE_LAST_HZ=%.3f\n", prefix, last_rate
      printf "%s_RATE_AVG_HZ=%.3f\n", prefix, avg_rate
      printf "%s_RATE_MIN_HZ=%.3f\n", prefix, min_rate
      printf "%s_RATE_MAX_HZ=%.3f\n", prefix, max_rate
      printf "%s_RATE_DEVIATION_PCT=%.2f\n", prefix, dev_pct
      printf "%s_MAX_INTERVAL_MS=%.3f\n", prefix, max_interval_ms
      printf "%s_MAX_STDDEV_MS=%.3f\n", prefix, max_stddev_ms
    }
  ' "$file"
}

summarize_link_diag() {
  local file="$1"
  awk '
    /^- [-0-9.]+$/ {
      values[++n] = $2 + 0
      if (n == 5) {
        packets++
        tx = values[1]
        rx = values[2]
        crc = values[3]
        ok = values[4]
        if (packets == 1 || tx < tx_min) tx_min = tx
        if (packets == 1 || tx > tx_max) tx_max = tx
        if (packets == 1 || rx < rx_min) rx_min = rx
        if (packets == 1 || rx > rx_max) rx_max = rx
        if (crc > crc_max) crc_max = crc
        if (ok < 0.5) serial_bad++
        n = 0
      }
    }
    END {
      if (packets == 0) {
        print "LINK_DIAG_STATUS=NO_DATA"
        print "LINK_DIAG_SAMPLES=0"
        exit
      }
      status = "OK"
      if (rx_min < 900 || tx_min < 900 || serial_bad > 0 || crc_max > 0) status = "SUSPECT"
      printf "LINK_DIAG_STATUS=%s\n", status
      printf "LINK_DIAG_SAMPLES=%d\n", packets
      printf "LINK_DIAG_TX_MIN_HZ=%.3f\n", tx_min
      printf "LINK_DIAG_TX_MAX_HZ=%.3f\n", tx_max
      printf "LINK_DIAG_RX_MIN_HZ=%.3f\n", rx_min
      printf "LINK_DIAG_RX_MAX_HZ=%.3f\n", rx_max
      printf "LINK_DIAG_CRC_MAX=%.0f\n", crc_max
      printf "LINK_DIAG_SERIAL_BAD_SAMPLES=%d\n", serial_bad
    }
  ' "$file"
}

{
  printf 'date=%s\n' "$(date -Iseconds)"
  printf 'workspace=%s\n' "$WORKSPACE_DIR"
  printf 'duration_sec=%s\n' "$DURATION_SEC"
  printf 'window=%s\n' "$WINDOW"
  printf 'kernel=%s\n' "$(uname -a)"
  printf 'rmw=%s\n' "${RMW_IMPLEMENTATION:-default}"
  printf '\n--- load ---\n'
  uptime
  printf '\n--- cpu governor ---\n'
  cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort | uniq -c || true
} > "$ENV_LOG"

printf '[INFO] Output: %s\n' "$OUT_DIR"
printf '[INFO] Observing dataflow for %ss, window=%s\n' "$DURATION_SEC" "$WINDOW"

ros2 node list >"$NODE_LOG" 2>&1 || true
ros2 topic list -t >"$TOPIC_LOG" 2>&1 || true

missing=0
for topic in "$JOINT_TOPIC" "$EFFORT_TOPIC"; do
  if ! topic_exists "$topic"; then
    printf '[WARN] Topic not found before capture: %s\n' "$topic"
    missing=1
  fi
done

capture_hz "$JOINT_TOPIC" "$JOINT_HZ_LOG" &
MONITOR_PIDS+=("$!")

capture_hz "$EFFORT_TOPIC" "$EFFORT_HZ_LOG" &
MONITOR_PIDS+=("$!")

if topic_exists "$LINK_TOPIC"; then
  timeout "$DURATION_SEC" ros2 topic echo "$LINK_TOPIC" >"$LINK_ECHO_LOG" 2>&1 &
  MONITOR_PIDS+=("$!")
else
  printf '[WARN] Topic not found before capture: %s\n' "$LINK_TOPIC"
  : > "$LINK_ECHO_LOG"
fi

timeout "$DURATION_SEC" top -b -d 1 >"$TOP_LOG" 2>&1 &
MONITOR_PIDS+=("$!")

for pid in "${MONITOR_PIDS[@]}"; do
  wait "$pid" 2>/dev/null || true
done
MONITOR_PIDS=()

{
  printf 'DATAFLOW_VERIFY_STATUS=%s\n' "$([ "$missing" -eq 0 ] && printf OK || printf SUSPECT)"
  printf 'DATAFLOW_VERIFY_OUT_DIR=%s\n' "$OUT_DIR"
  printf 'DATAFLOW_VERIFY_DURATION_SEC=%s\n' "$DURATION_SEC"
  printf 'DATAFLOW_VERIFY_WINDOW=%s\n' "$WINDOW"
  printf '\n--- topic frequency summary ---\n'
  summarize_hz_log "joint_states" "$JOINT_HZ_LOG" 1000
  summarize_hz_log "effort_controller_commands" "$EFFORT_HZ_LOG" 1000
  printf '\n--- hardware link summary ---\n'
  summarize_link_diag "$LINK_ECHO_LOG"
  printf '\n--- latest joint_states hz ---\n'
  tail -20 "$JOINT_HZ_LOG" || true
  printf '\n--- latest effort commands hz ---\n'
  tail -20 "$EFFORT_HZ_LOG" || true
} > "$SUMMARY"

cat "$SUMMARY"
printf '\n[INFO] Logs saved in %s\n' "$OUT_DIR"
