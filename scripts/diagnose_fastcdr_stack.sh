#!/usr/bin/env bash
# Collect evidence for eProsima Fast-CDR/FastDDS dependency failures.

set -u

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
OUT="${1:-$WORKSPACE_DIR/fastcdr_diagnosis.txt}"
PKGS=(
  ros-jazzy-fastcdr
  ros-jazzy-fastrtps
  ros-jazzy-fastrtps-cmake-module
  ros-jazzy-rmw-fastrtps-cpp
  ros-jazzy-rmw-fastrtps-shared-cpp
  ros-jazzy-rmw-dds-common
  ros-jazzy-rosidl-dynamic-typesupport
  ros-jazzy-rosidl-dynamic-typesupport-fastrtps
  ros-jazzy-rosidl-typesupport-fastrtps-c
  ros-jazzy-rosidl-typesupport-fastrtps-cpp
  ros-jazzy-rosidl-typesupport-c
  ros-jazzy-rosidl-typesupport-cpp
  ros-jazzy-rosidl-generator-c
  ros-jazzy-rosidl-generator-cpp
  ros-jazzy-rosidl-runtime-c
  ros-jazzy-rosidl-runtime-cpp
  ros-jazzy-service-msgs
  ros-jazzy-geometry-msgs
  ros-jazzy-trajectory-msgs
)

section() {
  printf '\n===== %s =====\n' "$1"
}

run() {
  printf '$ %s\n' "$*"
  "$@" 2>&1 || true
}

pkg_version() {
  dpkg-query -W -f='${Version}' "$1" 2>/dev/null || true
}

pkg_stamp() {
  local version="$1"
  if [[ "$version" =~ noble\.([0-9]{8})\.([0-9]{6}) ]]; then
    printf '%s.%s' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}"
  elif [[ "$version" =~ noble\.([0-9]{8}) ]]; then
    printf '%s' "${BASH_REMATCH[1]}"
  else
    printf 'no-build-stamp'
  fi
}

dependency_verdict() {
  local core_pkgs=(
    ros-jazzy-fastcdr
    ros-jazzy-fastrtps
    ros-jazzy-rmw-fastrtps-cpp
    ros-jazzy-rmw-fastrtps-shared-cpp
    ros-jazzy-rmw-dds-common
    ros-jazzy-rosidl-dynamic-typesupport-fastrtps
    ros-jazzy-rosidl-typesupport-fastrtps-c
    ros-jazzy-rosidl-typesupport-fastrtps-cpp
  )
  local pkg version stamp missing=0
  local -A stamps=()

  for pkg in "${core_pkgs[@]}"; do
    version="$(pkg_version "$pkg")"
    if [ -z "$version" ]; then
      printf 'FASTCDR_STACK_PACKAGE\t%s\tMISSING\tmissing\n' "$pkg"
      missing=1
      continue
    fi
    stamp="$(pkg_stamp "$version")"
    stamps["$stamp"]=1
    printf 'FASTCDR_STACK_PACKAGE\t%s\t%s\t%s\n' "$pkg" "$version" "$stamp"
  done

  if [ "$missing" -ne 0 ]; then
    printf 'FASTCDR_STACK_STATUS=BAD\n'
    printf 'FASTCDR_STACK_SUSPECT=missing-fastdds-or-rosidl-package\n'
    return
  fi

  if [ "${#stamps[@]}" -gt 1 ]; then
    printf 'FASTCDR_STACK_STATUS=SUSPECT\n'
    printf 'FASTCDR_STACK_SUSPECT=mixed-build-stamps-in-fastcdr-fastrtps-rmw-rosidl-stack\n'
    printf 'FASTCDR_STACK_REASON=apt-upgrade-may-have-left-FastDDS-stack-built-from-different-ROS-repository-snapshots\n'
  else
    printf 'FASTCDR_STACK_STATUS=OK\n'
    printf 'FASTCDR_STACK_SUSPECT=none-from-package-build-stamps\n'
  fi
}

{
  section "host"
  run date -Iseconds
  run uname -a
  run lsb_release -a

  section "environment"
  printf 'WORKSPACE_DIR=%s\n' "$WORKSPACE_DIR"
  printf 'AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH:-}"
  printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-}"
  printf 'FASTRTPS_DEFAULT_PROFILES_FILE=%s\n' "${FASTRTPS_DEFAULT_PROFILES_FILE:-}"

  section "dpkg versions"
  run dpkg-query -W -f='${Package}\t${Version}\t${Status}\n' "${PKGS[@]}"

  section "dependency verdict"
  dependency_verdict
  run apt-mark showhold

  section "apt policy"
  run apt-cache policy "${PKGS[@]}"

  section "apt depends"
  run apt-cache depends \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-rmw-fastrtps-shared-cpp \
    ros-jazzy-rosidl-typesupport-fastrtps-cpp \
    ros-jazzy-rosidl-typesupport-fastrtps-c \
    ros-jazzy-fastrtps \
    ros-jazzy-fastcdr

  section "library owners"
  for lib in \
    /opt/ros/jazzy/lib/libfastcdr.so.2 \
    /opt/ros/jazzy/lib/libfastrtps.so.2.14 \
    /opt/ros/jazzy/lib/librmw_fastrtps_cpp.so \
    /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_cpp.so \
    /opt/ros/jazzy/lib/librosidl_dynamic_typesupport_fastrtps.so; do
    [ -e "$lib" ] && run dpkg -S "$lib"
  done

  section "generated interface libraries"
  run find "$WORKSPACE_DIR/install/arv_v1_interfaces" -type f '(' -name '*.so' -o -name '*.so.*' ')' -print

  section "ldd generated fastrtps typesupport"
  for lib in \
    "$WORKSPACE_DIR/install/arv_v1_interfaces/lib/libarv_v1_interfaces__rosidl_typesupport_fastrtps_cpp.so" \
    "$WORKSPACE_DIR/install/arv_v1_interfaces/lib/libarv_v1_interfaces__rosidl_typesupport_fastrtps_c.so"; do
    [ -e "$lib" ] && run ldd "$lib"
  done

  section "ldd rmw_fastrtps"
  [ -e /opt/ros/jazzy/lib/librmw_fastrtps_cpp.so ] && run ldd /opt/ros/jazzy/lib/librmw_fastrtps_cpp.so

  section "recent apt history"
  run bash -lc "zgrep -hE 'Start-Date|Commandline|Upgrade:|Install:|Remove:' /var/log/apt/history.log* | tail -240"

  section "minimal ROS interface checks"
  run bash -lc "cd '$WORKSPACE_DIR' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 pkg prefix arv_v1_interfaces && ros2 interface show arv_v1_interfaces/srv/GripperControl"
  run bash -lc "cd '$WORKSPACE_DIR' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && timeout 5 ros2 run arv_v1_moveit torque_controller_node --ros-args --params-file src/arv_v1_moveit/config/controller_params_sim.yaml"
} > "$OUT"

printf 'diagnosis written to %s\n' "$OUT"
