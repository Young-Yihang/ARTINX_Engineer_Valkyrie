#!/usr/bin/env python3
"""
Interactive MoveIt planning tool with URDF visualization.
Uses pybullet for 3D preview + ROS2 MoveGroup action for execution.

Usage:
  # Interactive mode (pybullet GUI with sliders)
  ros2 run arv_v1_moveit move_to_pose.py

  # Preset poses
  ros2 run arv_v1_moveit move_to_pose.py escape
  ros2 run arv_v1_moveit move_to_pose.py start

  # Direct joint angles (rad)
  ros2 run arv_v1_moveit move_to_pose.py --joints 0 2.17 -0.94 -1.33 1.50 -1.68

Prerequisites:
  pip3 install pybullet
"""

import sys
import time
import math
import argparse
import subprocess
import os

# ─── Preset poses (joint space, rad) ───────────────────────────────
PRESETS = {
    "escape": [0.0, 2.1746, -0.937, -1.326, 1.5028, -1.6796],
    "start":  [0.0, 2.6343, -1.0785, 0.0, 0.0, 0.0],
}

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

JOINT_LIMITS = [
    (-1.2217, 1.2217),  # J1
    (0.49, 3.14),       # J2
    (-0.90, 0.70),      # J3
    (-2.975, 3.14),     # J4
    (-1.5708, 1.5708),  # J5
    (-3.14, 3.14),      # J6
]

def find_urdf():
    candidates = [
        "/home/wuhuan/repositories/src/arv_v1_model/urdf/arv_v1.urdf",
        os.path.expanduser("~/repositories/src/arv_v1_model/urdf/arv_v1.urdf"),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    # Try install path
    try:
        import ament_index_python
        pkg = ament_index_python.get_package_share_directory("arv_v1_model")
        p = os.path.join(pkg, "urdf", "arv_v1.urdf")
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return None


def send_joint_target_ros2(joint_values):
    """Send joint target via ros2 topic pub (one-shot)."""
    vals_str = ", ".join(f"{v:.4f}" for v in joint_values)
    msg = f"'{{data: [{vals_str}]}}'"
    cmd = [
        "ros2", "topic", "pub", "--once",
        "/joint_position_target",
        "std_msgs/msg/Float64MultiArray",
        msg
    ]
    print(f"\n[EXEC] Publishing to /joint_position_target: {[f'{v:.3f}' for v in joint_values]}")
    subprocess.run(cmd)


def send_via_trajectory_service(joint_values, name="manual_target"):
    """
    Create a single-point trajectory and execute via /load_trajectory service.
    This gives proper time parameterization and smooth motion.
    """
    import tempfile, yaml

    traj = {
        "meta": {
            "name": name,
            "description": f"Manual target from move_to_pose.py",
            "duration_sec": 3.0,
        },
        "joint_names": JOINT_NAMES,
        "start_position": joint_values,
        "points": [
            {"time": 0.0, "positions": joint_values},
        ]
    }

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, dir='/tmp') as f:
        yaml.dump(traj, f)
        tmp_path = f.name

    print(f"\n[EXEC] Calling /load_trajectory service with target: {[f'{v:.3f}' for v in joint_values]}")
    cmd = [
        "ros2", "service", "call",
        "/load_trajectory",
        "arv_v1_interfaces/srv/LoadTrajectory",
        f"{{name: '{tmp_path}', execute: true}}"
    ]
    subprocess.run(cmd)


def interactive_mode(urdf_path):
    """Launch pybullet GUI with URDF and sliders for joint adjustment."""
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        print("ERROR: pybullet not installed. Run: pip3 install pybullet")
        sys.exit(1)

    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

    num_joints = p.getNumJoints(robot_id)
    arm_joint_indices = []
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        if joint_name in JOINT_NAMES:
            arm_joint_indices.append(i)

    if len(arm_joint_indices) != 6:
        print(f"WARNING: Found {len(arm_joint_indices)} arm joints, expected 6")
        print("Joint mapping may be incorrect")

    sliders = []
    for idx, (ji, name) in enumerate(zip(arm_joint_indices, JOINT_NAMES)):
        lo, hi = JOINT_LIMITS[idx]
        default = (lo + hi) / 2.0
        slider_id = p.addUserDebugParameter(name, lo, hi, default)
        sliders.append(slider_id)

    # Preset buttons (pybullet doesn't have buttons, use debug parameters as toggles)
    preset_params = {}
    for preset_name in PRESETS:
        pid = p.addUserDebugParameter(f"LOAD: {preset_name}", 0, 1, 0)
        preset_params[preset_name] = (pid, 0)

    execute_param = p.addUserDebugParameter(">>> EXECUTE <<<", 0, 1, 0)
    execute_prev = 0

    print("\n" + "=" * 60)
    print("Interactive MoveIt Planning Tool")
    print("=" * 60)
    print("  - Drag sliders to set target joint angles")
    print("  - Click 'LOAD: escape/start' to load presets")
    print("  - Click '>>> EXECUTE <<<' to send to robot")
    print("  - Close window or Ctrl+C to exit")
    print("=" * 60)

    try:
        while p.isConnected(physics_client):
            # Check preset triggers
            for preset_name, (pid, prev_val) in preset_params.items():
                cur_val = p.readUserDebugParameter(pid)
                if cur_val != prev_val:
                    preset_params[preset_name] = (pid, cur_val)
                    values = PRESETS[preset_name]
                    print(f"\n[PRESET] Loading '{preset_name}': {values}")
                    for i, (ji, val) in enumerate(zip(arm_joint_indices, values)):
                        p.resetJointState(robot_id, ji, val)
                    # Can't reset slider values in pybullet, print reminder
                    print("  (Note: sliders not updated, use EXECUTE to send these values)")
                    # Store loaded preset for execute
                    preset_params['_loaded'] = values

            # Read sliders and update visualization
            joint_values = []
            for i, (ji, slider_id) in enumerate(zip(arm_joint_indices, sliders)):
                val = p.readUserDebugParameter(slider_id)
                joint_values.append(val)
                p.resetJointState(robot_id, ji, val)

            # Check execute trigger
            cur_exec = p.readUserDebugParameter(execute_param)
            if cur_exec != execute_prev:
                execute_prev = cur_exec
                # Use loaded preset if available, otherwise use slider values
                target = preset_params.get('_loaded', joint_values)
                if '_loaded' in preset_params:
                    del preset_params['_loaded']
                print(f"\n[TARGET] Joint angles: {[f'{v:.4f}' for v in target]}")
                send_joint_target_ros2(target)

            p.stepSimulation()
            time.sleep(1.0 / 60.0)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        p.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Interactive MoveIt planning with URDF preview")
    parser.add_argument("preset", nargs="?", default=None,
                        help=f"Preset name: {', '.join(PRESETS.keys())}")
    parser.add_argument("--joints", nargs=6, type=float, metavar="J",
                        help="6 joint angles in rad")
    parser.add_argument("--no-gui", action="store_true",
                        help="Skip pybullet GUI, send directly")

    args = parser.parse_args()

    # Direct execution modes
    if args.joints:
        print(f"Joint target: {args.joints}")
        send_joint_target_ros2(args.joints)
        return

    if args.preset:
        if args.preset not in PRESETS:
            print(f"Unknown preset '{args.preset}'. Available: {', '.join(PRESETS.keys())}")
            sys.exit(1)
        values = PRESETS[args.preset]
        print(f"Preset '{args.preset}': {values}")
        send_joint_target_ros2(values)
        return

    # Interactive mode
    urdf_path = find_urdf()
    if urdf_path is None:
        print("ERROR: Cannot find arv_v1.urdf")
        print("Searched: /home/wuhuan/repositories/src/arv_v1_model/urdf/arv_v1.urdf")
        sys.exit(1)

    print(f"URDF: {urdf_path}")

    if args.no_gui:
        print("No-GUI mode: enter joint angles manually")
        while True:
            try:
                line = input("Enter 6 joint angles (space-separated, rad): ")
                vals = [float(x) for x in line.strip().split()]
                if len(vals) != 6:
                    print("Need exactly 6 values")
                    continue
                send_joint_target_ros2(vals)
            except (ValueError, EOFError):
                break
        return

    interactive_mode(urdf_path)


if __name__ == "__main__":
    main()
