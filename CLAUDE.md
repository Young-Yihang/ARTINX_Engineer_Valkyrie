# ARV_V1 Robotic Arm Project - AI Assistant Guidelines

## Project Overview

**Project**: ARV_V1 6-DOF Robotic Arm Control System
**Framework**: ROS2 Jazzy + MoveIt2 + MuJoCo
**Architecture**: Multi-mode torque control with real-time constraints
**Workspace**: `~/ros2_ws`

## Critical Constraints

### DO NOT MODIFY - Core Control Files
The following files contain validated control algorithms. Never edit without explicit permission:
- `torque_controller_node.cpp` - Core torque control implementation
- `dynamics_computer.cpp` - Dynamics computation algorithms
- `cascade_pid.cpp` - Cascade PID controller
- `kalman_filter.cpp` - State estimation filters

### File Creation Policy
- **Prefer editing existing files over creating new ones**
- **Never create documentation files** (*.md) unless explicitly requested
- **Never create shell scripts** unless explicitly requested
- **Limit single edits to 150 lines** for maintainability

## Project Structure

```
/home/huan/ros2_ws/src/
├── ARV_V1_MODEL/           # URDF models and meshes
│   ├── urdf/               # Robot description files
│   └── meshes/             # STL/DAE visual models
├── ARV_V1_MOVEIT/          # Core control package
│   ├── src/                # C++ source (see constraints above)
│   ├── include/            # Header files
│   ├── config/             # YAML configuration files
│   ├── launch/             # ROS2 launch files
│   └── CMakeLists.txt
├── docs/                   # Technical documentation
│   ├── TODO_KDL.md         # System architecture & roadmap
│   ├── VISION_GRASP.md    # Vision system design
│   ├── VISION_LEARNING.md # Learning pathway
│   └── ARCHITECTURE_RT.md # Real-time architecture
└── scripts/
    ├── start_mujoco_system.sh  # Main startup script
    ├── stop_all_nodes.sh        # Cleanup script
    └── reload_params.sh         # Hot reload parameters
```

## Build and Run Commands

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select ARV_V1_MOVEIT ARV_V1_MODEL
source install/setup.bash
```

### Launch System
```bash
# Interactive menu for mode selection
./start_mujoco_system.sh

# Manual launch options:
# Mode 1: Pure simulation
ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py
ros2 run ARV_V1_MOVEIT torque_controller_node
ros2 run ARV_V1_MOVEIT mujoco_interface_node

# Mode 2: Hardware + Digital twin
ros2 run ARV_V1_MOVEIT hardware_interface_node --ros-args -p serial_port:=/dev/ttyACM0
ros2 run ARV_V1_MOVEIT mujoco_interface_node --ros-args -p visualization_only:=true
```

### Debugging Commands
```bash
# Check running nodes
ros2 node list | grep -E "(torque|mujoco|hardware)"

# Monitor topics
ros2 topic echo /joint_states
ros2 topic echo /effort_controller/commands

# Parameter adjustment
ros2 param set /torque_controller_action_server use_cascade_pid true
ros2 param set /torque_controller_action_server kalman.Q_vel 1e-5
```

## System Architecture

### Node Topology (200Hz Control Loop)
```
MoveIt2 → torque_controller → [mujoco_interface | hardware_interface] → joint_states
         ↑__________________________________________________|
```

### Key Topics
- `/joint_states` - Current robot state (position, velocity, effort)
- `/effort_controller/commands` - Computed torque commands
- `/ARM_controller/follow_joint_trajectory` - Trajectory action interface

### Control Modes
1. **Hold Mode**: Maintains current position with gravity compensation
2. **Execute Mode**: Follows trajectory with full dynamics feedforward

## Development Guidelines

### Communication Style
- Be concise - this is a CLI environment
- Use GitHub-flavored markdown for formatting
- Output text directly (no echo commands for communication)
- Include file paths with line numbers: `file_path:line_number`

### Testing Workflow
1. **Always test in simulation first** (mujoco_interface_node)
2. **Verify with digital twin** (visualization_only mode)
3. **Deploy to hardware** only after simulation validation

### Parameter Tuning
- PD gains: Higher for proximal joints (J1-J3), lower for distal (J4-J6)
- Kalman filter: Adjust Q_vel (1e-7 to 1e-5) for noise/response balance
- Cascade PID: Enable with `use_cascade_pid` parameter

## Current Development Status

**Branch**: feature/ros2_components
**Completed Features**:
- ✅ Dual-mode architecture (simulation/hardware)
- ✅ Kalman filtering
- ✅ Cascade PID control
- ✅ USB serial communication
- ✅ Digital twin visualization
- ✅ Parameter hot reload

**In Progress**:
- 🔧 Visual servoing integration
- 🔧 Dynamic obstacle avoidance

## Hardware Configuration

### Motors
- J1-3: J8009 high-torque motors
- J4: GM6020 gimbal motor
- J5: J4310 motor
- J6: M2006 motor

### Communication
- Protocol: USB-UART (921600 baud)
- Frequency: 200Hz bidirectional
- Format: Seasky protocol with CRC16

## Common Issues and Solutions

| Issue | Solution |
|-------|----------|
| Joint drift on startup | First joint_states auto-saved as target |
| Torque oscillation | Increase Kalman Q_vel parameter |
| Slow Joint1 motion | Collision detection disabled in URDF |
| MuJoCo black screen | OpenGL context threading issue - fixed |

## Performance Requirements

- Control loop: < 5ms latency @ 200Hz
- CPU usage: < 65% total (on Intel i5 8th gen)
- Memory: No runtime allocation (pre-allocated pools)
- Real-time: Core 4 isolated for control (RT-PREEMPT kernel)

---

**Last Updated**: 2026-01-16
**Maintainer**: Young-Yihang
**Version**: 2.0