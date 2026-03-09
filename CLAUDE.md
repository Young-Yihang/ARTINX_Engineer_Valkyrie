# ARV_V1 Robotic Arm Project - AI Assistant Guidelines

## Project Overview

**Project**: ARV_V1 6-DOF Robotic Arm Control System
**Purpose**: For Robomaster robotic competition, shall be robust and easily recorded
**Framework**: ROS2 Jazzy + MoveIt2 + MuJoCo
**Architecture**: Multi-mode torque control with real-time constraints
**Workspace**: `~/ros2_ws`

## Critical Constraints

### DO NOT MODIFY - Core Control Files
The following files contain validated control algorithms. Never edit without explicit permission:
- `src/control/torque_controller_node.cpp` - Core torque control implementation
- `src/core/dynamics_computer.cpp` - Dynamics computation algorithms
- `src/core/cascade_pid.cpp` - Cascade PID controller
- `src/core/kalman_filter.cpp` - State estimation filters

#### File Creation Policy
- **Prefer editing existing files over creating new ones**
- **Never create documentation files** (*.md) unless explicitly requested
- **Never create shell scripts** unless explicitly requested
- **Limit single edits to 150 lines** for maintainability

## Code Organization Architecture

### Layered Structure
```
┌─ Application Layer      # Launch files, system initialization
├─ Planning Layer         # MoveIt2 planning, trajectory generation
├─ Control Layer          # Torque control, PID, Kalman filter (PROTECTED)
├─ Interface Layer        # Hardware/MuJoCo interfaces
└─ Core Layer             # Utilities, common definitions
```

### Directory Mapping
```
arv_v1_moveit/src/
├── core/              # Protected control algorithms (DO NOT MODIFY)
│   ├── dynamics_computer.cpp/hpp
│   ├── cascade_pid.cpp/hpp
│   └── kalman_filter.cpp/hpp
├── control/           # Control nodes
│   └── torque_controller_node.cpp
├── interfaces/        # Hardware & simulation interfaces
│   ├── mujoco_interface_node.cpp     # MuJoCo sim + ImGui/ImPlot visualization
│   ├── hardware_interface_node.cpp   # USB-UART serial (Seasky protocol)
│   ├── Crc.cpp/hpp                   # CRC8/CRC16 lookup tables
│   └── serial_protocol.hpp           # Seasky frame builder/parser (header-only)
├── application/       # Application layer nodes
│   ├── trajectory_manager_node.cpp   # Trajectory CRUD + action client
│   ├── mission_executor_node.cpp     # ncurses TUI for mission control
│   ├── cartesian_controller_node.cpp # Cartesian IK control (Pilz LIN/PTP)
│   └── scene_manager_node.cpp        # MoveIt2 collision scene (unused in start script)
└── third_party/       # Vendored libraries
    ├── imgui_backends/ # ImGui GLFW+OpenGL3 backend
    └── implot/         # Real-time plotting
```

### Dependency Rules
- ✅ Upper layers MAY depend on lower layers
- ❌ Lower layers MUST NOT depend on upper layers
- ❌ Circular dependencies are FORBIDDEN
- ❌ Control layer should not depend on interface layer directly

## Project Structure

```
/home/huan/ros2_ws/src/
├── arv_v1_model/           # URDF models and meshes
│   ├── urdf/               # Robot description files (arv_v1.urdf + obstacles/)
│   ├── meshes/             # STL/DAE visual models
│   ├── config/             # Model configuration (joint_names_ARV_V1_MODEL.yaml)
│   └── launch/             # Model launch files
├── arv_v1_moveit/          # Core control package
│   ├── src/                # C++ source (layered subdirectories)
│   │   ├── core/           # Protected algorithms (.cpp + .hpp in same dir)
│   │   ├── control/        # Control nodes
│   │   ├── interfaces/     # Hardware/simulation interfaces (.cpp + .hpp)
│   │   └── application/    # Application layer nodes
│   ├── config/             # YAML configuration files
│   │   ├── controller_params.yaml    # Kalman, cascade PID, safety params
│   │   ├── scene_obstacles.yaml      # MuJoCo + MoveIt2 collision objects
│   │   ├── mission_sequence.yaml     # Task state machine definition
│   │   ├── cartesian_controller_param.yaml
│   │   └── trajectories/             # Runtime-saved trajectory YAMLs
│   ├── third_party/        # ImGui backends, ImPlot
│   ├── launch/             # ROS2 launch files
│   ├── docs/               # Package-specific documentation
│   └── CMakeLists.txt
├── arv_v1_interfaces/      # ROS2 custom service definitions
│   └── srv/                # Service message files
├── docs/                   # Technical documentation
│   ├── GRADUATION_PROJECT/ # Thesis-related documentation
│   ├── SYSTEM_ARCHITECTURE.md
│   ├── TECHNICAL_IMPLEMENTATION.md
│   └── VISION_SYSTEM.md
├── scripts/                # Utility scripts (check_system.sh)
├── start_mujoco_system.sh  # Main startup script (interactive menu)
├── stop_all_nodes.sh       # Cleanup script
└── reload_params.sh        # Hot-reload parameters script
```

## Build and Run Commands

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select arv_v1_moveit arv_v1_model
source install/setup.bash
```

### Launch System
```bash
# Recommended: Interactive menu for mode selection
./start_mujoco_system.sh

# Manual launch options:
# Mode 1: Pure simulation
ros2 launch arv_v1_moveit mujoco_demo.launch.py
ros2 run arv_v1_moveit torque_controller_node
ros2 run arv_v1_moveit mujoco_interface_node
ros2 run arv_v1_moveit trajectory_manager_node
ros2 run arv_v1_moveit cartesian_controller_node
ros2 run arv_v1_moveit mission_executor_node  # TUI interface

# Mode 2: Hardware + Digital twin
ros2 run arv_v1_moveit hardware_interface_node --ros-args -p serial_port:=/dev/ttyACM0
ros2 run arv_v1_moveit mujoco_interface_node --ros-args -p visualization_only:=true
ros2 run arv_v1_moveit trajectory_manager_node
ros2 run arv_v1_moveit cartesian_controller_node
ros2 run arv_v1_moveit mission_executor_node
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
┌──────────────────────────────────────────────────────────────────────┐
│  Application Layer                                                   │
│  ┌──────────────────┐  ┌───────────────────┐  ┌──────────────────┐  │
│  │ mission_executor │─→│ trajectory_manager│  │cartesian_control │  │
│  │ (ncurses TUI)    │  │ (save/load/exec)  │  │(Pilz LIN/PTP)   │  │
│  └──┬───────────┬───┘  └────────┬──────────┘  └────────┬─────────┘  │
│     │/gripper   │/load_traj     │/follow_joint_traj    │/move_group │
└─────│───────────│───────────────│───────────────────────│────────────┘
      ↓           ↓               ↓                       ↓
┌──────────────────────────────────────────────────────────────────────┐
│  Control Layer                                                       │
│  MoveIt2 → torque_controller → [mujoco_interface | hardware_interface]
│            ↑____________________________________________________|    │
│                              /joint_states                           │
│  torque_controller also hosts: /gripper_control service              │
└──────────────────────────────────────────────────────────────────────┘
```

### ROS2 Nodes
| Node | Directory | Description |
|------|-----------|-------------|
| `torque_controller_node` | `control/` | Core torque control with cascade PID/Kalman + gripper service |
| `mujoco_interface_node` | `interfaces/` | MuJoCo sim + ImGui/ImPlot visualization |
| `hardware_interface_node` | `interfaces/` | USB-UART serial (Seasky protocol, 921600 baud) |
| `trajectory_manager_node` | `application/` | Trajectory save/load/execute via YAML |
| `cartesian_controller_node` | `application/` | Cartesian IK (Pilz planner, LIN/PTP fallback) |
| `mission_executor_node` | `application/` | ncurses TUI for mission orchestration |

### Key Topics
- `/joint_states` - Current robot state (position, velocity, effort), 7 joints
- `/effort_controller/commands` - Torque/force commands (Float64MultiArray, 7 elements: 6 arm torques + 1 gripper force)
- `/ARM_controller/follow_joint_trajectory` - Trajectory action interface
- `/ARM_controller/joint_trajectory` - Forward trajectory for recording (used by trajectory_manager)
- `/cartesian_target_pose` - Cartesian goal input (PoseStamped, async)
- `/cartesian_controller/current_pose` - Current end-effector pose @ 30Hz
- `/robot_state_cmd` - Hardware MCU state notifications (UInt8: 0x01=RESET, 0x02=START, 0x03=NEXT)

### Service Interfaces (arv_v1_interfaces)
**Trajectory Manager:**
- `/list_trajectories` - List saved trajectories (names + descriptions)
- `/load_trajectory` - Load and optionally execute a saved trajectory
- `/save_trajectory` - Save trajectory from ROS message to YAML
- `/save_last_trajectory` - Save the last captured trajectory

**Torque Controller:**
- `/gripper_control` - Set gripper force in N (note: srv field named `torque` but gripper is prismatic, unit is N not Nm)

**Cartesian Controller:**
- `/move_to_cartesian_rpy` - Move to absolute Cartesian pose (x,y,z,r,p,y)
- `/stop_cartesian_motion` - Emergency stop of Cartesian motion

**Unused/Placeholder:**
- `/execute_action`, `/get_task_state` - Defined in srv/ but not bound to any node

### Control Modes
1. **Hold Mode**: Maintains current position with gravity compensation
2. **Execute Mode**: Follows trajectory with full dynamics feedforward

## Development Guidelines

### Comment Standards

#### File Header
Use single-line `///` Doxygen format (max 2 lines). Multi-line `/** */` headers are **prohibited** for new files.
```cpp
/// @file filename.cpp
/// @brief One-line English description.
```

#### Section Separators
Use `// --- Section Name ---` for logical sections within a file.
DO NOT change existing separator styles (e.g. `---` → `=====`) during unrelated changes.

#### Language
- Doxygen tags (`@file`, `@brief`): **English**
- Bracketed markers (`[FIX]`, `[SAFETY]`, `[TODO]`, `[OK]`, `[ERROR]`): **English**
- Inline comments: English preferred; Chinese OK for design rationale / derivation
- DO NOT translate or rewrite existing comments during unrelated changes

#### Density — Less is More
- **DO NOT** comment self-explanatory code (variable init, standard API calls, parameter declarations)
- **DO NOT** use numbered step comments (`// 1. xxx`, `// 2. xxx`) — use one summary at function head
- Algorithm functions: one formula/intent comment at top, NOT per-line narration
- Comment **only**: non-obvious logic, safety rationale, magic number derivation, concurrency contracts, external protocol specs
- Prefer shorter inline comment over multi-line block: `// 限幅防IK跳变` not 4-line explanation

### Concurrency Rules

#### Global Lock Ordering (torque_controller_node)
All code paths MUST acquire locks in this order to prevent deadlocks:
```
action_mutex_ → state_mutex_ → filter_mutex_
```
**DO NOT reorder lock acquisition in any code path.** If the documented ordering comment is present, it is authoritative.

#### Documentation Requirement
Every `std::mutex` and `std::atomic` MUST have an inline comment stating:
1. What data it protects
2. Which threads access it
```cpp
std::mutex state_mutex_;  // 保护 q_actual_, q_dot_*, state_received_ (controlLoop + jointStateCallback)
std::atomic<bool> is_executing_;  // action执行标志 (controlLoop读, handleAccepted写)
```

### Magic Numbers
- Joint counts MUST use named constants: `kArmJoints = 6`, `kAllJoints = 7` (arm + gripper)
- DO NOT replace named constants with literal numbers (e.g. `kArmJoints` → `6`)
- Timing thresholds MUST include units in variable name or trailing comment:
  `constexpr auto BYTE_TIMEOUT = std::chrono::milliseconds(200);  // 单字节超时`

### Naming Conventions (C++)
| Element | Convention | Example |
|---------|------------|---------|
| Class/Struct | `PascalCase` | `TorqueController`, `JointState` |
| Function/Method | `camelCase` | `computeDynamics()`, `getJointPosition()` |
| Variable | `snake_case` | `joint_positions`, `target_torque` |
| Constant/Macro | `UPPER_CASE` | `MAX_TORQUE`, `CONTROL_RATE` |
| Member variable | `snake_case_` | `node_handle_`, `publisher_` |
| ROS2 topics | `snake_case` | `/joint_states`, `/effort_commands` |
| ROS2 parameters | `snake_case` | `use_cascade_pid`, `kalman.Q_vel` |

### Interface Contracts (DO NOT BREAK)

Changes below require **explicit approval** and hardware team coordination:

#### ROS2 Topic Dimensions
| Topic | Type | Size | Content |
|-------|------|------|---------|
| `/effort_controller/commands` | Float64MultiArray | **7 elements** | [J1-J6 torques (Nm), gripper force (N)] |
| `/joint_states` | JointState | **7 joints** | 6 arm + 1 gripper (`joint_gripper1`) |

#### Seasky Serial Protocol CmdIDs
| CmdID | Direction | Freq | Payload | Description |
|-------|-----------|------|---------|-------------|
| `0x0001` | RX (MCU→PC) | 200Hz | 7×(float+float+uint32) = 84B | Joint feedback |
| `0x0002` | TX (PC→MCU) | 200Hz | 6×float = 24B | Arm torques |
| `0x0004` | TX (PC→MCU) | 50Hz | 1×uint8 | Gripper action (GRIP/RELEASE/STOP) |
| `0x0005` | RX (MCU→PC) | On-demand | 3×uint8 | Task command (cmd+param+seq) |

DO NOT change CmdID values, payload sizes, or joint counts without MCU firmware update.

### Error Handling
```cpp
// ✅ Correct: Use try-catch with ROS2 logging
try {
    auto result = computeDynamics(joint_state);
} catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Dynamics computation failed: %s", e.what());
    return fallbackBehavior();  // Always have degradation strategy
}

// ❌ Forbidden: Silent failures
auto result = computeDynamics(joint_state);  // No error handling

// ❌ Forbidden: Throwing without logging
throw std::runtime_error("Something failed");  // No context
```

### Logging Standards
```cpp
// ✅ Correct: Use ROS2 logging macros
RCLCPP_INFO(get_logger(), "Controller initialized");
RCLCPP_DEBUG(get_logger(), "Joint %d: pos=%.3f vel=%.3f", idx, pos, vel);
RCLCPP_WARN(get_logger(), "Approaching torque limit: %.2f Nm", torque);
RCLCPP_ERROR(get_logger(), "Failed to read joint state");

// ✅ Correct: Throttled logging for high-frequency loops (200Hz)
RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "Control loop running at %.1f Hz", rate);

// ❌ Forbidden: Console output
std::cout << "debug message" << std::endl;
printf("Position: %f\n", pos);
```

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

**Branch**: feature/pick_ores
**Completed Features**:
- ✅ Dual-mode architecture (simulation/hardware)
- ✅ Kalman filtering
- ✅ Cascade PID control
- ✅ USB serial communication (Seasky protocol)
- ✅ Digital twin visualization (MuJoCo visualization_only mode)
- ✅ Parameter hot reload (`reload_params.sh`)
- ✅ Trajectory management system (save/load/execute via YAML)
- ✅ Mission executor TUI (ncurses, state machine + trajectory CRUD)
- ✅ Cartesian controller (Pilz LIN/PTP, service + topic dual path)
- ✅ MuJoCo ImGui/ImPlot real-time telemetry overlay

**In Progress**:
- 🔧 Visual servoing integration
- 🔧 Pick & place automation (mission_sequence states defined, trajectories pending)
- 🔧 TUI Cartesian jogging (UI present, backend placeholder)

## Hardware Configuration

### Motors
- J1-3: J8009 high-torque motors (revolute, effort limit 40 Nm)
- J4: GM6020 gimbal motor (revolute, effort limit 1.2 Nm)
- J5: J4310 motor (revolute, effort limit 7 Nm)
- J6: M2006 motor (revolute, effort limit 1 Nm)
- Gripper: Motor-driven prismatic pair (0~40mm stroke, mimic joint)
  - Motor peak torque 7 Nm / 0.05m lever arm / 2 jaws = 70 N per jaw
  - URDF effort limit: 7 (unit: N, prismatic joint)
  - Hardware uses discrete 3-state control: GRIP / RELEASE / STOP (via Seasky CmdID 0x0004 @ 50Hz)
  - Simulation uses continuous force: ctrl[-5, 5] N (MuJoCo actuator)

### Communication
- Protocol: USB-UART (921600 baud)
- Frequency: 200Hz arm torques (TX), 50Hz gripper commands (TX), 200Hz joint feedback (RX)
- Format: Seasky protocol - CRC8 header + CRC16 whole frame
- Packet types: 0x0001 (joint feedback RX), 0x0002 (arm torque TX), 0x0004 (gripper TX), 0x0005 (state cmd RX)

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

## Prohibited Actions

1. **DO NOT modify protected core files** without explicit permission
2. **DO NOT allocate memory** in the 200Hz control loop (use pre-allocated buffers)
3. **DO NOT hardcode** topic names, parameters, or magic numbers
4. **DO NOT skip simulation** - always validate in MuJoCo before hardware deployment
5. **DO NOT use blocking calls** in real-time control paths
6. **DO NOT create circular dependencies** between packages or modules
7. **DO NOT commit untested code** to the main branch
8. **DO NOT ignore compiler warnings** - treat `-Wall -Wextra` warnings as errors
9. **DO NOT remove safety mechanisms** (try-catch with fallback, NaN/Inf checks, timeout protection, emergency stop paths) — degrading safety requires explicit approval
10. **DO NOT remove existing features** (control modes, gripper control, hardware protocol support) without explicit approval — scope reduction is not refactoring
11. **DO NOT change comment style in unrelated changes** (deleting Doxygen headers, replacing separators, translating comments) — cosmetic churn obscures real diffs

## Development Checklist

Before submitting code, verify:

- [ ] Code is placed in the correct layer/directory
- [ ] No modifications to protected core control files
- [ ] Follows naming conventions (see table above)
- [ ] Proper error handling with fallback strategies
- [ ] Uses ROS2 logging macros (no `std::cout`/`printf`)
- [ ] Parameters configured via YAML, no hardcoded values
- [ ] Tested in simulation (MuJoCo) first
- [ ] Tested with digital twin before hardware
- [ ] Compiles without warnings (`colcon build`)
- [ ] No runtime memory allocation in control loops

## Key Dependencies (beyond ROS2 standard)

| Library | Purpose | Used By |
|---------|---------|---------|
| MuJoCo (`$MUJOCO_PATH`) | Physics simulation engine | mujoco_interface_node |
| KDL / orocos_kdl | Kinematics & dynamics (M, C, G matrices) | dynamics_computer |
| Eigen3 | Matrix math | core libraries |
| ImGui + ImPlot | Real-time telemetry overlay in MuJoCo window | mujoco_interface_node |
| GLFW + OpenGL | MuJoCo rendering window | mujoco_interface_node |
| yaml-cpp | YAML config/trajectory parsing | trajectory_manager, mission_executor, mujoco_interface |
| serial_driver | USB-UART communication | hardware_interface_node |
| ncurses | Terminal UI | mission_executor_node |
| Pilz Industrial Motion Planner | Cartesian LIN/PTP planning | cartesian_controller_node |

---

**Last Updated**: 2026-03-02
**Maintainer**: Young-Yihang
**Version**: 2.4