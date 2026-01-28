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
- `torque_controller_node.cpp` - Core torque control implementation
- `dynamics_computer.cpp` - Dynamics computation algorithms
- `cascade_pid.cpp` - Cascade PID controller
- `kalman_filter.cpp` - State estimation filters

#### File Creation Policy
- **Prefer editing existing files over creating new ones**
- **Never create documentation files** (*.md) unless explicitly requested
- **Never create shell scripts** unless explicitly requested
- **Limit single edits to 150 lines** for maintainability
- **Maximum file size**: 500 lines per file (hard limit)

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
src/
├── core/           # Protected control algorithms (DO NOT MODIFY)
│   ├── torque_controller_node.cpp
│   ├── dynamics_computer.cpp
│   ├── cascade_pid.cpp
│   └── kalman_filter.cpp
├── interfaces/     # Hardware & simulation interfaces
│   ├── mujoco_interface_node.cpp
│   └── hardware_interface_node.cpp
├── planning/       # Motion planning utilities
├── services/       # ROS2 service implementations
└── utils/          # Helper functions, type definitions
```

### Dependency Rules
- ✅ Upper layers MAY depend on lower layers
- ❌ Lower layers MUST NOT depend on upper layers
- ❌ Circular dependencies are FORBIDDEN
- ❌ Control layer should not depend on interface layer directly

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

## Prohibited Actions

1. **DO NOT modify protected core files** without explicit permission
2. **DO NOT allocate memory** in the 200Hz control loop (use pre-allocated buffers)
3. **DO NOT hardcode** topic names, parameters, or magic numbers
4. **DO NOT skip simulation** - always validate in MuJoCo before hardware deployment
5. **DO NOT use blocking calls** in real-time control paths
6. **DO NOT create circular dependencies** between packages or modules
7. **DO NOT commit untested code** to the main branch
8. **DO NOT ignore compiler warnings** - treat `-Wall -Wextra` warnings as errors

## Development Checklist

Before submitting code, verify:

- [ ] Code is placed in the correct layer/directory
- [ ] No modifications to protected core control files
- [ ] File size < 500 lines
- [ ] Follows naming conventions (see table above)
- [ ] Proper error handling with fallback strategies
- [ ] Uses ROS2 logging macros (no `std::cout`/`printf`)
- [ ] Parameters configured via YAML, no hardcoded values
- [ ] Tested in simulation (MuJoCo) first
- [ ] Tested with digital twin before hardware
- [ ] Compiles without warnings (`colcon build`)
- [ ] No runtime memory allocation in control loops

## Code Templates

### New ROS2 Node Template
```cpp
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // Declare parameters
        this->declare_parameter("param_name", default_value);

        // Create publishers/subscribers
        publisher_ = this->create_publisher<MsgType>("topic", 10);
        subscription_ = this->create_subscription<MsgType>(
            "topic", 10,
            std::bind(&MyNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Node initialized");
    }

private:
    void callback(const MsgType::SharedPtr msg) {
        // Process message
    }

    rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    rclcpp::Subscription<MsgType>::SharedPtr subscription_;
};
```

### Adding New Parameters
```yaml
# In config/*.yaml
my_node:
  ros__parameters:
    control_rate: 200.0
    gains:
      kp: [100.0, 80.0, 60.0, 40.0, 30.0, 20.0]
      kd: [10.0, 8.0, 6.0, 4.0, 3.0, 2.0]
```

---

**Last Updated**: 2026-01-21
**Maintainer**: Young-Yihang
**Version**: 2.0