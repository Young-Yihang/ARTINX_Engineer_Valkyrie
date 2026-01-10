# CAN通信模块移除规划文档

**创建日期**: 2026-01-10  
**目标**: 彻底移除项目中所有与CAN通信相关的代码和配置

---

## 📋 一、架构分析总结

### 1.1 当前系统架构
根据 `ARCHITECTURE_RT.md` 和 `ARCHITECTURE_COMPARISON.md`，系统支持三种运行模式：

| 模式 | 描述 | 涉及节点 |
|------|------|----------|
| 模式1 | 纯仿真 | MoveIt + TorqueController + MuJoCo |
| 模式2 | 串口真机 + 数字孪生 | MoveIt + TorqueController + HardwareInterface(串口) + MuJoCo |
| 模式3 | **CAN真机 + 数字孪生** | MoveIt + TorqueController + **CanInterface** + MuJoCo |

### 1.2 CAN模块在架构中的位置

```
┌─────────────────────────────────────────────────────┐
│                MoveIt2 Planning                     │
└────────────┬────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────┐
│           Torque Controller Node                    │
│         (输出力矩命令到 /effort_controller/commands)│
└────────────┬────────────────────────────────────────┘
             │
             ├──────────────┬──────────────┬──────────────┐
             ▼              ▼              ▼              ▼
    ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
    │   MuJoCo    │ │  Hardware   │ │**CAN**      │ │   (其他)    │
    │  Interface  │ │  Interface  │ │**Interface**│ │             │
    │   (仿真)    │ │   (串口)    │ │**[删除]**   │ │             │
    └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘
         │                │               │
         ▼                ▼               ▼
    物理仿真          串口通信        **SocketCAN通信**
    /joint_states   /joint_states   **/joint_states**
```

**关键发现**:
- CAN模块是三种硬件接口之一，与MuJoCo和Hardware Interface并行
- CAN接口节点专门用于通过SocketCAN与MIT电机进行实时通信（1kHz）
- 移除CAN模块**不影响**仿真和串口模式的正常运行

---

## 🎯 二、CAN相关内容清单

### 2.1 核心代码文件 (需要删除)

| 文件路径 | 行数 | 功能描述 | 操作 |
|---------|------|---------|------|
| `ARV_V1_MOVEIT/src/can_interface_node.cpp` | 272行 | SocketCAN接口节点主程序 | **删除** |
| `ARV_V1_MOVEIT/src/mit_protocol.hpp` | 200行 | MIT电机CAN协议编解码库 | **删除** |

### 2.2 构建配置文件 (需要修改)

#### `ARV_V1_MOVEIT/CMakeLists.txt`
需要移除的内容（第236-253行）:
```cmake
# SocketCAN 接口节点 (Sim2Real - CAN通信)
add_executable(can_interface_node
  src/can_interface_node.cpp
)

target_include_directories(can_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(can_interface_node
  rclcpp
  sensor_msgs
  std_msgs
)
```

需要移除的安装目标（第264行）:
```cmake
install(TARGETS
  # ...其他节点...
  can_interface_node      # ← 删除此行
  DESTINATION lib/${PROJECT_NAME}
)
```

### 2.3 启动脚本 (需要修改)

#### `start_mujoco_system.sh`
需要移除的内容:

**1. 菜单选项（第45行）:**
```bash
echo -e "${GREEN}║${NC}  [3] SocketCAN真机 + 数字孪生         ${GREEN}║${NC}"
```

**2. CAN设备探测函数（第80-109行）:**
```bash
detect_can_device() {
    log_info "自动探测CAN设备..."
    local can_devices=()
    for i in 0 1 2 3; do
        if ip link show "can$i" &>/dev/null; then
            can_devices+=("can$i")
        fi
    done
    # ...后续代码...
}
```

**3. CAN模式启动函数（第199-205行）:**
```bash
start_can_mode() {
    detect_can_device || exit 1
    log_info "启动CAN真机模式 (接口: $DETECTED_CAN_DEVICE)..."
    start_node "MoveIt+RViz" "ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py" 0
    start_node "TorqueController" "ros2 run ARV_V1_MOVEIT torque_controller_node" 0
    sleep 3
    start_node "CANInterface" "ros2 run ARV_V1_MOVEIT can_interface_node --ros-args -p can_interface:=$DETECTED_CAN_DEVICE" 0
    start_node "MuJoCo(孪生)" "ros2 run ARV_V1_MOVEIT mujoco_interface_node --ros-args -p visualization_only:=true" 0
}
```

**4. 主菜单分支（第216行）:**
```bash
case $choice in
    1) start_sim_mode; break ;;
    2) start_serial_mode; break ;;
    3) start_can_mode; break ;;  # ← 删除此行
    0) log_info "退出"; exit 0 ;;
    *) log_warning "无效选择，请重试" ;;
esac
```

**5. 文件头部注释（第9行）:**
```bash
#   3. SocketCAN真机 + 数字孪生
```

#### `stop_all_nodes.sh`
需要移除的内容（第28行）:
```bash
nodes=(
    "mujoco_interface_node"
    "hardware_interface_node"
    "can_interface_node"          # ← 删除此行 (CAN模式)
    "torque_controller_node"
    # ...其他节点...
)
```

### 2.4 文档文件 (需要修改)

需要清理CAN相关的内容:

| 文件 | 需要修改的内容 |
|------|---------------|
| `docs/ARCHITECTURE_RT.md` | 删除CAN相关的CPU核心分配、优先级配置、启动命令 |
| `docs/ARCHITECTURE_COMPARISON.md` | 可保留（仅作为历史对比参考）|
| `docs/TODO_KDL.md` | 删除模式3相关描述、CAN接口完成标记、CAN帧设计说明 |
| `CLAUDE.md` | 删除can_interface相关条目 |

### 2.5 环境依赖

**无需额外清理的依赖**:
- `linux/can.h` - 系统头文件，不影响其他模块
- SocketCAN驱动 - 系统级，不影响ROS2工作空间

---

## 📝 三、详细移除步骤

### 阶段一：备份与准备 ⏱️ 5分钟

```bash
# 1. 进入工作空间
cd /home/huan/ros2_ws/src

# 2. 创建备份分支（可选）
git checkout -b backup-before-can-removal
git add .
git commit -m "备份: 移除CAN模块前的完整状态"

# 3. 切回主分支
git checkout main  # 或你的工作分支
```

### 阶段二：删除核心代码文件 ⏱️ 2分钟

```bash
# 删除CAN接口节点和协议头文件
rm ARV_V1_MOVEIT/src/can_interface_node.cpp
rm ARV_V1_MOVEIT/src/mit_protocol.hpp
```

### 阶段三：修改CMakeLists.txt ⏱️ 5分钟

在 `ARV_V1_MOVEIT/CMakeLists.txt` 中：

**1. 删除第236-253行** (整个can_interface_node构建配置)

**2. 在第264行附近，删除安装目标中的can_interface_node**

修改前:
```cmake
install(TARGETS
  dynamics_computer
  dynamics_solver_node
  torque_controller_node
  mujoco_interface_node
  hardware_interface_node
  can_interface_node      # ← 删除此行
  DESTINATION lib/${PROJECT_NAME}
)
```

修改后:
```cmake
install(TARGETS
  dynamics_computer
  dynamics_solver_node
  torque_controller_node
  mujoco_interface_node
  hardware_interface_node
  DESTINATION lib/${PROJECT_NAME}
)
```

### 阶段四：修改启动脚本 ⏱️ 10分钟

#### 4.1 修改 `start_mujoco_system.sh`

**删除内容**:
1. 第9行：注释中的模式3描述
2. 第45行：菜单中的选项3
3. 第80-109行：整个 `detect_can_device()` 函数
4. 第199-205行：整个 `start_can_mode()` 函数
5. 第216行：`case`语句中的 `3) start_can_mode; break ;;`

**菜单显示修改**（第43-47行）:
```bash
# 修改前
echo -e "${GREEN}║${NC}  [1] 纯仿真模式                      ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  [2] 串口真机 + 数字孪生            ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  [3] SocketCAN真机 + 数字孪生       ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  [0] 退出                          ${GREEN}║${NC}"

# 修改后
echo -e "${GREEN}║${NC}  [1] 纯仿真模式                      ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  [2] 串口真机 + 数字孪生            ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  [0] 退出                          ${GREEN}║${NC}"
```

#### 4.2 修改 `stop_all_nodes.sh`

删除第28行的 `"can_interface_node"` 条目。

### 阶段五：清理文档 ⏱️ 15分钟

#### 5.1 修改 `docs/ARCHITECTURE_RT.md`

**删除内容**:
- CPU核心分配表中的Core 5相关内容
- Component分组表中的组2 (can_interface)
- 优先级表中的can_interface条目
- 启动脚本中的can_interface_node命令
- 故障隔离表中的"CAN通信断开"条目

**修改前** (第17行):
```markdown
| 5 | 超硬实时 | 是 | can_interface (1kHz) |
```
删除或改为:
```markdown
| 5 | 预留 | 是 | 预留扩展 |
```

#### 5.2 修改 `docs/TODO_KDL.md`

**删除内容**:
- 第18-20行：模式3的运行命令
- 第34-35行：架构图中的can_node部分
- 第213行：选择列表中的CAN真机选项
- 第251行：SocketCAN接口完成标记
- 第355-357行：CAN帧设计说明

#### 5.3 修改 `CLAUDE.md`

删除包含 `can_interface` 的行（第54行附近）。

#### 5.4 保留 `docs/ARCHITECTURE_COMPARISON.md` (可选)

该文档是历史对比分析，可以保留作为设计参考。如果需要完全清理，可添加文档头说明：
```markdown
> **历史文档**: 此文档包含已废弃的CAN通信方案，仅作为架构演进参考。
```

### 阶段六：重新编译 ⏱️ 5分钟

```bash
cd /home/huan/ros2_ws

# 清理旧编译文件
rm -rf build/ARV_V1_MOVEIT install/ARV_V1_MOVEIT

# 重新编译
colcon build --packages-select ARV_V1_MOVEIT

# 检查是否有编译错误
echo $?  # 应该返回 0
```

### 阶段七：功能验证 ⏱️ 10分钟

```bash
# 1. 测试纯仿真模式
cd /home/huan/ros2_ws/src
./start_mujoco_system.sh
# 选择 [1]，确认系统正常启动

# 2. 测试串口模式（如果有硬件）
# 选择 [2]，确认串口接口正常工作

# 3. 验证不存在CAN节点
ros2 node list | grep can_interface
# 应该没有输出

# 4. 验证可执行文件已删除
ls install/ARV_V1_MOVEIT/lib/ARV_V1_MOVEIT/ | grep can
# 应该没有输出
```

---

## ✅ 四、验证清单

完成移除后，确认以下内容:

- [ ] `can_interface_node.cpp` 已删除
- [ ] `mit_protocol.hpp` 已删除
- [ ] `CMakeLists.txt` 不再包含can_interface_node构建配置
- [ ] `start_mujoco_system.sh` 菜单中无模式3选项
- [ ] `stop_all_nodes.sh` 不再尝试停止can_interface_node
- [ ] `docs/ARCHITECTURE_RT.md` 不再提及CAN相关配置
- [ ] `docs/TODO_KDL.md` 不再描述模式3
- [ ] `CLAUDE.md` 不再列出can_interface
- [ ] 编译通过无错误
- [ ] 模式1（纯仿真）正常工作
- [ ] 模式2（串口）正常工作（如有硬件）
- [ ] `ros2 node list` 无can_interface_node
- [ ] 安装目录无can_interface_node可执行文件

---

## 🔄 五、回滚方案

如果移除后出现问题，可以快速恢复：

```bash
# 方案1: 使用备份分支
cd /home/huan/ros2_ws/src
git checkout backup-before-can-removal
colcon build --packages-select ARV_V1_MOVEIT

# 方案2: 从git历史恢复特定文件
git checkout HEAD~1 -- ARV_V1_MOVEIT/src/can_interface_node.cpp
git checkout HEAD~1 -- ARV_V1_MOVEIT/src/mit_protocol.hpp
git checkout HEAD~1 -- ARV_V1_MOVEIT/CMakeLists.txt
# 然后重新编译
```

---

## 📊 六、影响分析

### 6.1 不受影响的功能 ✅
- ✅ MoveIt2 运动规划
- ✅ TorqueController 力矩控制
- ✅ MuJoCo 仿真
- ✅ 串口硬件接口
- ✅ 数字孪生可视化

### 6.2 被移除的功能 ❌
- ❌ SocketCAN实时通信能力
- ❌ MIT电机协议支持
- ❌ CAN总线硬件接口
- ❌ 1kHz高频CAN控制回路

### 6.3 系统架构变化

**移除前**（3种模式）:
```
模式1: MoveIt → Torque → MuJoCo
模式2: MoveIt → Torque → Serial → 真机
模式3: MoveIt → Torque → CAN → MIT电机
```

**移除后**（2种模式）:
```
模式1: MoveIt → Torque → MuJoCo
模式2: MoveIt → Torque → Serial → 真机
```

---

## 📌 七、注意事项

1. **备份重要性**: 建议在Git中创建备份分支或tag
2. **文档一致性**: 确保所有文档都移除CAN相关描述，避免混淆
3. **依赖检查**: 确认没有其他节点依赖`mit_protocol.hpp`
4. **测试覆盖**: 移除后需完整测试剩余两种模式
5. **团队沟通**: 如果是团队项目，通知所有成员此架构变更

---

## 📅 八、预计时间成本

| 阶段 | 时间 | 说明 |
|------|------|------|
| 备份与准备 | 5分钟 | Git备份 |
| 删除代码文件 | 2分钟 | 删除2个文件 |
| 修改构建配置 | 5分钟 | CMakeLists.txt |
| 修改启动脚本 | 10分钟 | 2个shell脚本 |
| 清理文档 | 15分钟 | 4个markdown文件 |
| 重新编译 | 5分钟 | colcon build |
| 功能验证 | 10分钟 | 测试两种模式 |
| **总计** | **52分钟** | 约1小时 |

---

## 🚀 九、执行建议

**推荐执行顺序**:
1. ✅ 先完成备份（安全第一）
2. ✅ 删除代码文件（核心清理）
3. ✅ 修改构建配置（确保编译通过）
4. ✅ 验证编译（早发现问题）
5. ✅ 修改脚本和文档（完善清理）
6. ✅ 最终测试（确保功能正常）

**最佳实践**:
- 一次只修改一个文件，及时commit
- 每个阶段后运行编译检查
- 保持文档与代码同步更新

---

**文档版本**: v1.0  
**最后更新**: 2026-01-10  
**维护者**: ARV V1 Team
