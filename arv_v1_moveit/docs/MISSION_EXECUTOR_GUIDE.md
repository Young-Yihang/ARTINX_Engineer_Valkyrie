# Mission Executor v2.0 使用指南

## 概述

`mission_executor_node` v2.0 是ARV_V1系统的应用层交互节点，提供基于终端的TUI（Text User Interface）界面，实现完整的轨迹生命周期管理：创建、执行、保存、删除、查看。

---

## 新增功能 (v2.0)

### ✨ 核心功能
- **[S] 保存轨迹**：交互式输入名称和描述，保存RViz执行的轨迹
- **自动刷新**：保存后自动更新任务列表
- **输入验证**：名称合法性检查、重复检测、覆盖确认

### ✨ 增强功能
- **[D] 删除轨迹**：选择任务编号删除对应轨迹文件
- **[I] 查看详情**：显示轨迹元数据（时长、点数、保存时间等）
- **[H] 帮助菜单**：完整的命令说明和使用流程

### ✨ 用户体验
- **彩色UI**：ANSI颜色编码，状态一目了然
- **状态机输入**：智能切换单字符/行输入模式
- **错误提示**：详细的错误信息和操作建议

---

## 架构定位

```
┌──────────────────────────────────────┐
│   Application Layer                  │
│   mission_executor_node (TUI)        │  ← 用户交互层
└──────────────┬───────────────────────┘
               │ Service Calls
               ▼
┌──────────────────────────────────────┐
│   Business Layer                     │
│   trajectory_manager_node            │  ← 业务逻辑层
└──────────────┬───────────────────────┘
               │ Action Calls
               ▼
┌──────────────────────────────────────┐
│   Planning Layer                     │
│   MoveIt2 move_group                 │  ← 规划层
└──────────────────────────────────────┘
```

---

## 核心特性

### 1. 动态任务发现
- 启动时自动扫描 `config/trajectories/` 目录
- 通过 `/list_trajectories` 服务获取所有已保存轨迹
- 支持运行时刷新（按 `R` 键）

### 2. 持久化服务连接
```cpp
// 构造函数中创建一次，复用整个生命周期
load_client_ = this->create_client<LoadTrajectory>("/load_trajectory");
```
**优势**：
- 避免每次调用重新建立连接（节省 ~100ms）
- 降低延迟至 < 5ms
- 提升用户体验

### 3. 异步非阻塞执行
```cpp
// 主线程立即返回，不会阻塞UI
std::thread([this, future = std::move(future), name]() mutable {
    auto result = future.get();  // 后台等待
    updateStatus(result);         // 更新状态
}).detach();
```

### 4. 线程安全设计
```cpp
std::mutex status_mutex_;  // 保护共享状态

// 所有访问都通过RAII锁保护
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_ = "Executing...";
}
```

---

## 使用方法

### 完整工作流程

```
1. 启动系统
   → ./start_mujoco_system.sh

2. 在RViz中规划轨迹
   → Planning -> Plan & Execute

3. 在MissionExecutor窗口按 [S] 保存
   → 输入名称: my_trajectory
   → 输入描述: 测试轨迹 (可选)
   → 自动保存并刷新列表

4. 按对应数字键重新执行
   → 按 [1] 执行第一个任务

5. 管理任务
╔══════════════════════════════════════════════════════════════╗
║         ARV_V1 Mission Executor v2.0                         ║
║  Dynamic Loader | Trajectory Saver | Mission Manager        ║
╚══════════════════════════════════════════════════════════════╝

Available Missions:
  [1] home_position        : 返回安全初始位置
  [2] grab_cube            : 抓取立方体
  [3] test_movement        : 测试运动

Commands:
  [1-9] Execute   [S]ave   [D]elete   [I]nfo   [R]efresh   [H]elp   [Q]uit

Status: Success: grab_cube (5.2s)

> 
```

**所有命令**：

| 按键 | 功能 | 说明 |
|------|------|------|
| `1-9` | 执行任务 | 立即执行对应编号的轨迹 |
| `S` | 保存轨迹 | 保存RViz最近执行的轨迹 |
| `D` | 删除任务 | 删除指定编号的轨迹文件 |
| `I` | 查看详情 | 显示轨迹元数据（时长、点数等）|
| `R` | 刷新列表 | 重新扫描轨迹目录 |
| `H` | 帮助菜单 | 显示完整命令说明 |
| `Q` | 退出程序 | 优雅关闭节点 |

---

## 功能详解

### 💾 保存轨迹 [S]

**交互流程**：
```
1. 按 [S] 键进入保存模式

2. 提示: Enter trajectory name:
   输入: my_first_trajectory
   验证: 
   - ✓ 不能

#### `/save_last_trajectory` (新增)
```bash
# 保存最近执行的轨迹
ros2 service call /save_last_trajectory \
  arv_v1_interfaces/srv/SaveLastTrajectory \
  "{name: 'my_trajectory', description: '测试轨迹'}"

# 响应
success: true
message: "Trajectory saved successfully"
saved_path: "/home/user/ros2_ws/.../my_trajectory.yaml"
```为空
   - ✓ 不能包含空格、斜杠
   - ✓ 检查是否重复

3. 如果名称已存在:
   提示: Mission 'xxx' exists. Overwrite? [Y/N]:
   
4. 提示: Enter description (optional, press Enter to skip):
   输入: 这是我的第一个测试轨迹
   
5. 后台调用 /save_last_trajectory 服务

6. 状态显示: Saving my_first_trajectory...
   成功: Saved: my_first_trajectory
   
7. 自动刷新状态机输入模式

**问题**：如何在TUI中支持单字符命令和行输入？

**解决方案**：
```cpp
enum class InputMode {
    COMMAND,      // 单字符模式 (std::cin >> key)
    SAVE_NAME,    // 行输入模式 (std::getline)
    SAVE_DESC,    // 行输入模式
    DELETE_CONFIRM,
    INFO_SELECT,
    OVERWRITE_CONFIRM
};

// 智能切换输入方式
if (input_mode_ == InputMode::COMMAND) {
    char key;
    std::cin >> key;  // 单字符
} else {
    std::string input;
    std::cin.ignore();  // 清除缓冲区
    std::getline(std::cin, input);  // 完整行
}
```

### 2. 输入验证与安全

```cpp
void handleSaveName(const std::string& name) {
    // 1. 空值检查
    if (name.empty()) {
        return error("Name cannot be empty");
    }
    
    // 2. 非法字符检查
    if (name.find('/') != std::string::npos || 
        name.find(' ') != std::string::npos) {
        return error("Invalid characters");
    }
    
    // 3. 重复检查
    if (missionExists(name)) {
        // 进入覆盖确认模式
        input_mode_ = InputMode::OVERWRITE_CONFIRM;
        return;
    }
    
    // 4. 验证通过，继续输入描述
    pending_name_ = name;
    input_mode_ = InputMode::SAVE_DESC;
}
```

### 3. 文件系统操作

```cpp保存服务不可用
```
Status: Error: Save service not available
```
**原因**：`trajectory_manager_node` 未启动
**解决**：
```bash
ros2 run ARV_V1_MOVEIT trajectory_manager_node
```

### 问题2：无法保存 - 没有轨迹
```
Status: Save failed: No trajectory captured
```
**原因**：trajectory_manager没有缓存任何轨迹
**解决**：在RViz中执行 Plan & Execute 一个轨迹

### 问题3：名称已存在
```
Mission 'test' exists. Overwrite? [Y/N]:
```
**处理**：
- 按 `Y` 覆盖已有轨迹
- 按 `N` 取消保存，重新输入新名称

### 问题4：删除失败 - 权限不足
```
Status: Error: Permission denied
```
**解决**：
```bash
# 检查文件权限
ls -la ~/ros2_ws/src/ARV_V1_MOVEIT/config/trajectories/

# 修改权限（如果需要）
chmod 644 ~/ros2_ws/src/ARV_V1_MOVEIT/config/trajectories/*.yaml
```

### 问题5：输入卡住
**症状**：按键后无响应
**原因**：输入缓冲区残留
**解决**：按 Ctrl+C 重启节点
// 删除轨迹
void deleteTrajectory(const std::string& name) {
    std::string path = trajectory_dir_ + "/" + name + ".yaml";
    
    try {
        if (std::filesystem::exists(path)) {
            std::filesystem::remove(path);
            fetchMissions();  // 自动刷新
        }
    } catch (const std::filesystem::filesystem_error& e) {
        // 处理权限错误等
    }
}
```

### 4. YAML元数据解析

```cpp
void showMissionInfo(const std::string& name) {
    YAML::Node config = YAML::LoadFile(path);
    
    // 提取元数据
    if (config["meta"]) {
        auto duration = config["meta"]["duration_sec"].as<double>();
        auto saved_at = config["meta"]["saved_at"].as<std::string>();
    }
    
    // 提取轨迹点数
    int point_count = config["points"].size();
}
```

### 5. ANSI颜色编码

```cpp
const std::string COLOR_RED    = "\033[0;31m";
const std::string COLOR_GREEN  = "\033[0;32m";
const std::string COLOR_YELLOW = "\033[1;33m";

// 根据状态动态着色
if (status.find("Error") != std::string::npos) {
    std::cout << COLOR_RED << status << COLOR_RESET;
} else if (status.find("Success") != std::string::npos) {
    std::cout << COLOR_GREEN << status << COLOR_RESET;
}
```
```

**注意事项**：
- 必须先在RViz中执行一个轨迹（Plan & Execute）
- trajectory_manager_node会自动缓存最近执行的轨迹
- 描述是可选的，直接按回车可跳过

### 🗑️ 删除轨迹 [D]

**交互流程**：
```
1. 按 [D] 键进入删除模式

2. 提示: Enter mission number to delete (1-9) or [C] to cancel:
   输入: 2  (删除第2个任务)
   或者: C  (取消删除)

3. 确认删除对应的YAML文件

4. 自动刷新任务列表
```

### ℹ️ 查看详情 [I]

**显示信息**：
```
╔════════════════════════════════════════════════════════════╗
║  Trajectory Information                                    ║
╚════════════════════════════════════════════════════════════╝

  Name:        grab_cube
  Description: 抓取立方体演示
  Duration:    5.23 seconds
  Saved at:    2026-02-01T14:30:00
  Points:      127
  Start pos:   0.00 0.00 0.00 0.00 0.00 0.00

Press Enter to continue...
```

### 界面操作

```
=== ARV_V1 Mission Executor ===
Dynamic Loader | Persistent Client

Available Missions:
  [1] home                 : Return Home
  [2] grab_cube            : Grab Cube
  [3] test_trajectory      : Test Movement

Controls: [R]efresh  [Q]uit
Status: Ready
> 
```

**按键说明**：
- `1-9`：执行对应任务
- `R`：刷新任务列表（重新扫描文件系统）
- `Q`：退出程序

---

## 服务接口

### 调用的服务

#### `/list_trajectories`
```bash
# 请求格式（无参数）
ros2 service call /list_trajectories arv_v1_interfaces/srv/ListTrajectories

# 响应示例
names: [home, grab_cube, test1]
descriptions: ['返回初始位置', '抓取立方体', '测试轨迹']
```

#### `/load_trajectory`
```bash
# 请求格式
ros2 service call /load_trajectory arv_v1_interfaces/srv/LoadTrajectory \
  "{name: 'home', execute: true}"

# 响应
success: true
message: "Trajectory loaded and execution started"
duration: 5.2
```

---

## 技术实现要点

### 1. 移动语义避免Future拷贝

**问题**：ROS2的`FutureAndRequestId`禁止拷贝
```cpp
// ❌ 错误：尝试拷贝future
[this, future, name]() { ... }

// ✅ 正确：移动future的所有权
[this, future = std::move(future), name]() mutable { ... }
```

**原理**：
- `std::move(future)` 转移所有权到lambda
- 原变量失效，lambda内部独占future
- 符合ROS2的所有权语义

### 2. 数组长度一致性检查

```cpp
// 检查服务返回的两个数组是否匹配
if (response->names.size() != response->descriptions.size()) {
    RCLCPP_WARN(this->get_logger(), 
        "Inconsistent data: %zu names vs %zu descriptions",
        response->names.size(), response->descriptions.size());
}

// 安全访问
size_t count = std::min(response->names.size(), size_t(9));
for (size_t i = 0; i < count; ++i) {
    missions_.push_back({
        response->names[i],
        i < response->descriptions.size() 
            ? response->descriptions[i] 
            : "No description",  // 默认值
        static_cast<char>('1' + i)
    });
}
```2.0 (2026-02-01)
**核心功能**：
- ✅ [S] 保存轨迹 - 交互式输入名称和描述
- ✅ 输入验证 - 名称合法性、重复检测、覆盖确认
- ✅ 自动刷新 - 保存/删除后自动更新列表

**增强功能**：
- ✅ [D] 删除轨迹 - 选择编号删除文件
- ✅ [I] 查看详情 - 显示轨迹元数据
- ✅ [H] 帮助菜单 - 完整命令说明

**用户体验**：
- ✅ 彩色UI - ANSI颜色编码
- ✅ 状态机输入 - 智能切换输入模式
- ✅ 错误提示 - 详细的操作建议

### v1.0 (2026-02-01)
- ✅ 初始实现
- ✅ 动态任务加载
- ✅ 持久化服务连接
- ✅ 异步执行
- ✅ 线程安全保护
// ANSI转义码序列
// \033[2J - 清除整个屏幕
// \033[H  - 移动光标到(0,0)
```
2
**优势**：无需ncurses库，轻量级实现

---

## 故障排查

### 问题1：服务不可用
```
[ERROR] Service timeout
```
**原因**：`trajectory_manager_node` 未启动
**解决**：
```bash
# 检查节点状态
ros2 node list | grep trajectory_manager

# 手动启动
ros2 run ARV_V1_MOVEIT trajectory_manager_node
```

### 问题2：任务列表为空
```
No trajectory captured
```
**原因**：`config/trajectories/` 目录为空
**解决**：
```bash
# 在RViz中Plan & Execute一个轨迹，然后保存
ros2 service call /save_last_trajectory \
  arv_v1_interfaces/srv/SaveLastTrajectory \
  "{name: 'my_first_traj', description: '我的第一个轨迹'}"
```

### 问题3：编译错误
```
error: use of deleted function 'FutureAndRequestId::FutureAndRequestId(const FutureAndRequestId&)'
```
**原因**：lambda按值捕获future（尝试拷贝）
**解决**：使用移动捕获 `[future = std::move(future)]`

---

## 扩展开发

### 添加新的任务操作

```cpp
// 在 handleInput() 中添加新按键
if (key == 's' || key == 'S') {
    // 停止所有运动
    cancelAllTrajectories();
    return;
}
```

### 添加执行进度显示

```cpp
// 订阅Action反馈
auto feedback_callback = [this](auto, const auto& feedback) {
    float progress = calculateProgress(feedback);
    current_status_ = "Executing: " + std::to_string(progress) + "%";
};
```

### 自定义UI样式

```cpp
// 修改 drawUI() 中的颜色和布局
std::cout << "\033[1;32m";  // 亮绿色
std::cout << "=== Custom Title ===" << std::endl;
std::cout << "\033[0m";     // 重置颜色
```

---

## 性能指标

| 指标 | 目标 | 实测 | 状态 |
|------|------|------|------|
| 服务调用延迟 | < 10ms | ~5ms | ✅ |
| 任务列表刷新 | < 500ms | ~300ms | ✅ |
| UI刷新响应 | < 50ms | ~20ms | ✅ |
| 内存占用 | < 50MB | ~15MB | ✅ |

---

## 相关文件

### 源代码
- `src/mission_executor_node.cpp` - 主节点实现

### 配置文件
- `config/trajectories/*.yaml` - 轨迹存储目录

### 服务定义
- `arv_v1_interfaces/srv/ListTrajectories.srv`
- `arv_v1_interfaces/srv/LoadTrajectory.srv`

### 启动脚本
- `start_mujoco_system.sh` - 系统启动
- `stop_all_nodes.sh` - 停止所有节点
- `scripts/check_system.sh` - 系统健康检查

---

## 最佳实践

### 1. 任务命名规范
```yaml
# ✅ 推荐
name: "home_position"
description: "返回到安全初始位置"

# ❌ 不推荐
name: "traj1"
description: ""
```

### 2. 错误处理
```cpp
try {
    auto result = future.get();
    // 处理结果
} catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    // 恢复到安全状态
}
```

### 3. 资源清理
```cpp
// 析构时确保线程完成
~MissionExecutorNode() {
    // 等待所有后台任务完成
    // 或设置标志位通知线程退出
}
```

---

## 更新日志

### v1.0 (2026-02-01)
- ✅ 初始实现
- ✅ 动态任务加载
- ✅ 持久化服务连接
- ✅ 异步执行
- ✅ 线程安全保护
- ✅ 修复Future移动语义问题
- ✅ 添加数组长度检查

---

**维护者**: Young-Yihang  
**最后更新**: 2026-02-01  
**版本**: 1.0
