# ARV_V1 力矩控制系统技术文档 + 功能扩展蓝图

## 当前系统架构 (v1.0 - 基础力矩控制)

```
RViz/MoveIt (规划)
     │ 轨迹 Goal
     ▼
torque_controller_node (200 Hz)
     │ 计算: τ = τ_ff + τ_fb
     │ - 前馈: M(q)q̈ + C(q,q̇) + G(q)
     │ - 反馈: Kp·e_p + Kd·e_v
     │ - Kalman: 速度滤波 (可选)
     ▼ /effort_controller/commands
mujoco_interface_node (200 Hz 仿真 + 60 Hz 渲染)
     │ 物理仿真 + 3D 可视化
     ▼ /joint_states
dynamics_computer (KDL 动力学库)
```

## 未来系统架构 (v2.0+ - 多功能集成)

```
┌─────────────────────────────────────────────────────────────────┐
│                      感知层 (Perception)                         │
├─────────────────────────────────────────────────────────────────┤
│  [Eye-in-Hand 相机] ──┐                                          │
│   └─ RealSense D435i  │                                          │
│                       ├──> visual_servo_node (30Hz)              │
│  [外部深度相机] ──────┘       │ PBVS/IBVS                        │
│   └─ RealSense D405          │ 目标跟踪                          │
│                              │                                   │
│  [3D LiDAR (可选)] ───────> obstacle_avoidance_node (20Hz)      │
│   └─ RPLIDAR A3 / Livox      │ 点云处理                          │
│                              │ 碰撞检测                          │
└──────────────────────────────┼──────────────────────────────────┘
                               │ /target_pose, /obstacle_cloud
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                      规划层 (Planning)                           │
├─────────────────────────────────────────────────────────────────┤
│  MoveIt2 + RViz                                                  │
│   ├─ 运动规划 (OMPL/Pilz)                                       │
│   ├─ 碰撞检测 (FCL + 动态障碍物)                                 │
│   └─ 实时重规划 (MoveIt Servo)                                  │
└──────────────────────────────┬──────────────────────────────────┘
                               │ /follow_joint_trajectory
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                      控制层 (Control)                            │
├─────────────────────────────────────────────────────────────────┤
│  torque_controller_node (200Hz)                                  │
│   ├─ 力矩控制: τ = τ_ff + τ_fb                                   │
│   ├─ Kalman 滤波                                                 │
│   └─ 安全限制                                                    │
└──────────────────────────────┬──────────────────────────────────┘
                               │ /effort_controller/commands
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    执行层 (Execution)                            │
├─────────────────────────────────────────────────────────────────┤
│  [仿真] mujoco_interface_node (200Hz)                            │
│  [实物] serial_interface_node (200Hz)                            │
│         └─ STM32 串口通信 (CAN/UART)                             │
└─────────────────────────────────────────────────────────────────┘
```

## 状态机

```
启动 → 等待 Goal → 保持模式 (Hold) ⇄ 执行模式 (Execute)
                       ↑_____________↓
                         轨迹完成返回
```

**切换逻辑**:
- 启动: 首次收到 `/joint_states` → 保存 `q_target_` → 保持模式
- 接收 Goal: 保存轨迹终点 → 执行模式
- 完成: 使用轨迹终点作为新 `q_target_` → 保持模式

## 控制律

**保持模式**: `τ = G(q) + Kp·(q_t - q) + Kd·(0 - q̇)`  
**执行模式**: `τ = M(q_d)q̈_d + C(q_d,q̇_d) + G(q_d) + Kp·(q_d - q) + Kd·(q̇_d - q̇)`

---

## 核心参数

### PD 增益
```cpp
// torque_controller_node.cpp:35-47
Kp: [700, 1000, 650, 150, 20, 5]    // N·m/rad
Kd: [11, 17, 15, 6, 2, 1]           // N·m·s/rad
```

### Kalman 滤波器 (可选启用)
```cpp
// 状态: x = [q, q̇]ᵀ  (仅滤波速度，位置保持编码器精度)
Q_pos: 1e-10    // 位置过程噪声
Q_vel: 1e-7     // 速度过程噪声 (调大 → 响应快，调小 → 平滑)
R_pos: 1e-3     // 位置测量噪声
R_vel: 2.5e-2   // 速度测量噪声

// 增益矩阵判断:
// K11, K22 < 0.05  → 过度平滑，增大 Q_vel
// K ∈ [0.1, 0.3]   → 平衡配置
// K > 0.5          → 过度信任测量，减小 Q_vel
```

### 控制频率
- 力矩控制器: **200 Hz** (5ms 周期)
- MuJoCo 仿真: **200 Hz**
- 渲染线程: **60 Hz**

### 执行器限制
- 最大力矩: **±20 N·m** (MuJoCo 配置)

---

## 关键代码位置

| 功能 | 文件 | 行数 |
|------|------|------|
| PD 增益初始化 | torque_controller_node.cpp | 35-47 |
| Kalman 参数声明 | torque_controller_node.cpp | 93-107 |
| 状态机变量 | torque_controller_node.cpp | 100-115 |
| 保存启动姿态 | torque_controller_node.cpp | 270-284 |
| 保存规划终点 | torque_controller_node.cpp | 220-236 |
| Kalman 滤波回调 | torque_controller_node.cpp | 330-390 |
| 保持模式控制 | torque_controller_node.cpp | 490-545 |
| 执行模式控制 | torque_controller_node.cpp | 547-669 |
| 动力学计算 | dynamics_computer.cpp | 11-52 |
| Kalman 实现 | kalman_filter.{cpp,hpp} | - |
| MuJoCo 碰撞配置 | mujoco_interface_node.cpp | 195-210 |
| UI 渲染 (mjr_text) | mujoco_interface_node.cpp | 882-970 |
| 交互回调 | mujoco_interface_node.cpp | 640-730 |

---

## 已解决的关键问题

### 1. Kalman 滤波器运行时崩溃 🔥
- **现象**: `ParameterNotDeclaredException: kalman.Q_pos`
- **原因**: 成员初始化列表在构造函数体之前执行,试图读取未声明参数
- **解决**: 移动参数声明/读取到构造函数体内 (93-107 行)

### 2. Joint1 运动极慢问题 🔥
- **现象**: Joint1 速度恒定 ~0.06 rad/s,执行器力矩达数千 N·m 无效
- **根因**: MuJoCo 碰撞约束力抵消执行器力矩
  ```
  qfrc_actuator   = -20.0 N·m  (执行器)
  qfrc_constraint = +19.5 N·m  (约束力)
  ────────────────────────────────────
  τ_net           ≈   0.0 N·m
  ```
- **碰撞源**: Link1 与 world (地面), Link4 与 Link6 自碰撞
- **解决**: 为所有 `<geom>` 添加 `contype="0" conaffinity="0"` 禁用碰撞检测

### 3. 力矩高频振荡 (±2 N·m)
- **现象**: 位置/速度平滑,但力矩输出振荡
- **原因分析**:
  1. Kd 放大相位滞后的速度误差 (主因)
  2. Kalman 增益过小 (K < 0.05) → 10-15ms 相位延迟
  3. Q/R 比值过小 (4e-6) → 过度平滑
- **建议解决**:
  - 增大 `Q_vel`: 1e-7 → 1e-5 (减小相位滞后)
  - 调整 Kd 至临界阻尼: ζ = Kd/(2√(M·Kp)) ≈ 0.7
  - 目标增益: K ∈ [0.1, 0.3]

### 4. MuJoCo UI 文本重叠
- **现象**: 所有 UI 文本渲染在同一位置
- **原因**: `mjr_overlay` 只接受粗粒度网格位置 (mjGRID_TOPLEFT)
- **解决**: 替换为 `mjr_text` 手动计算 Y 坐标: `start_y - line * height`

### 5. 死锁导致控制停止
- **问题**: `controlLoop()` 嵌套加锁 → 永久阻塞
- **解决**: 删除嵌套锁,外层加锁一次即可

### 6. 启动时机械臂漂移
- **问题**: 启动后无目标位置 → PD 无效 → 重力漂移
- **解决**: 首次收到状态时保存为 `q_target_`,立刻启用 PD

### 7. 轨迹完成后掉落
- **问题**: 规划终点未保存 → 完成后 PD 目标错误
- **解决**: 在 `handleAccepted()` 保存轨迹终点

### 8. RViz 机械臂闪烁
- **问题**: 两个节点同时发布 `/joint_states`
- **解决**: `mujoco_demo.launch.py` 禁用 `joint_state_broadcaster`

### 9. MuJoCo 窗口黑屏
- **问题**: OpenGL 上下文在错误线程创建
- **解决**: 在渲染线程中 `glfwMakeContextCurrent()` + `mjr_makeContext()`

### 10. 执行模式控制律错误
- **问题**: 前馈只用重力补偿,PD 期望位置用了实际位置
- **解决**: 
  - 前馈改用完整动力学: `computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff)`
  - PD 使用正确期望值: `computeFeedbackTorque(q_d, qd_d, q_actual, qd_actual, tau_fb)`

---

## 待优化项

### 🔥 高优先级

1. **Kalman 滤波器调参** (解决力矩振荡)
   ```bash
   ros2 param set /torque_controller_action_server kalman.Q_vel 1e-5
   # 观察 K22 是否增大到 0.1-0.3
   # 检查力矩振荡是否减小
   ```

2. **Kd 参数优化** (当前可能过大)
   - Joint 2: 测试 Kd = 10 (vs 当前 17)
   - 目标阻尼比: ζ = Kd/(2√(M·Kp)) ≈ 0.7

3. **力矩限幅**
   ```cpp
   const double MAX_TORQUE[6] = {20, 20, 20, 20, 20, 20};
   tau_total(i) = std::clamp(tau_total(i), -MAX_TORQUE[i], MAX_TORQUE[i]);
   ```

### ⚙️ 中优先级

4. **性能监控**
   - 控制循环耗时统计
   - CPU 占用率监控
   - 频率偏差检测

5. **速度信号质量**
   - 检查 MuJoCo 速度输出噪声
   - 验证速度数值范围合理性

---

## Kalman 滤波调参指南

### 理论基础
- **状态空间**: `x_{k+1} = F·x_k + w_k`, `z_k = H·x_k + v_k`
- **增益公式**: `K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹`
- **关键关系**: K 大小取决于 Q/R 比值,而非绝对值
- **误区**: K 矩阵元素平方和 ≠ 1 (无此约束)

### 判断方法

#### 1. 检查对角元素 (K11, K22)
- K < 0.05: 过度平滑 → 相位滞后严重 → 增大 Q
- K ∈ [0.1, 0.3]: 平衡 (推荐)
- K > 0.5: 过度信任测量 → 噪声放大 → 减小 Q

#### 2. 检查非对角元素 (K12, K21)
- K21 > K22: 位置测量显著影响速度估计 (正常)
- K12 接近 0: 速度测量不影响位置 (本系统使用编码器,精度高)

#### 3. 计算 Q/R 比值
- 位置: Q_pos/R_pos = 1e-10/1e-3 = 1e-7 (极保守)
- 速度: Q_vel/R_vel = 1e-7/2.5e-2 = 4e-6 (过度平滑)
- **目标**: Q/R ∈ [1e-4, 1e-3] 

#### 4. 判断滤波特性
- Q/R 小 → 信任模型 → 平滑但滞后
- Q/R 大 → 信任测量 → 响应快但噪声大

#### 5. 关联力矩振荡
- K < 0.05 → 10-15ms 相位延迟
- Kd·e_v 放大延迟误差 → ±2 N·m 振荡
- **解决**: 增大 Q_vel → K 增大 → 减小延迟

### 调参步骤

1. **初步评估**: 打印当前 K 矩阵
   ```cpp
   Eigen::Matrix2d K = joint_filters_[i].getKalmanGain();
   RCLCPP_INFO("K = [[%.4f, %.4f], [%.4f, %.4f]]", ...);
   ```

2. **调整速度噪声**: 
   - 振荡/滞后严重 → 增大 Q_vel (1e-7 → 1e-6 → 1e-5)
   - 噪声放大 → 减小 Q_vel

3. **验证效果**:
   - 绘制力矩波形 (PlotJuggler)
   - 检查相位延迟 (目标 < 5ms)
   - 测量振荡幅值 (目标 < 0.5 N·m)

4. **迭代优化**: 重复步骤 2-3 直至 K ∈ [0.1, 0.3]

---

## 快速启动

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-select ARV_V1_MOVEIT
source install/setup.bash
```

### 启动
```bash
# 一键启动 (推荐)
bash src/ARV_V1_MOVEIT/bash/start_mujoco_system.sh

# 或手动启动三个终端:
# 1. MuJoCo 仿真
ros2 run ARV_V1_MOVEIT mujoco_interface_node

# 2. 力矩控制器
ros2 run ARV_V1_MOVEIT torque_controller_node

# 3. MoveIt + RViz
ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py
```

### 测试
```bash
# 检查节点
ros2 node list | grep -E "(torque|mujoco)"

# 监控力矩输出
ros2 topic echo /effort_controller/commands

# 监控关节状态
ros2 topic hz /joint_states

# Kalman 参数调整
ros2 param set /torque_controller_action_server kalman.Q_vel 1e-5

# 在 RViz 中规划并执行
# Motion Planning → Plan → Execute
```

---

# 🚀 项目扩展蓝图 (Feature Roadmap)

## Git 分支管理策略

```
master (主分支 - 稳定版本)
  │
  ├── feature/kalman          ✅ [已完成] Kalman 滤波器集成
  │
  ├── feature/visual-servo    🔧 [规划中] 视觉伺服功能
  │   ├── eye-in-hand 配置
  │   ├── PBVS/IBVS 算法
  │   └── 目标识别与跟踪
  │
  ├── feature/serial-output   🔧 [规划中] 实物串口通信
  │   ├── STM32 串口协议
  │   ├── CAN 总线驱动
  │   └── 硬件接口节点
  │
  ├── feature/obstacle-avoidance 🔧 [规划中] 动态避障
  │   ├── 点云处理
  │   ├── 碰撞检测
  │   └── 实时重规划
  │
  └── release/v2.0            📅 [未来] 多功能集成版本
```

---

## 📷 功能模块 1: 视觉伺服 (Visual Servoing)

### 1.1 硬件方案推荐

#### **方案 A: Eye-in-Hand (推荐)** ⭐
**硬件配置**:
- **相机**: Intel RealSense D435i
  - RGB: 1920×1080 @ 30fps
  - 深度: 1280×720 @ 30fps (立体视觉)
  - IMU: 6-DoF 加速度计+陀螺仪
  - 价格: ~¥2000
  - 重量: 72g (适合末端安装)
  
- **安装位置**: 机械臂末端 (link6)
  - 设计 3D 打印支架
  - 连接末端法兰盘
  - USB 3.0 数据线管理

**优势**:
- ✅ 相机随末端移动,视野灵活
- ✅ 适合精细操作 (抓取、装配)
- ✅ 自带深度信息
- ✅ ROS2 官方支持好 (`realsense-ros`)

**劣势**:
- ❌ 增加末端负载 (~100g)
- ❌ 需要手眼标定 (Camera-to-Flange)

---

#### **方案 B: Eye-to-Hand (备选)**
**硬件配置**:
- **相机**: Intel RealSense D405 (短距专用版)
  - 工作距离: 7-100cm
  - 视野角: 87° × 58°
  - 价格: ~¥1500
  
- **安装位置**: 固定在工作台/外部支架
  - 俯视或侧视角度
  - 覆盖整个工作空间

**优势**:
- ✅ 不影响末端负载
- ✅ 工作空间视野完整
- ✅ 标定简单 (Base-to-Camera)

**劣势**:
- ❌ 遮挡问题 (机械臂挡住目标)
- ❌ 不适合精细操作

---

### 1.2 算法方案

#### **PBVS (Position-Based Visual Servoing)** ⭐ 推荐
**原理**: 先估计目标 3D 位姿 → 生成笛卡尔空间轨迹 → MoveIt 规划

**实现流程**:
```
1. 目标检测 (YOLO/AprilTag)
   ├─ RGB 图像识别
   └─ 输出目标 2D 边界框/角点

2. 位姿估计 (PnP/点云配准)
   ├─ 深度图提取 3D 点云
   ├─ ICP/PnP 计算目标位姿
   └─ 输出: T_target^camera (4×4 变换矩阵)

3. 坐标转换
   ├─ T_target^base = T_camera^base × T_target^camera
   └─ 发布到 /target_pose

4. MoveIt 规划
   ├─ 订阅 /target_pose
   ├─ 笛卡尔路径规划
   └─ 执行抓取动作
```

**核心代码** (Python 示例):
```python
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
import cv2, numpy as np
from cv_bridge import CvBridge

class PBVSNode(Node):
    def __init__(self):
        super().__init__('pbvs_node')
        
        # 订阅相机话题
        self.create_subscription(Image, '/camera/color/image_raw', 
                                self.image_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                self.depth_callback, 10)
        
        # 发布目标位姿
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # 相机内参 (从 CameraInfo 获取)
        self.K = None  # 3x3 内参矩阵
        
    def image_callback(self, msg):
        # 1. 检测目标 (ArUco/AprilTag)
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids = self.detect_aruco(image)
        
        if ids is None:
            return
        
        # 2. PnP 位姿估计
        rvec, tvec = cv2.solvePnP(object_points, corners, self.K, None)
        
        # 3. 转换到 base 坐标系
        T_target_base = self.transform_to_base(rvec, tvec)
        
        # 4. 发布目标位姿
        pose_msg = self.to_pose_msg(T_target_base)
        self.pose_pub.publish(pose_msg)
```

**优势**:
- ✅ 控制稳定 (闭环在关节空间)
- ✅ 可利用 MoveIt 避障
- ✅ 对相机标定误差鲁棒

**劣势**:
- ❌ 依赖深度信息质量
- ❌ 计算延迟 (检测+估计+规划)

---

#### **IBVS (Image-Based Visual Servoing)** (备选)
**原理**: 直接在图像空间控制 → 雅可比矩阵映射到关节速度

**实现流程**:
```
1. 提取图像特征 (s = [u, v, ...])
2. 计算误差: e = s_desired - s_current
3. 雅可比矩阵: J = ∂s/∂q (图像到关节)
4. 速度控制: q̇ = -λ·J^†·e
```

**优势**:
- ✅ 无需深度信息
- ✅ 反应快 (直接控制)

**劣势**:
- ❌ 易陷入局部极小值
- ❌ 不易集成避障

---

### 1.3 ROS2 实现架构

```
visual_servo_node (30Hz)
  │
  ├─ 订阅话题:
  │   ├─ /camera/color/image_raw      (RGB 图像)
  │   ├─ /camera/depth/image_rect_raw (深度图)
  │   └─ /joint_states                (当前关节角)
  │
  ├─ 发布话题:
  │   ├─ /target_pose                 (目标位姿)
  │   └─ /visual_servo/debug_image    (可视化图像)
  │
  └─ 调用服务:
      └─ /compute_ik                  (逆运动学求解)
```

**依赖库**:
```xml
<!-- package.xml -->
<depend>realsense2_camera</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>moveit_ros_planning_interface</depend>
<depend>apriltag_ros</depend>
```

---

## 🛡️ 功能模块 2: 动态避障 (Obstacle Avoidance)

### 2.1 硬件方案

#### **方案 A: 3D LiDAR (推荐)** ⭐
**硬件**:
- **RPLIDAR A3**: 
  - 扫描距离: 25m
  - 频率: 20Hz
  - 价格: ~¥2500
  - 接口: USB/UART
  
- **Livox Mid-360**:
  - 非重复扫描
  - FOV: 360° × 59°
  - 价格: ~¥4000
  - 精度更高

**安装位置**: 
- 工作台顶部/侧面
- 覆盖机械臂工作空间

**优势**:
- ✅ 大范围障碍物检测
- ✅ 实时点云数据
- ✅ 不受光照影响

---

#### **方案 B: 深度相机阵列 (备选)**
**硬件**: 2-3 个 RealSense D435i
- 多角度覆盖工作空间
- 点云融合

**优势**:
- ✅ 成本低 (如果已有相机)
- ✅ RGB+Depth 融合

**劣势**:
- ❌ 数据量大
- ❌ 需要多相机标定

---

### 2.2 算法方案

#### **实时点云处理流程**:
```
1. 点云获取
   ├─ LiDAR 扫描 → PointCloud2
   └─ 或深度相机 → 转换为点云

2. 点云滤波
   ├─ 体素滤波 (降采样)
   ├─ 统计滤波 (去噪)
   └─ 移除地面/机械臂自身

3. 障碍物检测
   ├─ 欧氏聚类
   ├─ 包围盒计算 (AABB/OBB)
   └─ 输出障碍物列表

4. 动态更新 MoveIt Planning Scene
   ├─ 发布 CollisionObject
   ├─ 标记为移动障碍物
   └─ 触发重规划
```

**核心代码** (C++):
```cpp
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
        // 订阅点云
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan_points", 10, 
            std::bind(&ObstacleAvoidanceNode::cloudCallback, this, _1)
        );
        
        // MoveIt 规划场景接口
        planning_scene_interface_ = 
            std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    }
    
private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. 转换点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // 2. 滤波
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.01, 0.01, 0.01);  // 1cm 体素
        voxel_filter.filter(*cloud);
        
        // 3. 聚类检测障碍物
        auto obstacles = detectObstacles(cloud);
        
        // 4. 更新 MoveIt 规划场景
        updatePlanningScene(obstacles);
    }
    
    void updatePlanningScene(const std::vector<Obstacle>& obstacles) {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        for (const auto& obs : obstacles) {
            moveit_msgs::msg::CollisionObject obj;
            obj.header.frame_id = "base_link";
            obj.id = "obstacle_" + std::to_string(obs.id);
            
            // 添加包围盒
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {obs.size_x, obs.size_y, obs.size_z};
            
            obj.primitives.push_back(primitive);
            obj.primitive_poses.push_back(obs.pose);
            obj.operation = obj.ADD;
            
            collision_objects.push_back(obj);
        }
        
        planning_scene_interface_->applyCollisionObjects(collision_objects);
    }
};
```

---

### 2.3 与 MoveIt 集成

**实时重规划策略**:
```cpp
// 订阅碰撞预警
auto collision_sub = create_subscription<std_msgs::msg::Bool>(
    "/collision_warning", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && is_executing_) {
            // 1. 停止当前轨迹
            current_goal_handle_->abort();
            
            // 2. 触发重规划
            replan_service_->async_send_request(replan_req);
        }
    }
);
```

---

## 🔌 功能模块 3: 串口输出 (Serial Interface)

### 3.1 硬件通信方案

#### **方案 A: UART 串口 (简单)**
**硬件**: 
- USB-TTL 转换器 (CH340/CP2102)
- STM32 UART 接口

**协议设计**:
```cpp
// 数据包格式 (20 bytes)
struct TorqueCommand {
    uint8_t  header[2];      // 帧头: 0xAA, 0x55
    float    torque[6];      // 6 关节力矩 (4 bytes × 6)
    uint16_t checksum;       // CRC16 校验
} __attribute__((packed));
```

**通信频率**: 200Hz (与控制频率同步)

---

#### **方案 B: CAN 总线 (推荐)** ⭐
**硬件**:
- USB-CAN 适配器 (PEAK PCAN/ZLG USBCAN)
- 每个关节独立 CAN ID

**优势**:
- ✅ 抗干扰能力强
- ✅ 多节点总线拓扑
- ✅ 实时性好 (< 1ms)

**CAN 帧设计**:
```cpp
// Joint 1 力矩指令
CAN ID: 0x101
Data: [torque_high_byte, torque_low_byte, 0x00, ...]

// 状态反馈
CAN ID: 0x201
Data: [position, velocity, current, ...]
```

---

### 3.2 ROS2 实现

```cpp
class SerialInterfaceNode : public rclcpp::Node {
public:
    SerialInterfaceNode() : Node("serial_interface_node") {
        // 订阅力矩指令
        torque_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10,
            std::bind(&SerialInterfaceNode::torqueCallback, this, _1)
        );
        
        // 发布关节状态
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10
        );
        
        // 打开串口
        serial_port_.open("/dev/ttyUSB0");
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(921600));
    }
    
private:
    void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        TorqueCommand cmd;
        cmd.header[0] = 0xAA;
        cmd.header[1] = 0x55;
        
        for (size_t i = 0; i < 6; i++) {
            cmd.torque[i] = static_cast<float>(msg->data[i]);
        }
        
        cmd.checksum = calculateCRC16(&cmd, sizeof(cmd) - 2);
        
        // 发送到串口
        boost::asio::write(serial_port_, boost::asio::buffer(&cmd, sizeof(cmd)));
    }
    
    boost::asio::serial_port serial_port_;
};
```

---

## 📊 系统集成与测试

### 4.1 分支合并策略

```bash
# 开发新功能
git checkout -b feature/visual-servo
# ... 开发完成后 ...
git checkout master
git merge --no-ff feature/visual-servo  # 保留分支历史

# 测试多功能组合
git checkout -b integration/vs-oa  # visual-servo + obstacle-avoidance
git merge feature/visual-servo
git merge feature/obstacle-avoidance
# 解决冲突并测试
```

---

### 4.2 性能指标

| 功能模块 | 频率 | 延迟 | CPU 占用 |
|---------|------|------|---------|
| 力矩控制 | 200Hz | < 5ms | ~15% |
| 视觉伺服 | 30Hz | < 50ms | ~25% |
| 避障检测 | 20Hz | < 100ms | ~20% |
| 串口通信 | 200Hz | < 2ms | ~5% |
| **总计** | - | - | **~65%** |

**硬件需求**: 
- CPU: Intel i5 8代+ (4核)
- RAM: 8GB+
- GPU: GTX 1050+ (YOLO 推理)

---

### 4.3 调试工具

```bash
# 1. 相机标定
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025

# 2. 点云可视化
ros2 run rviz2 rviz2 -d config/obstacle_detection.rviz

# 3. 串口监控
ros2 topic echo /serial/diagnostics

# 4. 性能分析
ros2 run ros2_performance performance_test --topic /effort_controller/commands
```

---

## 🎯 开发优先级建议

### Phase 1: 基础功能完善 (当前)
- [x] 力矩控制核心
- [x] Kalman 滤波器
- [ ] 力矩限幅与安全机制
- [ ] 参数自动调优

### Phase 2: 硬件扩展 (3-6个月)
- [ ] 串口通信协议实现
- [ ] 实物测试平台搭建
- [ ] 硬件接口调试

### Phase 3: 感知集成 (6-9个月)
- [ ] RealSense 相机集成
- [ ] PBVS 视觉伺服
- [ ] 目标识别与跟踪

### Phase 4: 智能避障 (9-12个月)
- [ ] LiDAR 点云处理
- [ ] 动态障碍物检测
- [ ] 实时重规划

### Phase 5: 系统优化 (12个月+)
- [ ] 多传感器融合
- [ ] 机器学习优化控制
- [ ] 云端监控平台

---

## 💡 关键技术难点

### 1. 手眼标定 (Eye-in-Hand)
**问题**: 相机与末端法兰坐标系关系未知  
**方案**: 
- 使用标定板 (ChArUco/AprilTag)
- easy_handeye2 ROS2 包
- 至少 15 组不同位姿数据

### 2. 实时性保证
**问题**: 视觉处理 30Hz + 控制 200Hz 冲突  
**方案**:
- 异步架构: 视觉节点与控制节点解耦
- 共享内存: 减少话题传输延迟
- GPU 加速: CUDA 加速图像处理

### 3. 碰撞检测误报
**问题**: 机械臂自身点云误识别为障碍物  
**方案**:
- 自碰撞过滤: 根据 URDF 模型移除
- 动态掩码: 实时更新机械臂占用空间
- 工作空间限制: ROI 裁剪

---

## 📚 参考资源

### 论文
- [PBVS] "Visual Servoing: A Tutorial" - F. Chaumette, 2006
- [避障] "Real-time Obstacle Avoidance for Manipulators" - O. Khatib, 1986

### 开源项目
- [visp_ros](https://github.com/lagadic/visp_ros) - 视觉伺服库
- [moveit_servo](https://github.com/ros-planning/moveit2) - 实时伺服
- [pcl_ros](https://github.com/ros-perception/perception_pcl) - 点云处理

### 硬件文档
- [RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [RPLIDAR](https://github.com/Slamtec/rplidar_ros)

---

**最后更新**: 2025-11-06  
**状态**: 基础系统稳定 ✅ | 扩展功能规划完成 📋  
**下一步**: Kalman 调参 → 串口协议实现 → 相机选型采购

### MuJoCo 交互
- **鼠标左键**: 旋转视角
- **鼠标右键**: 平移视角
- **滚轮/中键**: 缩放
- **空格**: 暂停/继续
- **H**: 隐藏/显示 UI
- **R**: 重置相机
- **ESC**: 退出

---

## 技术笔记

### Joint1 重力项为何接近 0?
Joint1 绕 Z 轴旋转,重力 (0,0,-9.81) 平行于旋转轴 → 力臂 = 0 → τ_g ≈ 0

### MuJoCo 碰撞配置
```xml
<!-- 方案1: 禁用所有接触 -->
<size nconmax="0" njmax="0"/>

<!-- 方案2: 禁用单个几何体碰撞 (当前使用) -->
<geom ... contype="0" conaffinity="0"/>
```

### MuJoCo vs Gazebo
| 特性 | MuJoCo | Gazebo |
|------|--------|--------|
| 物理仿真 | ✅ 快速精确 | ✅ 完善 |
| 可视化 | ✅ OpenGL 3D | ✅ 3D 窗口 |
| ROS2 Jazzy | ✅ 正常 | ❌ Repo Bug |

**选择原因**: Gazebo Harmonic + ROS2 Jazzy 有系统级依赖问题,MuJoCo 轻量快速

### 控制频率选择
- **100Hz**: 慢速运动,计算负载低
- **200Hz**: 标准控制 (当前),平衡性能
- **1000Hz**: 高速/高精度,计算负载高

---

**最后更新**: 2025-01-XX  
**状态**: 系统稳定运行 ✅ | Kalman 滤波器已集成 🔧  
**当前配置**: PD + 完整动力学前馈 + 可选 Kalman 速度滤波  
**下一步**: Kalman 调参 → Kd 优化 → 性能监控
