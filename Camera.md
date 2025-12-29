  ---
  # ARV_V1 机械臂项目评审：海康威视工业相机集成与物体识别夹取功能评估

  ## 1. 项目现状概述

  ### 1.1 当前项目架构
  ARV_V1_MODEL/       # URDF模型 + STL网格文件
  ├── urdf/ARV_V1_MODEL.urdf    # 6-DOF机械臂URDF模型
  ├── meshes/*.STL              # 各link的3D网格文件
  └── config/                   # 关节配置

  ARV_V1_MOVEIT/      # MoveIt配置 + 控制节点
  ├── src/
  │   ├── torque_controller_node.cpp    # 力矩控制器 (前馈+PD/级联PID)
  │   ├── mujoco_interface_node.cpp     # MuJoCo仿真接口 (发布/joint_states)
  │   ├── hardware_interface_node.cpp   # 硬件串口接口 (Sim2Real)
  │   ├── dynamics_computer.cpp/hpp     # KDL动力学计算 (M/C/G矩阵)
  │   ├── kalman_filter.cpp/hpp         # 卡尔曼滤波器 (速度滤波)
  │   ├── cascade_pid.cpp/hpp           # 级联PID控制器
  │   └── serial_protocol.hpp           # 串口通信协议 (CRC8校验)
  ├── launch/
  │   └── mujoco_demo.launch.py         # MuJoCo仿真启动文件
  └── config/
      ├── moveit_controllers.yaml       # MoveIt控制器配置
      └── ARV_V1_MODEL.srdf             # MoveIt语义描述

  ### 1.2 当前实现的功能
  | 功能模块 | 状态 | 说明 |
  |---------|------|------|
  | URDF建模 | ✅ 完成 | 6-DOF机械臂，含惯性参数 |
  | MoveIt2规划 | ✅ 完成 | FollowJointTrajectory Action |
  | 力矩控制 | ✅ 完成 | 前馈(M·q̈+C+G) + PD/级联PID |
  | MuJoCo仿真 | ✅ 完成 | 200Hz物理仿真 + 可视化窗口 |
  | 硬件串口通信 | ✅ 完成 | 921600波特率, CRC8校验 |
  | 卡尔曼滤波 | ✅ 完成 | 速度估计滤波 |

  ### 1.3 系统数据流
  [MoveIt2 RViz]
       ↓ FollowJointTrajectory Action
  [torque_controller_node]
       │ 订阅: /joint_states
       │ 计算: 前馈+反馈力矩
       ↓ 发布: /effort_controller/commands
  [mujoco_interface_node] ←或→ [hardware_interface_node]
       ↓ 发布: /joint_states (仿真)      ↓ 串口通信 (实机)

  ---

  ## 2. 当前项目不足之处分析 (影响视觉抓取接口)

  ### 2.1 末端执行器缺失 [严重 - P0]
  | 问题 | 具体表现 | 影响 |
  |------|---------|------|
  | URDF无夹爪模型 | link6_2006roll为末端，无gripper_link | 无法进行夹取仿真和规划 |
  | 无tool0坐标系 | SRDF中tip_link="link6_2006roll" | 手眼标定困难，TCP定义不明确 |
  | 无夹爪控制接口 | 仅有6轴力矩控制 | 无法控制夹爪开合 |
  | 无夹爪关节定义 | URDF中无gripper_joint | MoveIt无法规划夹爪动作 |

  **建议修改的文件:**
  - `ARV_V1_MODEL/urdf/ARV_V1_MODEL.urdf` - 添加gripper link和joint
  - `ARV_V1_MOVEIT/config/ARV_V1_MODEL.srdf` - 添加end_effector组

  ### 2.2 视觉感知模块完全缺失 [严重 - P0]
  | 问题 | 影响 |
  |------|------|
  | 无相机驱动节点 | 无法获取图像数据 |
  | 无图像话题 | 无法接入视觉处理流程 |
  | 无手眼标定数据 | 无法将像素坐标转换为机械臂坐标系 |
  | 无深度信息接口 | 无法获取物体3D位置 (仅2D相机时) |
  | 无TF变换 | camera_frame未定义 |

  ### 2.3 运动规划接口不完整 [中等 - P1]
  | 问题 | 当前状态 | 需要 |
  |------|---------|------|
  | 仅支持关节空间 | FollowJointTrajectory | 需支持笛卡尔空间MoveL/MoveP |
  | 无抓取姿态生成 | 需手动计算 | 自动生成approach/grasp/retreat路径 |
  | 无碰撞场景管理 | 静态场景 | 动态添加检测到的物体作为障碍物 |
  | 无力控接口 | 纯位置控制 | 抓取时需要力/阻抗控制 |

  ### 2.4 话题命名空间问题 [低 - P2]
  | 问题 | 说明 |
  |------|------|
  | hardware_interface发布到/hardware_joint_states | 与MoveIt期望的/joint_states不一致 |
  | 无统一任务状态机 | 难以协调多节点工作流程 |

  ---

  ## 3. 海康威视工业相机集成技术栈

  ### 3.1 相机驱动层
  技术选择:
  ├── 方案A: MVS SDK (海康威视官方)
  │   ├── 优点: 功能完整，支持所有相机参数
  │   └── 缺点: 需要适配ROS2接口
  │
  ├── 方案B: hikrobot_camera (开源ROS2驱动)
  │   ├── 优点: 开箱即用
  │   └── 缺点: 可能功能不全
  │
  └── 方案C: 基于GenICam标准驱动
      └── 适用于多品牌相机

  推荐: 方案A (MVS SDK) + 自行封装ROS2节点

  ### 3.2 图像处理与神经网络推理
  图像预处理:
  └── OpenCV 4.x (cv_bridge转换)

  神经网络框架 (选其一):
  ├── ONNX Runtime [推荐] - 跨平台，部署简单
  ├── TensorRT - NVIDIA GPU加速，延迟最低
  ├── OpenVINO - Intel CPU优化
  └── TFLite/PyTorch Mobile - 边缘设备

  目标检测模型:
  ├── YOLOv8/YOLOv9 [推荐] - 实时性好，精度高
  ├── RT-DETR - Transformer架构，无NMS
  └── 自定义模型 - 根据物体特性训练

  数据格式:
  ├── sensor_msgs/Image - 原始图像
  ├── vision_msgs/Detection2DArray - 检测结果
  └── geometry_msgs/PoseStamped - 3D位姿

  ### 3.3 3D感知与定位
  手眼标定:
  ├── Eye-in-hand (相机装末端)
  │   └── 标定: AX=XB问题
  └── Eye-to-hand (相机固定外部)
      └── 标定: AX=ZB问题

  标定工具:
  ├── easy_handeye2 [推荐] - ROS2专用
  ├── OpenCV solvePnP - 手动实现
  └── VISP - 专业视觉伺服库

  深度获取 (2D相机):
  ├── 结构光/投影仪辅助
  ├── 单目深度估计网络 (MiDaS/DPT)
  └── 已知物体尺寸 + PnP

  坐标变换:
  └── tf2_ros - 维护camera->base_link变换

  ### 3.4 抓取规划
  抓取姿态生成:
  ├── 规则方法 - 基于物体几何 (简单物体)
  ├── GraspIt! - 抓取仿真与评分
  └── 神经网络 - GraspNet/AnyGrasp (复杂场景)

  MoveIt2集成:
  ├── moveit_task_constructor - 任务级规划
  └── 自定义抓取流程 - Approach → Grasp → Retreat

  ---

  ## 4. 需要实现的功能模块

  ### 4.1 相机驱动节点 (camera_driver_node)
  ```cpp
  // 功能: 连接海康威视相机，发布图像话题
  // 文件: ARV_V1_MOVEIT/src/camera_driver_node.cpp

  发布话题:
  ├── /camera/image_raw (sensor_msgs/Image) - 原始图像
  ├── /camera/camera_info (sensor_msgs/CameraInfo) - 相机内参
  └── /camera/image_rect (sensor_msgs/Image) - 去畸变图像 [可选]

  服务:
  ├── /camera/set_exposure
  ├── /camera/set_gain
  └── /camera/trigger [触发模式]

  参数:
  ├── camera_ip / serial_number
  ├── exposure_time
  ├── gain
  └── frame_rate

  4.2 目标检测节点 (object_detection_node)

  // 功能: 运行神经网络推理，检测目标物体
  // 文件: ARV_V1_MOVEIT/src/object_detection_node.cpp

  订阅:
  └── /camera/image_raw

  发布:
  ├── /detection/results (vision_msgs/Detection2DArray)
  ├── /detection/annotated_image (sensor_msgs/Image) - 可视化
  └── /detection/masks (sensor_msgs/Image) [实例分割时]

  参数:
  ├── model_path - ONNX模型路径
  ├── confidence_threshold
  ├── nms_threshold
  └── class_names

  4.3 位姿估计节点 (pose_estimation_node)

  // 功能: 将2D检测结果转换为3D位姿
  // 文件: ARV_V1_MOVEIT/src/pose_estimation_node.cpp

  订阅:
  ├── /detection/results
  ├── /camera/image_raw
  └── /camera/camera_info

  发布:
  └── /object_poses (geometry_msgs/PoseArray)

  依赖:
  └── 手眼标定TF (camera_link -> base_link)

  方法:
  ├── PnP求解 (已知物体3D模型)
  ├── 深度估计 + 检测中心
  └── ArUco/AprilTag标记 (物体上贴标记)

  4.4 夹爪控制节点 (gripper_controller_node)

  // 功能: 控制夹爪开合
  // 文件: ARV_V1_MOVEIT/src/gripper_controller_node.cpp

  订阅:
  └── /gripper/command (std_msgs/Float64) - 开合度 0~1

  发布:
  └── /gripper/state (sensor_msgs/JointState)

  Action:
  └── /gripper/grasp (control_msgs/GripperCommand)

  硬件接口:
  └── 扩展serial_protocol.hpp添加夹爪控制命令

  4.5 抓取规划节点 (grasp_planner_node)

  // 功能: 根据物体位姿生成抓取方案
  // 文件: ARV_V1_MOVEIT/src/grasp_planner_node.cpp

  服务:
  └── /plan_grasp
      输入: geometry_msgs/PoseStamped (物体位姿)
      输出: 抓取姿态序列 (approach, grasp, retreat)

  Action:
  └── /execute_grasp - 执行完整抓取流程

  功能:
  ├── 计算抓取姿态 (基于物体朝向)
  ├── 生成预抓取路径 (approach)
  ├── 调用MoveIt规划
  └── 协调夹爪动作

  4.6 任务协调节点 (task_coordinator_node)

  // 功能: 状态机管理整个抓取流程
  // 文件: ARV_V1_MOVEIT/src/task_coordinator_node.cpp

  状态机:
  IDLE → DETECTING → PLANNING → APPROACHING → GRASPING → LIFTING → PLACING → IDLE

  服务:
  └── /pick_and_place
      输入: 目标物体类别, 放置位置
      输出: 执行结果

  --- ---
  5. URDF/SRDF修改建议

  5.1 添加夹爪到URDF

  <!-- 在ARV_V1_MODEL.urdf末尾，</robot>之前添加 -->

  <!-- 工具中心点 (TCP) -->
  <link name="tool0">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="tool0_joint" type="fixed">
    <parent link="link6_2006roll"/>
    <child link="tool0"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>  <!-- 根据实际夹爪长度调整 -->
  </joint>

  <!-- 夹爪基座 -->
  <link name="gripper_base">
    <inertial>
      <mass value="0.15"/>
      <origin xyz="0 0 0.02"/>
      <inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.02"/>
      <geometry><cylinder radius="0.02" length="0.04"/></geometry>
      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.02"/>
      <geometry><cylinder radius="0.02" length="0.04"/></geometry>
    </collision>
  </link>

  <joint name="gripper_base_joint" type="fixed">
    <parent link="link6_2006roll"/>
    <child link="gripper_base"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- 左手指 -->
  <link name="gripper_finger_left">
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry><box size="0.01 0.02 0.05"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><box size="0.01 0.02 0.05"/></geometry>
    </collision>
  </link>

  <joint name="gripper_finger_left_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="gripper_finger_left"/>
    <origin xyz="0 0.015 0.065" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.025" effort="10" velocity="0.1"/>
  </joint>

  <!-- 右手指 -->
  <link name="gripper_finger_right">
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry><box size="0.01 0.02 0.05"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><box size="0.01 0.02 0.05"/></geometry>
    </collision>
  </link>

  <joint name="gripper_finger_right_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="gripper_finger_right"/>
    <origin xyz="0 -0.015 0.065" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0" upper="0.025" effort="10" velocity="0.1"/>
    <mimic joint="gripper_finger_left_joint" multiplier="1"/>
  </joint>

  5.2 修改SRDF添加末端执行器组

  <!-- 在ARV_V1_MODEL.srdf中添加 -->

  <!-- 夹爪规划组 -->
  <group name="gripper">
    <joint name="gripper_finger_left_joint"/>
  </group>

  <!-- 定义末端执行器 -->
  <end_effector name="gripper" parent_link="tool0" group="gripper" parent_group="ARM"/>

  <!-- 夹爪预定义状态 -->
  <group_state name="open" group="gripper">
    <joint name="gripper_finger_left_joint" value="0.025"/>
  </group_state>

  <group_state name="close" group="gripper">
    <joint name="gripper_finger_left_joint" value="0.0"/>
  </group_state>

  <!-- 更新碰撞矩阵 -->
  <disable_collisions link1="link6_2006roll" link2="gripper_base" reason="Adjacent"/>
  <disable_collisions link1="gripper_base" link2="gripper_finger_left" reason="Adjacent"/>
  <disable_collisions link1="gripper_base" link2="gripper_finger_right" reason="Adjacent"/>

  5.3 添加相机坐标系 (Eye-in-hand配置)

  <!-- 在URDF中添加相机link -->
  <link name="camera_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry><box size="0.03 0.03 0.02"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="link6_2006roll"/>  <!-- 或gripper_base -->
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.02" rpy="0 1.57 0"/>  <!-- 朝下看，根据实际安装调整 -->
  </joint>

  <!-- 相机光学坐标系 -->
  <link name="camera_optical_frame"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>  <!-- ROS光学坐标系转换 -->
  </joint>

  ---
  6. 工作量估算

  6.1 硬件准备 (1-2周)

  | 任务        | 工时  | 说明                                         |
  |-----------|-----|--------------------------------------------|
  | 海康相机选型与采购 | 2天  | 推荐: MV-CE060-10UC (600万像素) 或 MV-CS050-10UC |
  | 电动夹爪选型与采购 | 2天  | 推荐: 大寰PGC/AGC系列，或因时机器人RG2                  |
  | 相机支架设计与安装 | 2天  | 3D打印/铝型材，Eye-in-hand需考虑重量                  |
  | 夹爪机械适配    | 3天  | 设计法兰连接件，加工或3D打印                            |
  | 电气接线与调试   | 2天  | 相机网线，夹爪控制线（RS485/PWM/IO）                   |

  6.2 驱动与底层开发 (2-3周)

  | 任务           | 工时  | 难度  | 依赖                  |
  |--------------|-----|-----|---------------------|
  | URDF夹爪建模     | 2天  | 低   | -                   |
  | SRDF末端执行器配置  | 1天  | 低   | URDF                |
  | 海康相机ROS2驱动开发 | 4天  | 中   | MVS SDK             |
  | 相机内参标定       | 1天  | 低   | 标定板                 |
  | 夹爪控制节点开发     | 3天  | 中   | -                   |
  | 串口协议扩展(夹爪)   | 2天  | 中   | serial_protocol.hpp |
  | MuJoCo夹爪仿真适配 | 2天  | 中   | -                   |

  6.3 视觉感知开发 (3-4周)

  | 任务          | 工时  | 难度    | 说明            |
  |-------------|-----|-------|---------------|
  | 数据采集        | 3天  | 低     | 拍摄目标物体图像      |
  | 数据标注        | 3天  | 低(耗时) | labelImg/CVAT |
  | YOLOv8模型训练  | 2天  | 中     | ultralytics库  |
  | ONNX模型导出与优化 | 1天  | 中     | -             |
  | 检测节点开发      | 3天  | 中     | -             |
  | 手眼标定实现      | 4天  | 高     | easy_handeye2 |
  | 位姿估计节点开发    | 5天  | 高     | -             |
  | 视觉系统测试优化    | 4天  | -     | -             |

  6.4 抓取规划与执行 (2-3周)

  | 任务           | 工时  | 难度  | 说明                     |
  |--------------|-----|-----|------------------------|
  | 抓取姿态生成算法     | 4天  | 中   | 基于物体位姿计算抓取角度           |
  | MoveIt抓取流程集成 | 3天  | 中   | approach/grasp/retreat |
  | 碰撞场景动态管理     | 2天  | 低   | PlanningScene          |
  | 任务状态机开发      | 3天  | 中   | -                      |
  | 端到端联调        | 4天  | -   | -                      |
  | 异常处理与恢复      | 2天  | 中   | 抓取失败重试等                |

  6.5 总工时汇总

  | 阶段    | 工时    | 备注           |
  |-------|-------|--------------|
  | 硬件准备  | 1-2周  | 部分可并行（采购等待期） |
  | 驱动与底层 | 2-3周  |              |
  | 视觉感知  | 3-4周  | 核心工作量        |
  | 抓取规划  | 2-3周  |              |
  | 总计    | 8-12周 | 单人全职，有ROS2经验 |

  ---
  7. 技术风险与建议

  7.1 高风险项

  | 风险       | 影响           | 缓解措施                   |
  |----------|--------------|------------------------|
  | 手眼标定精度不足 | 抓取位置偏移，成功率低  | 多点标定，在线校正，使用AprilTag辅助 |
  | 神经网络泛化性差 | 新物体/光照变化识别失败 | 数据增强，域随机化，持续收集数据       |
  | 实时性不满足   | 物体移动时跟踪滞后    | GPU加速推理，降低分辨率，优化流水线    |
  | 夹爪力控不当   | 物体滑落或损坏      | 力传感器反馈，力矩限制，软夹爪        |

  7.2 建议的实施顺序

  Phase 1 (2周): 夹爪集成 + 固定位置抓取
  ├── URDF/SRDF添加夹爪
  ├── 夹爪控制节点
  ├── MuJoCo夹爪仿真
  └── 验证: 在已知坐标处抓取物体

  Phase 2 (3周): 相机驱动 + 手眼标定
  ├── 海康相机驱动节点
  ├── 相机内参标定
  ├── 手眼标定 (Eye-in-hand)
  └── 验证: 点击图像，机械臂移动到对应位置

  Phase 3 (3周): 目标检测 + 位姿估计
  ├── 数据采集与标注
  ├── YOLOv8训练与部署
  ├── 位姿估计节点
  └── 验证: 检测物体并显示3D位姿

  Phase 4 (2周): 端到端抓取系统
  ├── 抓取规划节点
  ├── 任务状态机
  ├── 系统集成
  └── 验证: 完整识别-规划-抓取-放置流程

  ---
  8. 需要用户确认的问题

  在开始实施前，请明确以下选项:

  8.1 硬件配置

  1. 相机安装方式:
    - Eye-in-hand (相机装在末端) - 精度高，视野随末端移动
    - Eye-to-hand (相机固定外部) - 视野固定，不增加末端负载
  2. 夹爪类型:
    - 二指平行夹爪 (简单，适合规则物体)
    - 三指/多指夹爪 (适应性强，复杂)
    - 吸盘 (适合平面物体)
  3. 是否需要深度相机:
    - 仅2D相机 (需已知物体尺寸或结构光辅助)
    - RGB-D相机 (如RealSense，直接获取深度)

  8.2 目标物体

  4. 需要识别的物体类型:
    - 物体名称/类别: _______________
    - 物体尺寸范围: _______________
    - 是否为规则形状: _______________
  5. 抓取精度要求:
    - ±5mm (一般要求)
    - ±2mm (高精度)
    - ±1mm (超高精度，需高端硬件)

  8.3 部署环境 ---
  8.3 部署环境

  6. 推理硬件:
    - x86 CPU (Intel i5以上)
    - NVIDIA GPU (推荐RTX 2060以上)
    - 边缘设备 (Jetson Orin/Xavier)
  7. 实时性要求:
    - <100ms (高速抓取，运动物体)
    - <500ms (一般应用)
    - <1s (低速/静态场景)

  ---
  9. 需要新增的文件清单

  9.1 源代码文件

  ARV_V1_MOVEIT/src/
  ├── camera_driver_node.cpp      # 海康相机驱动 (~400行)
  ├── object_detection_node.cpp   # YOLO目标检测 (~500行)
  ├── pose_estimation_node.cpp    # 3D位姿估计 (~400行)
  ├── gripper_controller_node.cpp # 夹爪控制 (~300行)
  ├── grasp_planner_node.cpp      # 抓取规划 (~600行)
  ├── task_coordinator_node.cpp   # 任务状态机 (~500行)
  └── hand_eye_calibration.cpp    # 手眼标定工具 (~300行)

  9.2 配置文件

  ARV_V1_MOVEIT/config/
  ├── camera_params.yaml          # 相机参数 (曝光/增益/帧率)
  ├── camera_intrinsics.yaml      # 相机内参 (标定结果)
  ├── hand_eye_calibration.yaml   # 手眼标定结果
  ├── detection_params.yaml       # 检测参数 (阈值/类别)
  └── grasp_params.yaml           # 抓取参数 (approach距离等)

  9.3 启动文件

  ARV_V1_MOVEIT/launch/
  ├── camera.launch.py            # 启动相机驱动
  ├── vision_pipeline.launch.py   # 启动视觉流水线
  ├── grasp_system.launch.py      # 启动完整抓取系统
  └── calibration.launch.py       # 手眼标定启动

  9.4 模型文件

  ARV_V1_MOVEIT/models/
  ├── yolov8_custom.onnx          # 训练好的检测模型
  └── class_names.txt             # 类别名称列表

  9.5 需要修改的现有文件

  | 文件                                          | 修改内容                                       |
  |---------------------------------------------|--------------------------------------------|
  | ARV_V1_MODEL/urdf/ARV_V1_MODEL.urdf         | 添加gripper links/joints, tool0, camera_link |
  | ARV_V1_MOVEIT/config/ARV_V1_MODEL.srdf      | 添加gripper组, end_effector定义                 |
  | ARV_V1_MOVEIT/CMakeLists.txt                | 添加新节点编译, 添加OpenCV/ONNX依赖                   |
  | ARV_V1_MOVEIT/package.xml                   | 添加vision_msgs, cv_bridge等依赖                |
  | ARV_V1_MOVEIT/src/serial_protocol.hpp       | 添加夹爪控制命令帧定义                                |
  | ARV_V1_MOVEIT/src/mujoco_interface_node.cpp | 添加夹爪关节仿真                                   |

  ---
  10. CMakeLists.txt修改示例

  # 在现有CMakeLists.txt中添加

  # ============= 新增依赖 =============
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(vision_msgs REQUIRED)

  # ONNX Runtime (手动配置)
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_PATH}")
  set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_ROOT}/include")
  set(ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

  # ============= 相机驱动节点 =============
  add_executable(camera_driver_node src/camera_driver_node.cpp)
  target_link_libraries(camera_driver_node
    ${OpenCV_LIBS}
    MvCameraControl  # 海康MVS SDK
  )
  ament_target_dependencies(camera_driver_node
    rclcpp sensor_msgs cv_bridge image_transport
  )

  # ============= 目标检测节点 =============
  add_executable(object_detection_node src/object_detection_node.cpp)
  target_include_directories(object_detection_node PUBLIC ${ONNXRUNTIME_INCLUDE_DIRS})
  target_link_libraries(object_detection_node
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIBRARIES}
  )
  ament_target_dependencies(object_detection_node
    rclcpp sensor_msgs vision_msgs cv_bridge
  )

  # ============= 夹爪控制节点 =============
  add_executable(gripper_controller_node src/gripper_controller_node.cpp)
  ament_target_dependencies(gripper_controller_node
    rclcpp std_msgs sensor_msgs control_msgs
  )

  # ============= 安装 =============
  install(TARGETS
    camera_driver_node
    object_detection_node
    gripper_controller_node
    # ... 其他节点
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

  ---
  11. 系统话题与接口总览

  11.1 话题列表 (新增)

  /camera/image_raw           sensor_msgs/Image           相机原始图像
  /camera/camera_info         sensor_msgs/CameraInfo      相机内参
  /detection/results          vision_msgs/Detection2DArray 检测结果
  /detection/annotated_image  sensor_msgs/Image           标注图像(可视化)
  /object_poses               geometry_msgs/PoseArray     物体3D位姿
  /gripper/command            std_msgs/Float64            夹爪开合指令
  /gripper/state              sensor_msgs/JointState      夹爪状态

  11.2 服务列表 (新增)

  /camera/set_exposure        std_srvs/SetFloat64         设置曝光
  /plan_grasp                 custom_msgs/PlanGrasp       规划抓取
  /execute_pick_place         custom_msgs/PickPlace       执行抓取放置

  11.3 Action列表 (新增)

  /gripper/grasp              control_msgs/GripperCommand 夹爪动作
  /pick_and_place             custom_msgs/PickAndPlace    完整抓取流程

  11.4 TF坐标系 (新增)

  world
  └── base_link
      └── ... (现有链接)
          └── link6_2006roll
              ├── tool0              # TCP
              ├── gripper_base
              │   ├── gripper_finger_left
              │   └── gripper_finger_right
              └── camera_link
                  └── camera_optical_frame

  ---
  12. 总结

  12.1 项目现状评价

  当前ARV_V1项目已完成机械臂运动控制的核心功能:
  - URDF/SRDF建模完整
  - MoveIt2轨迹规划可用
  - 力矩控制 (前馈+PD/级联PID) 实现
  - MuJoCo仿真环境搭建完成
  - Sim2Real硬件接口就绪

  12.2 视觉抓取的主要差距

  | 缺失模块      | 优先级 | 工作量  |
  |-----------|-----|------|
  | 末端执行器(夹爪) | P0  | 1周   |
  | 相机驱动      | P0  | 1周   |
  | 目标检测      | P0  | 2周   |
  | 手眼标定      | P0  | 1周   |
  | 位姿估计      | P1  | 1.5周 |
  | 抓取规划      | P1  | 1.5周 |
  | 任务协调      | P2  | 1周   |

  12.3 建议的技术选型

  | 模块   | 推荐方案                       |
  |------|----------------------------|
  | 相机   | 海康MV-CE060-10UC + MVS SDK  |
  | 夹爪   | 大寰PGC系列电动夹爪                |
  | 检测模型 | YOLOv8n/s (ONNX Runtime部署) |
  | 手眼标定 | easy_handeye2              |
  | 位姿估计 | PnP (已知物体) / 深度相机          |

  12.4 预计总工期

  8-12周 (单人全职，有ROS2/机器视觉经验)

  ---
  文档生成日期: 2025-12-29
  项目: ARV_V1 6-DOF机械臂
  评审目的: 海康威视工业相机集成与物体识别夹取功能可行性分析
