# STM32 嵌入式代码架构上下文

## 项目概览

- **项目名**: Engineer_2025 (国赛工程机器人)
- **路径**: `~/artinx/engineer2025`
- **平台**: STM32F4系列微控制器
- **代码规模**: 175,760行 (三个模块总计)
- **控制频率**: 1000Hz (1ms周期)
- **状态**: 国赛已完成，代码稳定

## 项目结构

```
engineer_2025/
├── arm/                      # 机械臂控制系统 (54.8K行)
│   ├── Src/Controller/Arm/   # 核心状态机和控制器
│   │   ├── ArmKine.cpp       # 运动学算法 (426行) ★重要
│   │   ├── ArmAutoPosition.cpp # 自动取矿控制 (600+行)
│   │   └── ArmController.cpp # 状态机管理
│   ├── Src/Monitor/          # 监控和任务管理系统
│   └── Projects/RmBoardA/    # ARM主板配置
├── chassis/                  # 底盘移动控制系统 (51.8K行)
│   ├── Src/Controller/       # 底盘状态机
│   │   └── ChassisController.cpp # 麦轮底盘控制
│   └── Projects/RmBoardA/    # 底盘主板配置
└── szh/                      # 第三个子系统模块 (69.1K行)
```

## 机械臂系统架构

### 电机配置 (7自由度)

| 关节 | 电机型号 | 功能 | 控制协议 |
|------|---------|------|---------|
| Link1 | DM8009 | 大Yaw(水平转) | CAN |
| Link2 | DM8009 | 第一段臂 | CAN |
| Link3 | DM8009 | 第二段臂 | CAN |
| Link4 | LK5005 | 图传Yaw | CAN |
| Link5 | G6020 | Roll(翻滚) | CAN |
| Link6 | DM4310 | 腕Yaw | CAN |
| Link7 | M2006 | 吸盘驱动 | CAN |

### 状态机架构 (FSM)

**模板化有限状态机框架** (`StateMachine.hpp`, 75行):
```cpp
template<typename controller_type>
class StateMachine
{
    State<controller_type>* m_pCurrentState;    // 当前状态
    State<controller_type>* m_pPreviousState;   // 前一状态
    State<controller_type>* m_pGlobalState;     // 全局状态

    void ChangeState(State<controller_type>* _state);
    // 智能状态转移：执行Exit -> 转移 -> Enter
};
```

**机械臂状态 (7个离散状态)**:
1. ArmHandwithgesture - 手动遥控
2. ArmRelaxLearning - 松弛学习
3. ArmResetWrist - 腕部复位
4. ArmAutoMove - 自动单关节运动
5. ArmAutoPosition - 自动位姿规划 (6DOF IK)
6. ArmTargetMove - 目标点运动
7. ArmEscape - 紧急复位

### 运动学核心算法 (ArmKine.cpp, 426行) ★学术价值最高

**位姿解耦方案**:
```
前3关节: 位置解析法 (快速精确, <50µs)
后3关节: 姿态矩阵法 (支持四元数)
→ 将6DOF分解为3+3维子问题
```

**正向运动学(FK)**:
```cpp
// 位置FK: q[3] -> [x, y, z]
float* forward_kine_position(float link_data[3], const float link[7]);

// 姿态FK: q[3] -> [yaw, pitch, roll]
float* forward_kine_ges(float link_angle[3]);
```

**逆向运动学(IK)**:
```cpp
// 位置IK: [x, y, z] -> q[3] (解析法)
float* inverse_kine_position(float HT_Mat[4][4], const float link[7]);

// 姿态IK: [yaw, pitch, roll] -> q[3] (矩阵法)
float* inverse_kine_ges(float HT_Mat[4][4], float just_matrix[4][4]);
```

**关节参数**:
```cpp
float link_length[7] = {381.0, 416.00, 66.407, 1.0, 416.00, 32.15, 17.25}; // mm
float link_mass[4] = {8.031, 3.861, 4.586, 5.88}; // g
```

**坐标系定义**:
- X轴: 车体水平向前
- Y轴: 车体水平向左
- Z轴: 车体竖直向上
- 采用4×4齐次变换矩阵

## 底盘控制系统

### 底盘构成

| 部件 | 电机型号 | 功能 |
|------|---------|------|
| 4个驱动轮 | M3508 | 麦轮底盘驱动 |
| 1个辅助电机 | M2006 | UWB定位驱动 |

### 底盘控制架构

```cpp
class ChassisController : public ControllerEntity
{
    M3508 m_ChassisWheel[4];     // 四个麦轮
    float m_Vx, m_Vy, m_Vw;      // 底盘速度指令
    float m_VxFdb, m_VyFdb, m_VwFdb;  // 反馈速度

    void ChassisSpd2MotorSpd();   // 底盘速度->电机速度映射
    void MotorSpd2ChassisSpdFdb(); // 反馈速度解耦
};
```

### PID控制参数

**M3508速度控制**:
- Kp=3000, Ki=0, Kd=95
- 最大输出=20000

**M2006位置控制**:
- Kp=7.0, Ki=0, Kd=10.0
- 最大输出=50.0

## 通信协议系统

### 多层通信架构

```
┌─ 遥控器 (Dr16, VT13)
├─ CAN总线 (DJI官方 + DM达妙 + LK瓴控)
├─ 板间通信 (BoardPacket, CAN)
├─ 自定义控制器 (TransmitterJS, USB/UART)
└─ 裁判系统 (JudgeSystem)
```

### BoardPacket - 板间通信

**CAN ID分配**:
- Channel0: 0x120
- Channel1: 0x130

**关键数据包**:
```cpp
class ChuckPickPacket : public BoardPacket  // 取矿控制
{
    float yaw_set_left, front_set_left, rise_set_left;
    float yaw_set_right, front_set_right, rise_set_right;
};

class UIPacket : public BoardPacket         // UI反馈
{
    bool is_motor_alive[7];    // 电机在线状态
    bool isArmGasOpen;         // 吸盘状态
};

class BoardCPacket : public BoardPacket     // 相机数据
{
    float IMU_yaw, pitch, roll;
    float rvecX/Y/Z, tvecX/Y/Z;  // 视觉位姿
};
```

### TransmitterJS - 与ROS2通信

**协议格式**:
```
[SOF(0xA5)][Len(2)][序列号][CRC8][协议ID][数据][CRC16]
```

**数据帧类型**:
- 0x0302: 控制器→机器人 (30Hz)
- 0x0304: 选手端→机器人 (30Hz)
- 0x0309: 机器人→控制器 (10Hz)

## 主程序事件循环 (1ms/tick)

```cpp
void SysTick_Handler(void)  // 1ms中断
{
    Time::Tick();
    Dr16::Instance()->Update();             // 遥控器
    CanMsgDispatcher::Instance()->Update(); // CAN消息分发
    DjiCanMotorCommander::Instance()->Update();  // DJI电机
    DMCanMotorCommander::Instance()->Update();   // DM电机
    LKCanMotorCommanderSingle::Instance()->Update(); // LK电机
    BoardPacketManager::Instance()->Update();   // 板间通信
    TransmitterJS::Instance()->Update();        // ROS2通信
    WitIMU::Instance()->Update();               // IMU
    Monitor::Instance()->Update();              // 监控系统
}
```

## 学术价值分析

### 可发表的学术亮点

| 模块 | 行数 | 学术价值 | 论文适用性 |
|------|------|---------|-----------|
| **ArmKine** | 426 | 6DOF位姿解耦运动学 | ☆☆☆☆☆ 核心章节 |
| **StateMachine** | 75 | 泛型C++模板FSM | ☆☆☆☆ 设计模式 |
| **多协议电机驱动** | 1000+ | DJI/DM/LK统一接口 | ☆☆☆ 工程创新 |
| **任务管理系统** | 1500 | 层次化任务规划 | ☆☆☆ 系统设计 |

### 论文必引用文件

- `arm/Src/Controller/Arm/ArmKine.cpp` - 运动学核心
- `arm/Src/Controller/Arm/ArmAutoPosition.cpp` - 自动定位
- `arm/Src/StateMachine.hpp` - FSM框架

### 可选引用文件

- `arm/Src/TransmitterJS.hpp` - 通信协议
- `chassis/Src/Controller/Chassis/ChassisController.cpp` - 底盘控制

## 实时约束

| 要求 | 指标 |
|------|------|
| 控制循环 | 1ms (1000Hz) |
| CAN带宽 | 1Mbps |
| CPU利用率 | <70% |

## 完成度

| 模块 | 状态 | 行数 |
|------|------|------|
| 机械臂运动学 | ✅ 100% | 426 |
| 自动取矿 | ✅ 100% | 600+ |
| 状态机框架 | ✅ 100% | 75 |
| 底盘控制 | ✅ 100% | 500+ |
| CAN通信 | ✅ 100% | 1000+ |
| 板间通信 | ✅ 100% | 300+ |
| 任务管理 | ✅ 100% | 1500 |
