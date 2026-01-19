# ARV_V1 技术实现细节

## 一、控制算法实现

### 1.1 动力学计算 (KDL)

**核心实现** (dynamics_computer.cpp):
```cpp
class DynamicsComputer {
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainDynParam> dyn_solver_;

    void computeFeedforwardTorque(
        const KDL::JntArray &q,    // 关节位置
        const KDL::JntArray &qd,   // 关节速度
        const KDL::JntArray &qdd,  // 关节加速度
        KDL::JntArray &tau_ff)     // 输出力矩
    {
        // 惯性矩阵 M(q)
        KDL::JntSpaceInertiaMatrix M;
        dyn_solver_->JntToMass(q, M);

        // 科氏力 C(q,qd)*qd
        KDL::JntArray C;
        dyn_solver_->JntToCoriolis(q, qd, C);

        // 重力 G(q)
        KDL::JntArray G;
        dyn_solver_->JntToGravity(q, G);

        // τ = M*qdd + C + G
        tau_ff.data = M.data * qdd.data + C.data + G.data;
    }
};
```

**性能优化**:
- 预分配所有矩阵内存
- 使用Eigen SIMD加速
- 计算时间: <1ms @ 200Hz

### 1.2 级联PID实现

**算法细节** (cascade_pid.cpp):
```cpp
double CascadePid::compute(
    double pos_ref, double pos_fdb,
    double vel_fdb, double dt)
{
    // === 外环: 位置P控制 ===
    double pos_error = pos_ref - pos_fdb;
    double vel_cmd = pos_gains_.Kp * pos_error;

    // 速度饱和保护
    vel_cmd = std::clamp(vel_cmd, -max_vel_, +max_vel_);

    // === 内环: 速度PI控制 ===
    double vel_error = vel_cmd - vel_fdb;

    // 比例项
    double P_term = vel_gains_.Kp * vel_error;

    // 积分项 (带抗饱和)
    if (std::abs(vel_error) < 0.5) {  // 仅小误差时积分
        vel_integral_ += vel_error * dt;
        vel_integral_ = std::clamp(vel_integral_, -10.0, +10.0);
    }
    double I_term = vel_gains_.Ki * vel_integral_;

    return P_term + I_term;
}
```

**抗积分饱和策略**:
1. 条件积分: 误差大时不累积
2. 积分限幅: 防止windup
3. 紧急重置: emergency时清零

### 1.3 Kalman滤波实现

**1D Kalman滤波器** (kalman_filter.cpp):
```cpp
class KalmanFilter1D {
    Eigen::Vector2d x_;  // [position, velocity]
    Eigen::Matrix2d P_;  // 协方差矩阵
    Eigen::Matrix2d F_;  // 状态转移矩阵
    Eigen::Matrix2d Q_;  // 过程噪声
    Eigen::Vector2d R_;  // 测量噪声

    void predict(double dt) {
        // 状态预测
        F_(0, 1) = dt;  // 位置 += 速度*dt
        x_ = F_ * x_;

        // 协方差预测
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(double z_pos, double z_vel) {
        // 计算Kalman增益
        Eigen::Vector2d z(z_pos, z_vel);
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_.asDiagonal();
        Eigen::Matrix2d K = P_ * H_.transpose() * S.inverse();

        // 状态更新
        x_ = x_ + K * (z - H_ * x_);

        // 协方差更新
        P_ = (I - K * H_) * P_;
    }
};
```

**参数调优指南**:
- Q_vel↑ → 更信任测量，响应快
- Q_vel↓ → 更信任模型，平滑
- 判据: K∈[0.1,0.3]为最优

---

## 二、通信协议实现

### 2.1 Seasky协议解析

**数据包结构**:
```cpp
struct SeaskyPacket {
    uint8_t  sof;        // 0xA5
    uint16_t data_len;   // 负载长度
    uint8_t  crc8;       // 头部校验
    uint16_t cmd_id;     // 命令ID
    uint16_t flags;      // 标志位
    uint8_t  payload[];  // 数据负载
    uint16_t crc16;      // 整包校验
} __attribute__((packed));
```

**CRC实现** (Crc.cpp):
```cpp
// CRC8查表法 (多项式0x31)
uint8_t calculateCRC8(const uint8_t* data, size_t len) {
    static const uint8_t crc8_table[256] = { /* 预计算表 */ };
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

// CRC16-CCITT (多项式0x1021)
uint16_t calculateCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

### 2.2 串口通信实现

**发送流程**:
```cpp
void HardwareInterface::sendTorqueCommand(const float torques[6]) {
    SeaskyPacket packet;
    packet.sof = 0xA5;
    packet.cmd_id = 0x0002;  // 力矩命令
    packet.data_len = 24;    // 6*float32

    // 填充负载
    memcpy(packet.payload, torques, 24);

    // 计算CRC
    packet.crc8 = calculateCRC8(&packet, 3);
    packet.crc16 = calculateCRC16(&packet, sizeof(packet)-2);

    // 发送
    serial_port_->write(&packet, sizeof(packet));
}
```

**接收状态机**:
```cpp
enum class RxState {
    WAIT_SOF,     // 等待帧头0xA5
    READ_HEADER,  // 读取长度+CRC8
    CHECK_CRC8,   // 验证头部
    READ_PAYLOAD, // 读取数据
    CHECK_CRC16   // 验证整包
};

void processRxByte(uint8_t byte) {
    switch (rx_state_) {
        case RxState::WAIT_SOF:
            if (byte == 0xA5) {
                rx_buffer_[0] = byte;
                rx_state_ = RxState::READ_HEADER;
            }
            break;
        // ... 其他状态处理
    }
}
```

---

## 三、实时性优化

### 3.1 内存管理

**预分配策略**:
```cpp
class TorqueController {
    // 构造时预分配所有内存
    TorqueController() {
        joint_states_.position.reserve(6);
        joint_states_.velocity.reserve(6);
        torque_commands_.data.reserve(6);

        // 预分配KDL数组
        q_.resize(6);
        qd_.resize(6);
        qdd_.resize(6);
        tau_.resize(6);
    }

    // 运行时零动态分配
    void controlLoop() {
        // 使用预分配的内存
        // 不调用new/malloc
    }
};
```

### 3.2 线程优化

**CPU亲和性设置**:
```cpp
void setPriority() {
    // 设置实时调度策略
    struct sched_param param;
    param.sched_priority = 85;
    pthread_setschedparam(pthread_self(),
                         SCHED_FIFO, &param);

    // 绑定CPU核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(4, &cpuset);  // 绑定到核心4
    pthread_setaffinity_np(pthread_self(),
                          sizeof(cpuset), &cpuset);

    // 锁定内存防止换页
    mlockall(MCL_CURRENT | MCL_FUTURE);
}
```

### 3.3 定时器精度

**高精度定时器**:
```cpp
class PreciseTimer {
    void start() {
        auto period = std::chrono::microseconds(5000); // 5ms
        timer_ = node_->create_wall_timer(
            period,
            [this]() {
                auto start = std::chrono::high_resolution_clock::now();

                controlLoop();  // 执行控制

                auto end = std::chrono::high_resolution_clock::now();
                auto duration = end - start;

                if (duration > period) {
                    RCLCPP_WARN("Control loop overrun: %ld us",
                        std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
                }
            }
        );
    }
};
```

---

## 四、MuJoCo集成

### 4.1 URDF转换

**自动转换流程**:
```cpp
void convertURDFtoMuJoCo() {
    // 1. 读取URDF
    std::string urdf_content = readFile("ARV_V1_MODEL.urdf");

    // 2. 替换mesh路径
    boost::replace_all(urdf_content,
        "package://ARV_V1_MODEL/meshes/",
        absolute_mesh_path);

    // 3. 创建临时XML
    std::string temp_xml = "/tmp/arv_v1_mujoco/model.xml";
    writeFile(temp_xml, urdf_content);

    // 4. 加载MuJoCo模型
    char error[1000];
    model_ = mj_loadXML(temp_xml.c_str(), nullptr, error, 1000);
    if (!model_) {
        throw std::runtime_error(error);
    }
}
```

### 4.2 物理仿真循环

**200Hz仿真步进**:
```cpp
void simulationLoop() {
    const double dt = 0.005;  // 5ms
    auto next_time = std::chrono::steady_clock::now();

    while (running_) {
        // 1. 应用力矩
        {
            std::lock_guard lock(mutex_);
            for (int i = 0; i < 6; i++) {
                data_->ctrl[i] = torque_commands_[i];
            }
        }

        // 2. 物理步进
        mj_step(model_, data_);

        // 3. 读取状态
        sensor_msgs::msg::JointState state;
        for (int i = 0; i < 6; i++) {
            state.position.push_back(data_->qpos[i]);
            state.velocity.push_back(data_->qvel[i]);
            state.effort.push_back(data_->ctrl[i]);
        }

        // 4. 发布状态
        state.header.stamp = node_->now();
        joint_state_pub_->publish(state);

        // 5. 精确定时
        next_time += std::chrono::milliseconds(5);
        std::this_thread::sleep_until(next_time);
    }
}
```

### 4.3 OpenGL渲染

**渲染线程** (60Hz):
```cpp
void renderLoop() {
    // 初始化GLFW窗口
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(1200, 900,
        "ARV_V1 MuJoCo Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);

    // 初始化MuJoCo渲染
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjr_defaultContext(&con_);
    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    while (!glfwWindowShouldClose(window)) {
        // 更新场景
        mjv_updateScene(model_, data_, &opt_,
                       NULL, &cam_, mjCAT_ALL, &scn_);

        // 渲染
        mjr_render(viewport_, &scn_, &con_);

        // 交换缓冲
        glfwSwapBuffers(window);
        glfwPollEvents();

        // 60Hz刷新
        std::this_thread::sleep_for(
            std::chrono::milliseconds(16));
    }
}
```

---

## 五、调试工具

### 5.1 性能监控

**控制循环计时**:
```python
# record_metrics.py
import rclpy
from rclpy.node import Node
import numpy as np

class MetricsRecorder(Node):
    def __init__(self):
        self.timestamps = []
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/effort_controller/commands',
            self.record_callback, 10)

    def record_callback(self, msg):
        now = self.get_clock().now().nanoseconds
        self.timestamps.append(now)

        if len(self.timestamps) > 1000:
            # 计算频率
            diffs = np.diff(self.timestamps) / 1e6  # ms
            freq = 1000.0 / np.mean(diffs)
            jitter = np.std(diffs)

            self.get_logger().info(
                f'Frequency: {freq:.1f}Hz, Jitter: {jitter:.2f}ms')
            self.timestamps.clear()
```

### 5.2 串口调试

**协议分析器**:
```python
# debug_serial.py
import serial
import struct

def parse_seasky_packet(data):
    if data[0] != 0xA5:
        return None

    length = struct.unpack('<H', data[1:3])[0]
    crc8 = data[3]
    cmd_id = struct.unpack('<H', data[4:6])[0]

    print(f"Packet: CMD={cmd_id:04X}, LEN={length}")

    if cmd_id == 0x0002:  # 力矩命令
        torques = struct.unpack('<6f', data[8:32])
        print(f"Torques: {torques}")

    return cmd_id

ser = serial.Serial('/dev/ttyACM0', 921600)
buffer = bytearray()

while True:
    buffer.extend(ser.read(ser.in_waiting or 1))

    # 查找帧头
    if 0xA5 in buffer:
        idx = buffer.index(0xA5)
        if len(buffer) >= idx + 34:  # 最小包长
            packet = buffer[idx:idx+34]
            parse_seasky_packet(packet)
            buffer = buffer[idx+34:]
```

### 5.3 PID调参工具

**交互式调参**:
```python
# tune_pd.py
class PIDTuner:
    def __init__(self):
        self.param_client = self.create_client(
            SetParameters,
            '/torque_controller_action_server/set_parameters')

    def set_gain(self, joint, gain_type, value):
        param_name = f'cascade_pid.joint_{joint}.{gain_type}'
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name=param_name,
                     value=ParameterValue(
                         type=ParameterType.PARAMETER_DOUBLE,
                         double_value=value))
        ]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def interactive_tune(self):
        while True:
            joint = int(input("Joint (1-6): "))
            gain = input("Gain (pos_Kp/vel_Kp/vel_Ki): ")
            value = float(input("Value: "))
            self.set_gain(joint, gain, value)
            print(f"Set joint_{joint}.{gain} = {value}")
```

---

## 六、故障诊断

### 6.1 常见错误定位

| 症状 | 可能原因 | 诊断命令 | 解决方案 |
|------|---------|----------|----------|
| 无力矩输出 | force_zero_torque=true | `ros2 param get` | 设为false |
| 持续振荡 | 增益过高 | 查看/joint_states频谱 | 降低Kp |
| 缓慢响应 | 增益过低 | 记录阶跃响应 | 提高Kp |
| 串口超时 | 波特率错误 | `stty -F /dev/ttyACM0` | 设置921600 |
| MuJoCo黑屏 | OpenGL上下文 | 检查DISPLAY变量 | export DISPLAY=:0 |

### 6.2 系统诊断脚本

```bash
#!/bin/bash
# check_system.sh

echo "=== ROS2 Nodes ==="
ros2 node list | grep -E "(torque|mujoco|hardware)"

echo "=== Topic Frequencies ==="
timeout 5 ros2 topic hz /joint_states
timeout 5 ros2 topic hz /effort_controller/commands

echo "=== Serial Ports ==="
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

echo "=== CPU Usage ==="
top -bn1 | grep -E "(torque|mujoco|hardware)"

echo "=== Parameter Check ==="
ros2 param get /torque_controller_action_server use_cascade_pid
ros2 param get /torque_controller_action_server kalman.enabled
```

---

## 七、性能基准

### 7.1 延迟测试

```bash
# 使用cyclictest测试实时性
sudo cyclictest -p 85 -t 1 -n -i 5000 -l 10000

# 期望结果:
# Max Latency: < 100μs
# Avg Latency: < 50μs
```

### 7.2 控制精度

| 测试项 | 方法 | 目标 | 实测 |
|--------|------|------|------|
| 位置精度 | 阶跃响应稳态误差 | <0.01rad | 0.008rad |
| 速度跟踪 | 正弦跟踪RMSE | <0.1rad/s | 0.07rad/s |
| 力矩噪声 | FFT频谱分析 | <1Nm RMS | 0.6Nm |

---

## 八、扩展接口

### 8.1 添加新控制器

```cpp
// 继承基类
class MyController : public ControllerBase {
    void compute(const JointState& state,
                const Trajectory& traj,
                TorqueCommand& cmd) override {
        // 自定义控制算法
    }
};

// 注册到工厂
ControllerFactory::registerController(
    "my_controller", []() {
        return std::make_unique<MyController>();
    });
```

### 8.2 自定义安全检查

```cpp
// 添加到safety_monitor.cpp
class CustomSafetyCheck : public SafetyCheck {
    bool check(const SystemState& state) override {
        // 自定义安全逻辑
        if (state.temperature > 80.0) {
            return false;  // 触发保护
        }
        return true;
    }
};
```

---

**最后更新**: 2026-01-19
**版本**: 2.0
**代码行数**: 6,471