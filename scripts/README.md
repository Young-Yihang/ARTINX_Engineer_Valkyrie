# ARV_V1 调试工具集

## 工具清单

| 脚本 | 功能 | 依赖话题 | 使用场景 |
|------|------|---------|---------|
| `check_system.sh` | 系统链路检查 | 多个 | 启动后第一步 |
| `tune_pd.py` | PD增益调参 | 无(用ros2 param) | 调控制器增益 |
| `tune_kalman.py` | Kalman参数调参 | 无(用ros2 param) | 调滤波器 |
| `record_metrics.py` | 性能记录评估 | /joint_states, /effort_controller/commands | 量化控制效果 |
| `calibrate_gravity.py` | 重力标定 | /joint_states, /effort_controller/commands | 标定重力补偿 |

---

## 1. check_system.sh — 系统链路检查

### 做什么
启动后快速诊断整个系统是否正常。

### 检查项
```
[节点] torque_controller, move_group, mujoco/hardware_interface
[话题] /joint_states (200Hz), /effort_controller/commands (200Hz)
[TF]   base_link → link6_2006roll 链完整性
[参数] Kp/Kd, Kalman, 级联PID 是否加载
[资源] CPU/内存占用
```

### 工作原理
```bash
ros2 node list              # 检查节点存在
ros2 topic hz /joint_states # 采样2秒测频率
ros2 param list ...         # 检查参数
ros2 run tf2_ros tf2_echo   # 检查TF
```

### 使用
```bash
cd ~/ros2_ws/src
./scripts/check_system.sh
```

### 输出示例
```
[✓] 力矩控制器 (torque_controller)
[✓] 关节状态: 198.5 Hz (期望 200 Hz)
[!] Kalman滤波: 禁用
```

---

## 2. tune_pd.py — PD增益调参

### 做什么
交互式调节6个关节的 Kp/Kd 增益，无需记命令。

### 工作原理
```
不订阅任何话题！
直接调用 ros2 param get/set 命令

内部执行：
  ros2 param get /torque_controller_action_server Kp.joint_1
  ros2 param set /torque_controller_action_server Kp.joint_1 50.0
```

### 参数路径
```
/torque_controller_action_server:
  Kp.joint_1 ~ Kp.joint_6    # 位置增益
  Kd.joint_1 ~ Kd.joint_6    # 速度增益
```

### 使用
```bash
python3 scripts/tune_pd.py

# 菜单驱动：
#   [1] 查看当前增益
#   [2] 调单关节
#   [3] 批量调所有关节
#   [4] 应用预设模板
```

### 预设模板
| 模板 | Kp | Kd | 适用 |
|------|----|----|------|
| 保守 | [20,30,20,8,0.5,0.5] | [0.5,0.5,0.5,0.2,0.5,0.5] | 初次真机 |
| 标准 | [30,50,30,10,1,1] | [1,1,1,0.3,1,1] | 默认 |
| 激进 | [50,80,50,15,2,2] | [1.5,1.5,1.5,0.5,1.5,1.5] | 高精度 |

---

## 3. tune_kalman.py — Kalman滤波调参

### 做什么
调节Kalman滤波器参数，评估滤波效果。

### 工作原理
```
不订阅任何话题！
直接调用 ros2 param get/set

参数路径：
  kalman.enabled   # 开关
  kalman.Q_pos     # 位置过程噪声
  kalman.Q_vel     # 速度过程噪声 ← 主要调这个
  kalman.R_pos     # 位置测量噪声
  kalman.R_vel     # 速度测量噪声
```

### 核心原理
```
Kalman增益 K 决定滤波强度：
  K小 → 更信任预测 → 平滑但延迟
  K大 → 更信任测量 → 响应快但噪声

K 由 Q/R 比值决定：
  Q_vel 大 → K 大 → 响应快
  R_vel 大 → K 小 → 更平滑
```

### 判断标准
```
K < 0.05  → 过度平滑，增大 Q_vel
K ∈ [0.1, 0.3] → 平衡 ✓
K > 0.5   → 过度信任测量，减小 Q_vel
```

### 查看增益K
控制器每5秒打印一次：
```
=== Kalman Gain Observation (Loop #1000) ===
Joint 1: K = [[0.0012, 0.0001], [0.0001, 0.1523]]
                                       ↑
                              这个是速度增益，看这个
```

### 预设
| 模板 | Q_vel | R_vel | 效果 |
|------|-------|-------|------|
| 响应优先 | 1e-4 | 1e-2 | K大，快 |
| 平衡 | 1e-5 | 2.5e-2 | 默认 |
| 平滑优先 | 1e-6 | 5e-2 | K小，稳 |

---

## 4. record_metrics.py — 性能记录评估

### 做什么
记录一段时间的控制数据，计算性能指标。

### 订阅话题
```
/joint_states                    # 关节位置/速度 (200Hz)
/effort_controller/commands      # 力矩指令 (200Hz)
```

### 工作原理
```python
# 1. 订阅话题，缓存数据
for duration 秒:
    收集 (timestamp, position, velocity, torque)

# 2. 计算指标
RMSE_pos = sqrt(mean((pos - pos[0])^2))  # 位置稳定性
RMSE_vel = sqrt(mean(vel^2))             # 速度稳定性
饱和次数 = count(|tau| > 0.95 * LIMIT)   # 力矩饱和

# 3. 输出报告
```

### 使用
```bash
# 记录10秒
python3 scripts/record_metrics.py -d 10

# 记录并保存CSV
python3 scripts/record_metrics.py -d 10 --save
```

### 输出示例
```
  ARV_V1 控制性能报告 | 采样: 2000 | 时长: 10.0s
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
关节   RMSE位置(rad)  最大误差     RMSE速度     力矩饱和
J1     0.001234       0.003456     0.0123       0
J2     0.002345       0.005678     0.0234       3  ← 红色警告
...

综合评估:
  平均位置RMSE: 0.001823 rad (0.1044°)
  [警告] 力矩饱和 3 次，考虑降低增益
```

### 使用场景
1. 调完参数后，跑一段轨迹，量化效果
2. 对比不同参数组合的性能
3. 检测力矩饱和问题

---

## 5. calibrate_gravity.py — 重力标定

### 做什么
采集多个静止姿态的 (位置, 保持力矩)，拟合重力模型。

### 订阅话题
```
/joint_states                    # 获取当前位置
/effort_controller/commands      # 获取保持力矩
```

### 工作原理
```
物理原理：
  静止时，力矩 = 重力矩
  τ_hold = G(q) = f(sin(q), cos(q))

采集流程：
  1. 移动机械臂到姿态1，静止
  2. 记录 (q1, τ1)
  3. 移动到姿态2，静止
  4. 记录 (q2, τ2)
  ...
  N. 采集足够多姿态

分析：
  对每个关节，拟合 τ = a·sin(q) + b·cos(q) + c
  a, b 反映重力矩特性
  c 是偏置（摩擦/零漂）
```

### 使用
```bash
python3 scripts/calibrate_gravity.py

# 交互操作：
# 1. 用RViz/示教器移动机械臂到某姿态
# 2. 确保静止，按回车采集
# 3. 重复，采集5-10个姿态
# 4. 输入 done 完成并分析
```

### 输出
```
重力标定分析
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Joint 1:
  力矩范围: [-2.345, 3.456] N·m
  拟合系数: a=2.12, b=0.34, c=0.05
  拟合RMSE: 0.0823 N·m

保存文件: gravity_calib_20260113_143022.json
```

### 结果使用
1. **检查URDF准确性**：拟合系数应与URDF动力学模型接近
2. **修正前馈**：用拟合系数补偿重力计算误差
3. **诊断问题**：RMSE大说明模型不准或有其他干扰

---

## 典型工作流

### 首次真机调试
```bash
# 1. 启动系统
./start_mujoco_system.sh  # 选择串口模式

# 2. 检查链路
./scripts/check_system.sh

# 3. 用保守参数
python3 scripts/tune_pd.py
# 选择 [4] 模板 → [1] 保守

# 4. 小幅运动测试，记录性能
python3 scripts/record_metrics.py -d 10

# 5. 逐步调高增益，观察效果
```

### 重力标定流程
```bash
# 1. 确保系统运行正常
./scripts/check_system.sh

# 2. 采集多姿态数据
python3 scripts/calibrate_gravity.py
# 移动到5-10个不同姿态，每个姿态按回车采集

# 3. 分析结果
# 脚本自动输出拟合系数和RMSE
```

### Kalman调参流程
```bash
# 1. 启动系统
./start_mujoco_system.sh

# 2. 打开Kalman调参工具
python3 scripts/tune_kalman.py

# 3. 观察控制器终端的增益K打印

# 4. 根据K值调整Q_vel
#    K < 0.05 → 增大 Q_vel
#    K > 0.5  → 减小 Q_vel
```
