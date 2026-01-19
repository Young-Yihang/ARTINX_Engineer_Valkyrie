# ARV_V1 比赛快速参考手册

> 🏆 **一页纸掌握关键信息 - 比赛现场必备**

## 🚀 快速启动 (30秒)

```bash
# 选项1: 自动启动（推荐）
cd ~/ros2_ws/src && ./start_mujoco_system.sh
# 选择: [2] Hardware + Digital Twin

# 选项2: 手动启动
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ARV_V1_MOVEIT torque_controller_node &
ros2 run ARV_V1_MOVEIT hardware_interface_node --ros-args -p serial_port:=/dev/ttyACM0 &
```

## ⚡ 关键命令速查

### 状态检查
```bash
# 节点是否运行
ros2 node list | grep -E "(torque|hardware)"

# 频率是否正常 (应显示200Hz)
ros2 topic hz /joint_states

# 查看当前关节状态
ros2 topic echo /joint_states --once
```

### 紧急操作
```bash
# 🔴 紧急停止
ros2 param set /hardware_interface force_zero_torque true

# 🟡 重启硬件接口
pkill -f hardware_interface && sleep 1
ros2 run ARV_V1_MOVEIT hardware_interface_node &

# 🟢 恢复正常
ros2 param set /hardware_interface force_zero_torque false
```

## 🔧 常见问题快速处理

### 1. 串口连接失败
```bash
# 检查设备
ls /dev/ttyACM* /dev/ttyUSB*

# 修复权限
sudo chmod 666 /dev/ttyACM0

# 指定端口重启
ros2 run ARV_V1_MOVEIT hardware_interface_node \
  --ros-args -p serial_port:=/dev/ttyUSB0
```

### 2. 机械臂抖动/振荡
```bash
# 立即降低所有增益50%
for i in {1..6}; do
  ros2 param set /torque_controller_action_server \
    cascade_pid.joint_${i}.vel_Kp 3.5
done

# 增强滤波
ros2 param set /torque_controller_action_server \
  kalman.Q_vel 1e-5
```

### 3. 跟踪误差过大
```bash
# 增加积分增益
for i in {1..6}; do
  ros2 param set /torque_controller_action_server \
    cascade_pid.joint_${i}.vel_Ki 1.0
done
```

### 4. 控制频率异常
```bash
# 检查CPU占用
top -p $(pgrep torque_controller)

# 降低日志级别减少开销
ros2 param set /torque_controller_action_server \
  log_level WARN
```

## 📊 关键参数表

| 参数类型 | 正常值 | 保守值 | 激进值 |
|---------|--------|--------|--------|
| **位置P增益** | 3-40 | 2-30 | 5-50 |
| **速度P增益** | 7-10 | 5-7 | 10-15 |
| **速度I增益** | 0.5-1.0 | 0.3 | 1.5 |
| **卡尔曼Q_vel** | 1e-4 | 1e-5 | 1e-3 |
| **力矩限制** | 1-20 N·m | 0.8-15 | 1.2-25 |

## 🎯 性能指标基准

| 指标 | 优秀 | 合格 | 告警 |
|------|------|------|------|
| 控制频率 | 200±1 Hz | 195-205 Hz | <190 Hz |
| 延迟 | <3ms | 3-5ms | >5ms |
| 位置误差 | <0.02 rad | <0.05 rad | >0.1 rad |
| CPU占用 | <60% | 60-80% | >80% |
| 丢包率 | <0.01% | <0.1% | >1% |

## 🛠️ 调试工具

### 实时监控
```bash
# 启动监控面板
python3 ~/ros2_ws/src/scripts/record_metrics.py -d 10

# 查看诊断信息
ros2 topic echo /diagnostics
```

### 参数调优
```bash
# 交互式PID调优
python3 ~/ros2_ws/src/scripts/tune_pd.py

# 卡尔曼滤波器调优
python3 ~/ros2_ws/src/scripts/tune_kalman.py
```

## 📝 比赛前检查清单

- [ ] USB线缆牢固（建议热胶固定）
- [ ] 串口权限设置正确
- [ ] 所有节点启动无错误
- [ ] 控制频率稳定200Hz
- [ ] 紧急停止测试通过
- [ ] 参数文件备份完成
- [ ] 监控脚本就绪

## 🔴 故障决策树

```
机械臂异常？
├─ 完全不动？
│  ├─ 检查串口连接 → ls /dev/tty*
│  ├─ 检查节点状态 → ros2 node list
│  └─ 重启硬件接口 → pkill + restart
│
├─ 抖动/振荡？
│  ├─ 降低增益50% → 修改vel_Kp
│  ├─ 增强滤波 → 降低Q_vel
│  └─ 检查机械松动
│
├─ 响应慢/误差大？
│  ├─ 增加P增益 → pos_Kp × 1.2
│  ├─ 增加I增益 → vel_Ki × 1.5
│  └─ 检查负载变化
│
└─ 间歇性故障？
   ├─ 查看日志 → ros2 topic echo /rosout
   ├─ 检查CRC错误率
   └─ 更换USB线缆
```

## 💡 专家提示

1. **黄金参数组合**（大多数情况适用）:
   ```bash
   Kp_pos=5, Kp_vel=7, Ki_vel=0.5, Q_vel=1e-4
   ```

2. **快速恢复三步法**:
   - 步骤1: 紧急停止 → `force_zero_torque true`
   - 步骤2: 重启节点 → `pkill + restart`
   - 步骤3: 加载保守参数 → `tune_pd.py → option 2`

3. **性能优化优先级**:
   - 高: 修复串口超时问题
   - 中: 降低内存分配频率
   - 低: SIMD优化

## 📞 紧急联系

- 主程序员: Young-Yihang
- ROS2专家: [团队成员]
- 硬件负责: [团队成员]

---

**最后更新**: 2026-01-19 | **版本**: 2.0 | **分支**: feature/ros2_components

> 打印此页，比赛带上！祝比赛顺利！🏆