# ROS2 Component 实时多进程架构设计

## 一、系统概述

**平台**: Intel NUC 6核 i5/i7 + RT-PREEMPT 内核
**目标**: 硬实时控制 + 进程隔离 + 故障恢复

---

## 二、CPU核心分配策略

| Core | 用途 | 隔离 | 节点 |
|------|------|------|------|
| 0-1 | 非实时 | 否 | move_group, rviz, mujoco渲染 |
| 2-3 | 软实时 | 否 | mujoco仿真, 视觉节点 |
| 4 | 硬实时 | 是 | torque_controller + hardware_interface |
| 5 | 预留 | 是 | 预留扩展 |

**内核参数** (GRUB_CMDLINE_LINUX):
```
isolcpus=4,5 nohz_full=4,5 rcu_nocbs=4,5
```

---

## 三、Component 封装设计

### 3.1 为什么用 Component？
- **零拷贝通信**: 同执行器内节点共享内存
- **进程隔离**: 不同 Executor 独立进程，crash 不影响其他
- **灵活部署**: 可动态加载/卸载，便于调试

### 3.2 节点分组

| 组 | 进程类型 | 节点 | CPU绑定 |
|----|---------|----- |------|
| 组1 | RT进程 | torque_controller + hardware_interface | Core 4 |
| 组2 | 普通进程 | mujoco_interface | Core 2-3 |
| 组3 | 普通进程 | move_group + rviz | Core 0-1 |

### 3.3 Component 改造要点
```cpp
// 1. 移除 main() 函数
// 2. 添加注册宏
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TorqueControllerActionServer)

// 3. 生命周期管理 (可选)
// 线程在 on_activate() 启动，on_deactivate() 停止
```

---

## 四、实时性配置

### 4.1 SCHED_FIFO 优先级
| 节点 | 优先级 | 说明 |
|------|--------|------|
| torque_controller | 85 | 200Hz |
| hardware_interface | 85 | 200Hz |

### 4.2 内存锁定
```cpp
#include <sys/mman.h>
mlockall(MCL_CURRENT | MCL_FUTURE);
// 预分配内存池，避免运行时动态分配
```

### 4.3 CPU亲和性
```cpp
#include <pthread.h>
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(4, &cpuset);  // 绑定到 Core 4
pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
```

---

## 五、故障隔离与恢复

| 场景 | 处理策略 |
|------|----------|
| 控制节点崩溃 | Watchdog 检测 → 自动重启 → 切换保持模式 |
| 视觉节点崩溃 | 不影响控制回路，等待手动恢复 |
| 串口通信断开 | 超时检测 → 触发安全停止 |

---

## 六、启动脚本设计

```bash
#!/bin/bash
# start_rt_system.sh

# 1. 设置实时优先级
sudo chrt -f 85 ros2 run ARV_V1_MOVEIT torque_controller_node &

# 2. 绑定CPU (taskset)
taskset -c 4 ros2 run ARV_V1_MOVEIT hardware_interface_node &
taskset -c 2-3 ros2 run ARV_V1_MOVEIT mujoco_interface_node &

# 3. 普通进程
ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py &
```

---

## 七、性能验证

### 7.1 延迟测试
```bash
# cyclictest (RT内核延迟)
sudo cyclictest -p 80 -t 4 -m -D 60
# 目标: 最大延迟 < 100μs
```

### 7.2 监控指标
- 控制回路 jitter (ros2 topic delay)
- CPU 负载 (htop 分核查看)
- 内存使用 (无运行时分配)

---

**最后更新**: 2026-01-08
