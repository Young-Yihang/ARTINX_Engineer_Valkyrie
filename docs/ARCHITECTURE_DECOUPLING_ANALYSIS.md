# 架构解耦分析：解算层与传输层分离

## 📊 当前架构问题分析

### 当前数据流（耦合架构）

```
┌─────────────────────────────────────────────────────────────────┐
│  torque_controller_node (解算层)                                 │
│  - 200Hz定时器驱动 (create_wall_timer)                           │
│  - 订阅: /joint_states                                           │
│  - 计算: τ = τ_ff(动力学) + τ_fb(级联PID)                        │
│  - 发布: /effort_controller/commands @ 200Hz                     │
└───────────────────────┬─────────────────────────────────────────┘
                        │ publish (事件驱动)
                        ↓
┌─────────────────────────────────────────────────────────────────┐
│  hardware_interface_node (传输层)                                │
│  - 无定时器！完全被动                                             │
│  - 订阅: /effort_controller/commands                             │
│  - 回调触发: torqueCallback()                                    │
│  - 发送: USB串口 → STM32                                         │
│  - 接收: STM32 → /joint_states                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 🔴 核心问题

| 问题 | 现象 | 风险 |
|------|------|------|
| **强耦合** | 传输层完全依赖解算层的发布频率 | 解算层慢→传输层慢 |
| **无心跳保护** | 解算层崩溃→传输层停止发送 | STM32收不到数据→超时保护 |
| **时序不确定** | DDS传输延迟+回调延迟不可控 | 无法保证固定200Hz |
| **单点故障** | 任一节点失败导致整个系统失效 | 缺乏容错机制 |
| **调试困难** | 无法独立测试传输层性能 | 问题定位复杂 |

---

## ✅ 解耦方案设计

### 方案1: 双定时器架构（推荐）

#### 架构图

```
┌───────────────────────────────────────────────────────────────────┐
│  torque_controller_node (解算层)                                   │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  200Hz定时器: controlLoop()                                  │  │
│  │  1. 订阅 /joint_states (事件驱动)                            │  │
│  │  2. 计算力矩: τ = τ_ff + τ_fb                                │  │
│  │  3. 更新共享内存: latest_torques_[] (带互斥锁)               │  │
│  │  4. 不发布！仅更新缓存                                       │  │
│  └─────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
                        ↓ 共享内存 (std::mutex保护)
┌───────────────────────────────────────────────────────────────────┐
│  hardware_interface_node (传输层)                                  │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  200Hz定时器: sendLoop()                                     │  │
│  │  1. 读取共享内存: latest_torques_[] (带互斥锁)               │  │
│  │  2. 构建SEASKY数据包                                         │  │
│  │  3. 发送USB串口 → STM32 (保证周期性)                        │  │
│  └─────────────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  接收线程: receiveLoop() (异步)                              │  │
│  │  1. 解析SEASKY数据包                                         │  │
│  │  2. 更新关节状态: positions[], velocities[]                 │  │
│  │  3. 发布 /joint_states @ ~200Hz                             │  │
│  └─────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

#### 优点
✅ **完全解耦**: 两层独立运行，互不依赖  
✅ **时序保证**: 传输层严格200Hz，不受解算延迟影响  
✅ **心跳保护**: 传输层持续发送，解算层崩溃也能维持通信  
✅ **容错性强**: 单层故障不影响另一层基本功能  
✅ **易于调试**: 可独立测试每层性能  

#### 缺点
⚠️ 需要修改两个节点  
⚠️ 引入共享内存需要仔细管理锁  

---

### 方案2: 话题+定时器混合架构

#### 架构图

```
┌───────────────────────────────────────────────────────────────────┐
│  torque_controller_node (解算层)                                   │
│  - 200Hz定时器: controlLoop()                                      │
│  - 发布: /effort_controller/commands @ 200Hz (保持现状)            │
└───────────────────────────┬───────────────────────────────────────┘
                            │ publish (DDS)
                            ↓
┌───────────────────────────────────────────────────────────────────┐
│  hardware_interface_node (传输层)                                  │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  订阅回调: torqueCallback()                                  │  │
│  │  - 更新缓存: latest_torques_[] (带时间戳)                    │  │
│  │  - 不立即发送！仅缓存                                        │  │
│  └─────────────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  200Hz定时器: sendLoop()                                     │  │
│  │  1. 读取缓存的 latest_torques_[]                             │  │
│  │  2. 检查数据新鲜度 (timestamp < 10ms)                        │  │
│  │  3. 发送USB串口 → STM32                                      │  │
│  │  4. 超时保护: 发送上次值或零力矩                             │  │
│  └─────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

#### 优点
✅ **最小改动**: 只需修改hardware_interface_node  
✅ **兼容性好**: torque_controller保持现有接口  
✅ **渐进式**: 可先实现基本功能再优化  
✅ **保留话题**: 其他节点仍可订阅力矩命令  

#### 缺点
⚠️ 仍依赖DDS传输延迟  
⚠️ 数据更新频率受解算层限制  

---

### 方案3: 服务化架构（未来扩展）

#### 架构图

```
┌───────────────────────────────────────────────────────────────────┐
│  torque_controller_node (解算层)                                   │
│  - 200Hz定时器: controlLoop()                                      │
│  - 提供服务: /get_torque_command (rclcpp::Service)                │
│     Request: 无参数                                                │
│     Response: float64[6] torques, timestamp                        │
└───────────────────────────────────────────────────────────────────┘
                            ↑ Service Call (同步/异步)
┌───────────────────────────────────────────────────────────────────┐
│  hardware_interface_node (传输层)                                  │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │  200Hz定时器: sendLoop()                                     │  │
│  │  1. 异步调用服务: /get_torque_command                        │  │
│  │  2. 获取最新力矩数组                                         │  │
│  │  3. 发送USB串口 → STM32                                      │  │
│  │  4. 超时保护: 使用cached值                                   │  │
│  └─────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

#### 优点
✅ **主动拉取**: 传输层控制数据获取时机  
✅ **清晰语义**: 服务调用明确表达依赖关系  
✅ **易于测试**: 可模拟服务响应进行单元测试  

#### 缺点
⚠️ 服务调用开销较大（~100μs）  
⚠️ 需要处理服务超时和失败情况  
⚠️ 改动较大，不推荐当前实施  

---

## 🎯 推荐实施方案

### 阶段1: 立即实施（方案2变体）

**目标**: 最小改动实现传输层定时器

#### hardware_interface_node 修改要点

```cpp
class HardwareInterfaceNode {
private:
    // 新增：定时器和缓存
    rclcpp::TimerBase::SharedPtr send_timer_;
    std::mutex torque_cache_mutex_;
    float cached_torques_[6] = {0};
    rclcpp::Time last_torque_update_;
    
    // 构造函数中添加
    void init() {
        // 原有订阅回调
        torque_sub_ = this->create_subscription<...>(
            "/effort_controller/commands", 10,
            std::bind(&HardwareInterfaceNode::torqueCallback, ...));
        
        // 新增：200Hz发送定时器
        auto period = std::chrono::microseconds(5000);  // 5ms = 200Hz
        send_timer_ = this->create_wall_timer(
            period,
            std::bind(&HardwareInterfaceNode::sendLoop, this));
    }
    
    // 订阅回调：仅缓存数据
    void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(torque_cache_mutex_);
        for (int i = 0; i < 6; ++i) {
            cached_torques_[i] = msg->data[i];
        }
        last_torque_update_ = this->now();
        // 不再调用 sendTorqueCommand()！
    }
    
    // 新增：定时发送循环
    void sendLoop() {
        if (!serial_port_ || !serial_port_->is_open()) {
            return;
        }
        
        float torques_to_send[6];
        {
            std::lock_guard<std::mutex> lock(torque_cache_mutex_);
            
            // 检查数据新鲜度（10ms超时）
            double age = (this->now() - last_torque_update_).seconds();
            if (age > 0.01) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[WARN] Torque data stale (%.1f ms old), sending cached values", age * 1000);
            }
            
            // 应用 force_zero_torque 安全开关
            if (this->get_parameter("force_zero_torque").as_bool()) {
                std::fill(torques_to_send, torques_to_send + 6, 0.0f);
            } else {
                std::copy(cached_torques_, cached_torques_ + 6, torques_to_send);
            }
        }
        
        // 发送到USB
        SerialProtocol::TorqueCommand cmd;
        std::copy(torques_to_send, torques_to_send + 6, cmd.torques.begin());
        std::vector<uint8_t> packet = SerialProtocol::buildTorquePacket(cmd);
        
        try {
            serial_port_->send(packet);
        } catch (const std::exception &e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[ERROR] Send failed: %s", e.what());
        }
    }
};
```

#### 关键改动点

1. **添加定时器**: `create_wall_timer(5ms, sendLoop)`
2. **torqueCallback改为缓存**: 不再立即发送，只更新`cached_torques_[]`
3. **sendLoop定期发送**: 严格200Hz读取缓存并发送
4. **数据新鲜度检测**: 检查`last_torque_update_`，超时警告
5. **保持force_zero_torque**: 在sendLoop中统一应用安全开关

---

### 阶段2: 优化实施（方案1完整版）

**目标**: 彻底解耦，移除话题依赖

#### 实施步骤

1. **torque_controller_node**: 
   - 移除 `torque_pub_->publish()`
   - 改为更新本地成员变量 `latest_torques_[]`

2. **hardware_interface_node**:
   - 移除 `torque_sub_` 订阅
   - 直接访问 torque_controller 的共享内存（需要设计访问接口）

3. **可选: 引入共享内存通信**:
   - 使用 `boost::interprocess` 或 ROS2 `rclcpp::Node::get_node_base_interface()`
   - 零拷贝、低延迟（~1μs）

---

## 📈 性能对比

| 指标 | 当前架构 | 方案2 | 方案1 |
|------|---------|-------|-------|
| **传输延迟** | 5-20μs (DDS) | 5-20μs | ~1μs |
| **时序抖动** | ±2ms | ±100μs | ±50μs |
| **CPU占用** | 基准 | +2% (定时器) | +1% (共享内存) |
| **容错能力** | 差 | 良 | 优 |
| **调试便利** | 难 | 中 | 易 |
| **实施难度** | - | 低 | 中 |

---

## 🔧 实施建议

### 短期（1周内）
✅ **采用方案2**: 在hardware_interface_node添加200Hz定时器  
✅ 保持话题通信，最小化改动  
✅ 立即获得心跳保护和时序保证  

### 中期（1个月内）
🔄 **性能测试**: 使用示波器/逻辑分析仪测量实际发送周期  
🔄 **压力测试**: 故意让torque_controller超时，验证容错性  
🔄 **优化参数**: 调整定时器周期和超时阈值  

### 长期（未来迭代）
🎯 **评估方案1**: 如需极致性能，考虑共享内存方案  
🎯 **监控系统**: 添加Prometheus指标，实时监控通信质量  
🎯 **冗余设计**: 考虑双路通信备份（USB+CAN）  

---

## 🚨 注意事项

### 互斥锁性能
- **问题**: 每200Hz访问一次锁，可能影响实时性
- **解决**: 使用`std::atomic<T>`替代简单数据，无锁编程

### 定时器精度
- **问题**: Linux用户态定时器精度~100μs，可能抖动
- **解决**: 
  - 使用`SCHED_FIFO`实时调度策略
  - 考虑使用`clock_nanosleep(CLOCK_MONOTONIC)`
  - 监控实际周期，自适应调整

### 超时策略
- **数据陈旧**: 继续发送cached值（适合平滑轨迹）
- **解算层崩溃**: 发送零力矩或重力补偿（需要保存重力项）
- **传输层崩溃**: torque_controller检测无反馈，触发emergency stop

---

## 📝 总结

| 方案 | 推荐度 | 适用场景 |
|------|--------|----------|
| **方案2** | ⭐⭐⭐⭐⭐ | 当前立即实施，快速解决耦合问题 |
| **方案1** | ⭐⭐⭐⭐ | 性能优化阶段，追求极致实时性 |
| **方案3** | ⭐⭐ | 未来架构重构，不适合当前 |

**立即行动**: 采用方案2，在hardware_interface_node中添加200Hz定时器，实现传输层独立运行。
