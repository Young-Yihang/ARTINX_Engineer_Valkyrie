// ============================================================
// MIT 电机 CAN 协议 (MIT Motor CAN Protocol)
//
// 职责:
//   - 标准MIT电机控制帧编码
//   - 反馈帧解码
//   - 6关节电机ID配置
//
// 帧格式 (8字节):
//   控制帧: [p_des(16)] [v_des(12)] [kp(12)] [kd(12)] [t_ff(12)]
//   反馈帧: [motor_id(8)] [position(16)] [velocity(12)] [torque(12)]
//
// Stage: SIM2REAL - SocketCAN通信
// Author: ARV V1 Team
// Date: 2025-01-08
// ============================================================

#ifndef MIT_PROTOCOL_HPP
#define MIT_PROTOCOL_HPP

#include <cstdint>
#include <cstring>
#include <array>
#include <linux/can.h>

namespace MitProtocol
{
    // ========== 常量定义 ==========
    constexpr size_t NUM_JOINTS = 6;

    // MIT电机参数范围 (需要根据实际电机型号调整)
    constexpr float P_MIN = -12.5f;     // 位置最小值 (rad)
    constexpr float P_MAX = 12.5f;      // 位置最大值 (rad)
    constexpr float V_MIN = -45.0f;     // 速度最小值 (rad/s)
    constexpr float V_MAX = 45.0f;      // 速度最大值 (rad/s)
    constexpr float T_MIN = -18.0f;     // 力矩最小值 (Nm)
    constexpr float T_MAX = 18.0f;      // 力矩最大值 (Nm)
    constexpr float KP_MIN = 0.0f;      // Kp最小值
    constexpr float KP_MAX = 500.0f;    // Kp最大值
    constexpr float KD_MIN = 0.0f;      // Kd最小值
    constexpr float KD_MAX = 5.0f;      // Kd最大值

    // ========== 电机ID配置 (TODO: 用户提供具体ID) ==========
    // 控制帧ID: 发送到电机的CAN ID
    // 反馈帧ID: 电机返回的CAN ID (通常为 控制ID + offset)
    struct MotorConfig {
        uint32_t ctrl_id;    // 控制帧CAN ID
        uint32_t feedback_id; // 反馈帧CAN ID
    };

    // 默认ID配置 (占位符，需要根据实际硬件修改)
    // TODO: 用户提供6个电机的实际CAN ID
    inline std::array<MotorConfig, NUM_JOINTS> getDefaultMotorConfig() {
        return {{
            {0x01, 0x01},  // Joint 1
            {0x02, 0x02},  // Joint 2
            {0x03, 0x03},  // Joint 3
            {0x04, 0x04},  // Joint 4
            {0x05, 0x05},  // Joint 5
            {0x06, 0x06},  // Joint 6
        }};
    }

    // ========== 数据结构 ==========

    // 控制命令 (发送到电机)
    struct MotorCommand {
        float p_des;    // 期望位置 (rad)
        float v_des;    // 期望速度 (rad/s)
        float kp;       // 位置增益
        float kd;       // 速度增益
        float t_ff;     // 前馈力矩 (Nm)
    };

    // 电机反馈 (从电机接收)
    struct MotorFeedback {
        uint8_t motor_id;   // 电机ID
        float position;     // 当前位置 (rad)
        float velocity;     // 当前速度 (rad/s)
        float torque;       // 当前力矩 (Nm)
    };

    // ========== 编码函数 ==========

    // 浮点数转无符号整数 (线性映射)
    inline uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
        float span = x_max - x_min;
        if (x < x_min) x = x_min;
        if (x > x_max) x = x_max;
        return static_cast<uint16_t>((x - x_min) / span * ((1 << bits) - 1));
    }

    // 无符号整数转浮点数 (线性映射)
    inline float uintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits) {
        float span = x_max - x_min;
        return static_cast<float>(x) / ((1 << bits) - 1) * span + x_min;
    }

    // 编码控制帧 (MIT格式)
    // 返回: 填充好的CAN帧
    inline struct can_frame encodeControlFrame(uint32_t can_id, const MotorCommand& cmd) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));

        frame.can_id = can_id;
        frame.can_dlc = 8;

        // 编码各字段
        uint16_t p = floatToUint(cmd.p_des, P_MIN, P_MAX, 16);
        uint16_t v = floatToUint(cmd.v_des, V_MIN, V_MAX, 12);
        uint16_t kp = floatToUint(cmd.kp, KP_MIN, KP_MAX, 12);
        uint16_t kd = floatToUint(cmd.kd, KD_MIN, KD_MAX, 12);
        uint16_t t = floatToUint(cmd.t_ff, T_MIN, T_MAX, 12);

        // 打包到8字节 (大端序，MIT标准格式)
        frame.data[0] = p >> 8;
        frame.data[1] = p & 0xFF;
        frame.data[2] = v >> 4;
        frame.data[3] = ((v & 0x0F) << 4) | (kp >> 8);
        frame.data[4] = kp & 0xFF;
        frame.data[5] = kd >> 4;
        frame.data[6] = ((kd & 0x0F) << 4) | (t >> 8);
        frame.data[7] = t & 0xFF;

        return frame;
    }

    // 解码反馈帧 (MIT格式)
    inline MotorFeedback decodeFeedbackFrame(const struct can_frame& frame) {
        MotorFeedback fb;

        // 解析字段 (大端序)
        fb.motor_id = frame.data[0];
        uint16_t p = (frame.data[1] << 8) | frame.data[2];
        uint16_t v = (frame.data[3] << 4) | (frame.data[4] >> 4);
        uint16_t t = ((frame.data[4] & 0x0F) << 8) | frame.data[5];

        // 转换为物理量
        fb.position = uintToFloat(p, P_MIN, P_MAX, 16);
        fb.velocity = uintToFloat(v, V_MIN, V_MAX, 12);
        fb.torque = uintToFloat(t, T_MIN, T_MAX, 12);

        return fb;
    }

    // ========== 纯力矩模式辅助函数 ==========
    // 当只需要发送力矩（p_des=0, v_des=0, kp=0, kd=0）时使用
    inline struct can_frame encodeTorqueOnlyFrame(uint32_t can_id, float torque) {
        MotorCommand cmd = {0.0f, 0.0f, 0.0f, 0.0f, torque};
        return encodeControlFrame(can_id, cmd);
    }

} // namespace MitProtocol

#endif // MIT_PROTOCOL_HPP
