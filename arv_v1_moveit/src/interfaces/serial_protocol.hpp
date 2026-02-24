#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "Crc.hpp"

// SEASKY协议: [SOF(1)][Len(2)][CRC8(1)][CmdID(2)][Flags(2)][Payload(N)][CRC16(2)]
// DataLen = sizeof(CmdID) + sizeof(Flags) + sizeof(Payload)
//
// 包汇总 (TX: 上位机 → 下位机):
//   0x0002  CMD_TORQUE_CONTROL   200Hz  6×float = 24 B   6轴力矩
//   0x0004  CMD_GRIPPER_CONTROL   50Hz  1×uint8 = 1  B   夹爪动作flag(由力矩阈值转换)
// 包汇总 (RX: 下位机 → 上位机):
//   0x0001  CMD_JOINT_FEEDBACK   200Hz  7×(float+float+uint32) = 84 B  7关节状态
//   0x0005  CMD_ROBOT_STATE_REQ  按需    1×uint8                        任务状态切换通知

namespace SerialProtocol {
// ─────────── 基础常量 ───────────
constexpr uint8_t SOF = 0xA5;

// CmdID 定义
constexpr uint16_t CMD_JOINT_FEEDBACK = 0x0001;   // RX: 7关节状态反馈
constexpr uint16_t CMD_TORQUE_CONTROL = 0x0002;   // TX: 6轴力矩 200Hz
constexpr uint16_t CMD_GRIPPER_OFF = 0x0003;      // 保留（兼容旧固件）
constexpr uint16_t CMD_GRIPPER_CONTROL = 0x0004;  // TX: 夹爪控制 50Hz（力矩阈值→flag）
constexpr uint16_t CMD_ROBOT_STATE_REQ = 0x0005;  // RX: 下位机→上位机 任务状态切换通知

constexpr size_t NUM_ARM_JOINTS = 6;  // 机械臂关节数 (KDL 动力学链)
constexpr size_t NUM_ALL_JOINTS = 7;  // 含夹爪的总关节数（反馈帧使用）

// ─────────── 任务状态切换通知枚举 ───────────
// RX方向：下位机主动推送（限位触发、传感器事件等），通知上位机状态机推进
enum class RobotStateCmd : uint8_t {
  EMERGENCY_RESET = 0x01,  // 紧急复位：停止一切运动，回原点
  START_ORE_PICK = 0x02,   // 开始取矿1：进入取矿执行序列第一步
  NEXT_STEP = 0x03,        // 下一步：推进当前任务序列到下一状态
  // 预留扩展位: 0x04~0xFF
};

// ─────────── 数据结构 ───────────

// 力矩控制包 Payload (6 关节, 200Hz)
struct TorqueCommand {
  std::array<float, NUM_ARM_JOINTS> torques;  // [J1..J6], 不含夹爪
};

// 夹爪控制包 Payload (1 byte flag, 50Hz)
enum class GripperAction : uint8_t {
  RELEASE = 0x00,  // 松开
  GRIP = 0x01,     // 夹紧
  STOP = 0x02,     // 停止保持
};

// 任务状态切换请求包 Payload (1 byte cmd, 按需发送)
struct RobotCmdPacket {
  RobotStateCmd cmd;  // 见 RobotStateCmd 枚举
};

// 关节状态反馈包 Payload (7 关节, 200Hz, 下位机 → 上位机)
struct JointFeedback {
  std::array<float, NUM_ALL_JOINTS> positions;
  std::array<float, NUM_ALL_JOINTS> velocities;
  std::array<uint32_t, NUM_ALL_JOINTS> islive;  // 每关节存活状态标志（暂不使用）
};

// CRC Helpers - delegate to centralized Crc implementation
// Use Crc::Get_CRC* to avoid duplicated tables across the codebase.
inline uint8_t Get_CRC8_Check_Sum(const uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8) {
  // Crc API expects non-const pointer
  return Crc::Get_CRC8_Check_Sum(const_cast<uint8_t *>(pchMessage), static_cast<uint32_t>(dwLength),
                                 ucCRC8);
}

inline uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
  return Crc::Get_CRC16_Check_Sum(const_cast<uint8_t *>(pchMessage), dwLength, wCRC);
}

// Serialization Helpers (Little Endian)
inline void append_uint8(std::vector<uint8_t> &buffer, uint8_t value) {
  buffer.push_back(value);
}

inline void append_uint16(std::vector<uint8_t> &buffer, uint16_t value) {
  buffer.push_back(static_cast<uint8_t>(value & 0xFF));
  buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

inline void append_float(std::vector<uint8_t> &buffer, float value) {
  uint8_t bytes[4];
  std::memcpy(bytes, &value, 4);
  for (int i = 0; i < 4; ++i) buffer.push_back(bytes[i]);
}

// Deserialization Helpers
inline uint16_t read_uint16(const uint8_t *buffer, size_t &offset) {
  uint16_t value = buffer[offset] | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
  offset += 2;
  return value;
}

inline float read_float(const uint8_t *buffer, size_t &offset) {
  float value;
  std::memcpy(&value, buffer + offset, 4);
  offset += 4;
  return value;
}

inline uint32_t read_uint32(const uint8_t *buffer, size_t &offset) {
  uint32_t value = buffer[offset] | (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
                   (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
                   (static_cast<uint32_t>(buffer[offset + 3]) << 24);
  offset += 4;
  return value;
}

// ─────────── 基础构建器（内部复用，帧结构唯一来源） ───────────
// payload 已经序列化完毕的字节序列，函数负责封装 SOF/Len/CRC8/CRC16
inline std::vector<uint8_t> buildPacket(uint16_t cmd_id, uint16_t flags,
                                        const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> pkt;
  pkt.reserve(4 + 2 + 2 + payload.size() + 2);  // Header+CmdID+Flags+Payload+CRC16

  // ── Header placeholder ──
  pkt.push_back(SOF);
  pkt.push_back(0);  // Len L (填后)
  pkt.push_back(0);  // Len H (填后)
  pkt.push_back(0);  // CRC8  (填后)

  // ── Body ──
  append_uint16(pkt, cmd_id);
  append_uint16(pkt, flags);
  pkt.insert(pkt.end(), payload.begin(), payload.end());

  // ── DataLen = CmdID(2) + Flags(2) + Payload ──
  const uint16_t data_len = static_cast<uint16_t>(pkt.size() - 4);
  pkt[1] = static_cast<uint8_t>(data_len & 0xFF);
  pkt[2] = static_cast<uint8_t>((data_len >> 8) & 0xFF);

  // ── Header CRC8 (SOF + Len_L + Len_H) ──
  pkt[3] = Get_CRC8_Check_Sum(pkt.data(), 3, 0xFF);

  // ── Whole-packet CRC16 ──
  const uint16_t crc16 = Get_CRC16_Check_Sum(pkt.data(), pkt.size(), 0xFFFF);
  append_uint16(pkt, crc16);

  return pkt;
}

// ─────────── 具体包构建器（仅负责序列化 payload） ───────────

// 6轴力矩包 (TX, 200Hz)
inline std::vector<uint8_t> buildTorquePacket(const TorqueCommand &cmd) {
  std::vector<uint8_t> payload;
  payload.reserve(NUM_ARM_JOINTS * 4);
  for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) append_float(payload, cmd.torques[i]);
  return buildPacket(CMD_TORQUE_CONTROL, 0x0000, payload);
}

// 夹爪动作包 (TX, 50Hz，force→flag 转换由调用方完成)
inline std::vector<uint8_t> buildGripperPacket(GripperAction action) {
  std::vector<uint8_t> payload = {static_cast<uint8_t>(action)};
  return buildPacket(CMD_GRIPPER_CONTROL, 0x0000, payload);
}


// CRC tables are centralized in Crc.{hpp,cpp}; avoid duplicating them here to ensure a single
// source of truth.

}  // namespace SerialProtocol

#endif  // SERIAL_PROTOCOL_HPP
