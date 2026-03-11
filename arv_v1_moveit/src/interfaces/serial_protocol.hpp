/// @file serial_protocol.hpp
/// @brief Seasky protocol over USB CDC: [SOF][Len][CRC8][CmdID][Flags][Payload][CRC16]
#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "Crc.hpp"
// TX: 0x0002 torque 1kHz 24B, 0x0004 gripper 50Hz 1B, 0x0006 status 10Hz 4B
// RX: 0x0001 feedback 1kHz 84B, 0x0005 task_cmd on-demand 3B

namespace SerialProtocol {
constexpr uint8_t SOF = 0xA5;

constexpr uint16_t CMD_JOINT_FEEDBACK = 0x0001;   // RX 7-joint state 1kHz
constexpr uint16_t CMD_TORQUE_CONTROL = 0x0002;   // TX 6-axis torque 1kHz
constexpr uint16_t CMD_GRIPPER_CONTROL = 0x0004;  // TX gripper flag 50Hz
constexpr uint16_t CMD_TASK_COMMAND = 0x0005;     // RX task cmd 3B
constexpr uint16_t CMD_ARM_STATUS = 0x0006;       // TX arm status 4B 10Hz

constexpr size_t NUM_ARM_JOINTS = 6;
constexpr size_t NUM_ALL_JOINTS = 7;  // arm + gripper

// ─────────── 0x0005 TaskCmd: [cmd(1)][param(1)][seq(1)] ───────────
enum class TaskCmd : uint8_t {
  EMERGENCY_STOP = 0x01,    // param ignored
  RESET_HOME = 0x02,        // param ignored
  PICK_ORE = 0x10,          // param = ore_id (0-5)
  STOW_ORE = 0x11,          // param = slot_id (0-5)
  EXCHANGE_MODE = 0x20,     // param: 1=enter, 0=exit
  NEXT_STEP = 0x30,         // param ignored
  ABORT_TASK = 0x31,        // param ignored, hold position
  GRIPPER_CMD = 0x40,       // param = GripperAction
  SET_CONTROL_MODE = 0x50,  // param = ControlMode
};

// ─────────── 0x0006 ArmStatus: [state][progress][error][gripper] ───────────
enum class ArmState : uint8_t {
  IDLE = 0x00,
  EXECUTING = 0x01,
  HOLDING = 0x02,
  ERROR = 0x03,
  HOMING = 0x04,
  RELAX = 0x05,
  FREEDRIVE = 0x06,
};

// ─────────── ControlMode (via /control_mode UInt8 topic) ───────────
enum class ControlMode : uint8_t {
  RELAX = 0,      // zero torque
  FREEDRIVE = 1,  // G(q) only
  ARMED = 2,      // G(q) + PD hold, accepts trajectory execution
};

enum class ArmError : uint8_t {
  NO_ERROR = 0x00,
  PLANNING_FAIL = 0x01,
  EXECUTION_FAIL = 0x02,
  TIMEOUT = 0x03,
  JOINT_LIMIT = 0x04,
  ESTOP = 0x05,
};

enum class GripperState : uint8_t {
  OPEN = 0x00,
  CLOSED = 0x01,
  MOVING = 0x02,
};

// ─────────── 数据结构 ───────────

struct TorqueCommand {
  std::array<float, NUM_ARM_JOINTS> torques;  // [J1..J6]
};

enum class GripperAction : uint8_t {
  RELEASE = 0x00,
  GRIP = 0x01,
  STOP = 0x02,
};

struct TaskCommandPacket {
  TaskCmd cmd;
  uint8_t param;
  uint8_t seq;  // dedup: same seq → skip
};

struct ArmStatusPacket {
  ArmState arm_state;
  uint8_t task_progress;  // 0-100
  ArmError error_code;
  GripperState gripper_state;
};

struct JointFeedback {
  std::array<float, NUM_ALL_JOINTS> positions;
  std::array<float, NUM_ALL_JOINTS> velocities;
  std::array<uint32_t, NUM_ALL_JOINTS> islive;
};

inline uint8_t Get_CRC8_Check_Sum(const uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8) {
  return Crc::Get_CRC8_Check_Sum(const_cast<uint8_t *>(pchMessage), static_cast<uint32_t>(dwLength),
                                 ucCRC8);
}

inline uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
  return Crc::Get_CRC16_Check_Sum(const_cast<uint8_t *>(pchMessage), dwLength, wCRC);
}

// ─────────── Serialization (Little Endian) ───────────
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

// ─────────── Deserialization ───────────
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

// ─────────── Packet builder (single source of frame structure) ───────────
inline std::vector<uint8_t> buildPacket(uint16_t cmd_id, uint16_t flags,
                                        const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> pkt;
  pkt.reserve(4 + 2 + 2 + payload.size() + 2);  // Header+CmdID+Flags+Payload+CRC16

  pkt.push_back(SOF);
  pkt.push_back(0);  // Len L (filled below)
  pkt.push_back(0);  // Len H
  pkt.push_back(0);  // CRC8

  append_uint16(pkt, cmd_id);
  append_uint16(pkt, flags);
  pkt.insert(pkt.end(), payload.begin(), payload.end());

  const uint16_t data_len = static_cast<uint16_t>(pkt.size() - 4);
  pkt[1] = static_cast<uint8_t>(data_len & 0xFF);
  pkt[2] = static_cast<uint8_t>((data_len >> 8) & 0xFF);
  pkt[3] = Get_CRC8_Check_Sum(pkt.data(), 3, 0xFF);
  const uint16_t crc16 = Get_CRC16_Check_Sum(pkt.data(), pkt.size(), 0xFFFF);
  append_uint16(pkt, crc16);

  return pkt;
}

// ─────────── Per-command builders ───────────
inline std::vector<uint8_t> buildTorquePacket(const TorqueCommand &cmd) {
  std::vector<uint8_t> payload;
  payload.reserve(NUM_ARM_JOINTS * 4);
  for (size_t i = 0; i < NUM_ARM_JOINTS; ++i) append_float(payload, cmd.torques[i]);
  return buildPacket(CMD_TORQUE_CONTROL, 0x0000, payload);
}

inline std::vector<uint8_t> buildGripperPacket(GripperAction action) {
  std::vector<uint8_t> payload = {static_cast<uint8_t>(action)};
  return buildPacket(CMD_GRIPPER_CONTROL, 0x0000, payload);
}

inline std::vector<uint8_t> buildArmStatusPacket(const ArmStatusPacket &status) {
  std::vector<uint8_t> payload = {
      static_cast<uint8_t>(status.arm_state),
      status.task_progress,
      static_cast<uint8_t>(status.error_code),
      static_cast<uint8_t>(status.gripper_state),
  };
  return buildPacket(CMD_ARM_STATUS, 0x0000, payload);
}

}  // namespace SerialProtocol

#endif  // SERIAL_PROTOCOL_HPP
