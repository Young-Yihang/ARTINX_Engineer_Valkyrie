#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "Crc.hpp"

// SEASKY协议: [SOF(1)][Len(2)][CRC8(1)][CmdID(2)][Flags(2)][Payload(N)][CRC16(2)]
// DataLen = sizeof(CmdID) + sizeof(Flags) + sizeof(Payload)

namespace SerialProtocol {
// Constants
constexpr uint8_t SOF = 0xA5;
constexpr uint16_t CMD_TORQUE_CONTROL = 0x0002;
constexpr uint16_t CMD_JOINT_FEEDBACK = 0x0001;
constexpr uint16_t CMD_GRIPPER_CONTROL = 0x0103;

constexpr size_t NUM_JOINTS = 6;

// Data Structures (Logical)
struct TorqueCommand {
  std::array<float, NUM_JOINTS> torques;
};

struct GripperCommand {
  float position;  // 0.0 (close) - 1.0 (open)
  float force;     // 0.0 - 1.0
};

struct JointFeedback {
  std::array<float, NUM_JOINTS> positions;
  std::array<float, NUM_JOINTS> velocities;
  std::array<uint32_t, NUM_JOINTS> islive;  // 每关节存活状态标志（暂不使用）
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

// Packet Builder
template <typename T>
std::vector<uint8_t> buildPacket(uint16_t cmd_id, const T &data) {
  std::vector<uint8_t> packet;
  packet.reserve(64);  // Pre-allocate reasonable size

  // 1. Payload Serialization
  std::vector<uint8_t> payload_bytes;
  // This part needs specialization or a generic way.
  // For simplicity, let's do it manually in specific builders.
  return packet;
}

// Specific Builders
inline std::vector<uint8_t> buildTorquePacket(const TorqueCommand &cmd) {
  std::vector<uint8_t> packet;
  packet.reserve(64);

  // Header Placeholder
  packet.push_back(SOF);
  packet.push_back(0);  // Len L
  packet.push_back(0);  // Len H
  packet.push_back(0);  // CRC8

  // Body
  append_uint16(packet, CMD_TORQUE_CONTROL);
  append_uint16(packet, 0x0000);  // Flags
  for (float t : cmd.torques) append_float(packet, t);

  // Calculate DataLen (CmdID + Flags + Payload)
  uint16_t data_len = packet.size() - 4;  // Current size - Header size
  packet[1] = static_cast<uint8_t>(data_len & 0xFF);
  packet[2] = static_cast<uint8_t>((data_len >> 8) & 0xFF);

  // Calculate CRC8 (SOF + Len_L + Len_H)
  packet[3] = Get_CRC8_Check_Sum(packet.data(), 3, 0xFF);

  // Calculate CRC16 (Whole packet so far)
  uint16_t crc16 = Get_CRC16_Check_Sum(packet.data(), packet.size(), 0xFFFF);
  append_uint16(packet, crc16);

  return packet;
}

inline std::vector<uint8_t> buildGripperPacket(const GripperCommand &cmd) {
  std::vector<uint8_t> packet;
  packet.reserve(32);

  packet.push_back(SOF);
  packet.push_back(0);
  packet.push_back(0);
  packet.push_back(0);

  append_uint16(packet, CMD_GRIPPER_CONTROL);
  append_uint16(packet, 0x0000);
  append_float(packet, cmd.position);
  append_float(packet, cmd.force);

  uint16_t data_len = packet.size() - 4;
  packet[1] = static_cast<uint8_t>(data_len & 0xFF);
  packet[2] = static_cast<uint8_t>((data_len >> 8) & 0xFF);
  packet[3] = Get_CRC8_Check_Sum(packet.data(), 3, 0xFF);

  uint16_t crc16 = Get_CRC16_Check_Sum(packet.data(), packet.size(), 0xFFFF);
  append_uint16(packet, crc16);

  return packet;
}

// CRC tables are centralized in Crc.{hpp,cpp}; avoid duplicating them here to ensure a single
// source of truth.

}  // namespace SerialProtocol

#endif  // SERIAL_PROTOCOL_HPP
