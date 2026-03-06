/**
 * @file serial_protocol.hpp
 * @brief Seasky serial protocol — frame builder/parser, CmdID definitions, enums
 *
 * Frame: [SOF(1)][Len(2)][CRC8(1)][CmdID(2)][Flags(2)][Payload(N)][CRC16(2)]
 * DataLen = sizeof(CmdID) + sizeof(Flags) + sizeof(Payload)
 */
#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "Crc.hpp"
//
// 包汇总 (TX: 上位机 → 下位机):
//   0x0002  CMD_TORQUE_CONTROL   200Hz  6×float = 24 B   6轴力矩
//   0x0004  CMD_GRIPPER_CONTROL   50Hz  1×uint8 = 1  B   夹爪动作flag(由力矩阈值转换)
//   0x0006  CMD_ARM_STATUS        10Hz  4×uint8 = 4  B   机械臂状态反馈
// 包汇总 (RX: 下位机 → 上位机):
//   0x0001  CMD_JOINT_FEEDBACK   200Hz  7×(float+float+uint32) = 84 B  7关节状态
//   0x0005  CMD_TASK_COMMAND     按需    3×uint8 = 3  B   任务指令(cmd+param+seq)

namespace SerialProtocol {
// ─────────── 基础常量 ───────────
constexpr uint8_t SOF = 0xA5;

// CmdID 定义
constexpr uint16_t CMD_JOINT_FEEDBACK = 0x0001;   // RX: 7关节状态反馈
constexpr uint16_t CMD_TORQUE_CONTROL = 0x0002;   // TX: 6轴力矩 200Hz
constexpr uint16_t CMD_GRIPPER_CONTROL = 0x0004;  // TX: 夹爪控制 50Hz（力矩阈值→flag）
constexpr uint16_t CMD_TASK_COMMAND = 0x0005;     // RX: 下位机→上位机 任务指令 (3B)
constexpr uint16_t CMD_ARM_STATUS = 0x0006;  // TX: 上位机→下位机 机械臂状态反馈 (4B, 10Hz)

constexpr size_t NUM_ARM_JOINTS = 6;  // 机械臂关节数 (KDL 动力学链)
constexpr size_t NUM_ALL_JOINTS = 7;  // 含夹爪的总关节数（反馈帧使用）

// ─────────── 0x0005 任务指令枚举 (下位机 → 上位机, 3B) ───────────
// 下位机操作手通过键鼠触发，MCU 打包发送给上位机执行
// Payload: [task_cmd(1)][param(1)][seq(1)]
//   task_cmd: 指令类型
//   param:    指令参数（含义随 task_cmd 变化）
//   seq:      序列号 0-255 循环，上位机用于去重（同 seq 不重复执行）
enum class TaskCmd : uint8_t {
  // ── 系统控制 0x0X ──
  EMERGENCY_STOP = 0x01,  // 急停: 零力矩, param 忽略
  RESET_HOME = 0x02,      // 回 Home: 执行 reset_trajectory, param 忽略

  // ── 取矿 0x1X ──
  PICK_ORE = 0x10,  // 取矿: param = ore_id (0-5), 执行对应取矿轨迹
  STOW_ORE = 0x11,  // 存矿: param = slot_id (0-5), 存到对应槽位

  // ── 兑矿 0x2X ──
  EXCHANGE_MODE    = 0x20,  // 兑矿模式切换: param = 1 进入(控制权交下位机), 0 退出(上位机接管)

  // ── 流程控制 0x3X ──
  NEXT_STEP = 0x30,   // 推进当前任务下一步: param 忽略
  ABORT_TASK = 0x31,  // 中止当前任务: param 忽略, 保持当前位置

  // ── 末端 0x4X ──
  GRIPPER_CMD = 0x40,  // 夹爪直接控制: param = GripperAction (0松/1夹/2停)

  // ── 控制模式 0x5X ──
  SET_CONTROL_MODE = 0x50,  // 设置控制模式: param = ControlMode (0/1/2)
};

// ─────────── 0x0006 机械臂状态反馈枚举 (上位机 → 下位机, 4B, 10Hz) ───────────
// Payload: [arm_state(1)][task_progress(1)][error_code(1)][gripper_state(1)]
// 下位机据此在操作手 UI 上显示状态、决定下一步指令时机

enum class ArmState : uint8_t {
  IDLE = 0x00,       // 空闲, 等待指令
  EXECUTING = 0x01,  // 正在执行轨迹
  HOLDING = 0x02,    // 保持位置 (轨迹完成后)
  ERROR = 0x03,      // 出错
  HOMING = 0x04,     // 正在回 Home
  RELAX = 0x05,      // 全零力矩 (电机断力)
  FREEDRIVE = 0x06,  // 仅重力补偿 (可手动推臂)
};

// ─────────── 控制模式枚举 (torque_controller 使用) ───────────
// 通过 /control_mode (UInt8) topic 由 mission_executor 统一管理
enum class ControlMode : uint8_t {
  RELAX = 0,      // 全零力矩: 电机断力, 上电默认
  FREEDRIVE = 1,  // 仅重力补偿: G(q), 无PD, 可手动推臂
  HOLD = 2,       // 重力补偿+PD: G(q)+PD, 锁定位置
  EXECUTE = 3,    // 轨迹执行: 全动力学前馈+反馈 (action server 内部切换)
};

enum class ArmError : uint8_t {
  NO_ERROR = 0x00,
  PLANNING_FAIL = 0x01,   // MoveIt 规划失败
  EXECUTION_FAIL = 0x02,  // 轨迹执行中断
  TIMEOUT = 0x03,         // 超时
  JOINT_LIMIT = 0x04,     // 关节超限
  ESTOP = 0x05,           // 急停触发
};

enum class GripperState : uint8_t {
  OPEN = 0x00,    // 已松开
  CLOSED = 0x01,  // 已夹紧
  MOVING = 0x02,  // 运动中
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

// 任务指令包 Payload (3 bytes, 下位机→上位机, 按需)
struct TaskCommandPacket {
  TaskCmd cmd;    // 指令类型
  uint8_t param;  // 指令参数 (ore_id / slot_id / GripperAction 等)
  uint8_t seq;    // 序列号 (去重用)
};

// 机械臂状态反馈包 Payload (4 bytes, 上位机→下位机, 10Hz)
struct ArmStatusPacket {
  ArmState arm_state;          // 当前状态
  uint8_t task_progress;       // 任务进度 0-100
  ArmError error_code;         // 错误码
  GripperState gripper_state;  // 夹爪状态
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

// 6轴力矩包 (TX)
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

// 机械臂状态反馈包 (TX, 10Hz)
inline std::vector<uint8_t> buildArmStatusPacket(const ArmStatusPacket &status) {
  std::vector<uint8_t> payload = {
      static_cast<uint8_t>(status.arm_state),
      status.task_progress,
      static_cast<uint8_t>(status.error_code),
      static_cast<uint8_t>(status.gripper_state),
  };
  return buildPacket(CMD_ARM_STATUS, 0x0000, payload);
}

// CRC tables are centralized in Crc.{hpp,cpp}; avoid duplicating them here to ensure a single
// source of truth.

}  // namespace SerialProtocol

#endif  // SERIAL_PROTOCOL_HPP
