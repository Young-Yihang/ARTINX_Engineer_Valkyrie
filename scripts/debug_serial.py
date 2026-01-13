#!/usr/bin/env python3
"""
ARV_V1 USB串口调试工具 - 基于Seasky魔改协议
功能: 设备检测、连接状态、抓包解析、统计分析
用法: python3 debug_serial.py [--port /dev/ttyACM0] [--baud 921600]
"""
import serial
import struct
import time
import argparse
import glob
import os
from dataclasses import dataclass
from typing import Optional, List
from collections import deque

# ========== Seasky协议常量 ==========
SOF = 0xA5
CMD_TORQUE_CONTROL = 0x0101  # NUC → STM32
CMD_JOINT_FEEDBACK = 0x0102  # STM32 → NUC
CMD_GRIPPER_CONTROL = 0x0103

# CRC8表 (与C++一致)
CRC8_TABLE = bytes([
    0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
    0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
    0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
    0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
    0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
    0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
    0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
    0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
    0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
    0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
    0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
    0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
    0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
    0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
    0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
    0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35,
])

def crc8(data: bytes, init=0xFF) -> int:
    crc = init
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc

@dataclass
class PacketStats:
    total: int = 0
    valid: int = 0
    crc8_err: int = 0
    crc16_err: int = 0
    incomplete: int = 0
    feedback_count: int = 0
    torque_count: int = 0

class SerialDebugger:
    def __init__(self, port: str, baud: int = 921600):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.stats = PacketStats()
        self.last_feedback_time = 0
        self.feedback_intervals = deque(maxlen=100)

    def detect_devices(self) -> List[str]:
        """检测可用串口设备"""
        patterns = ['/dev/ttyACM*', '/dev/ttyUSB*']
        devices = []
        for p in patterns:
            devices.extend(glob.glob(p))
        return sorted(devices)

    def check_device_info(self, dev: str) -> dict:
        """获取设备详细信息"""
        info = {'path': dev, 'exists': os.path.exists(dev)}
        if info['exists']:
            info['readable'] = os.access(dev, os.R_OK)
            info['writable'] = os.access(dev, os.W_OK)
            # 尝试读取USB信息
            try:
                base = os.path.basename(dev)
                sysfs = f"/sys/class/tty/{base}/device"
                if os.path.exists(sysfs):
                    vendor = os.path.join(sysfs, "../idVendor")
                    product = os.path.join(sysfs, "../idProduct")
                    if os.path.exists(vendor):
                        info['vendor'] = open(vendor).read().strip()
                    if os.path.exists(product):
                        info['product'] = open(product).read().strip()
            except:
                pass
        return info

    def connect(self) -> bool:
        """连接串口"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"[OK] 已连接: {self.port} @ {self.baud}")
            return True
        except Exception as e:
            print(f"[错误] 连接失败: {e}")
            return False

    def parse_packet(self, data: bytes) -> Optional[dict]:
        """解析Seasky协议包"""
        if len(data) < 4 or data[0] != SOF:
            return None

        # Header: SOF(1) + DataLen(2) + CRC8(1)
        data_len = data[1] | (data[2] << 8)
        crc8_recv = data[3]
        crc8_calc = crc8(data[:3])

        if crc8_recv != crc8_calc:
            self.stats.crc8_err += 1
            return {'error': 'CRC8', 'expected': crc8_calc, 'got': crc8_recv}

        total_len = 4 + data_len + 2  # Header + Body + CRC16
        if len(data) < total_len:
            self.stats.incomplete += 1
            return {'error': 'incomplete', 'need': total_len, 'got': len(data)}

        # Body: CmdID(2) + Flags(2) + Payload(N)
        cmd_id = data[4] | (data[5] << 8)
        flags = data[6] | (data[7] << 8)
        payload = data[8:4+data_len]

        result = {'cmd_id': cmd_id, 'flags': flags, 'len': data_len}

        # 解析具体命令
        if cmd_id == CMD_JOINT_FEEDBACK and len(payload) >= 48:
            positions = struct.unpack('<6f', payload[0:24])
            velocities = struct.unpack('<6f', payload[24:48])
            result['positions'] = positions
            result['velocities'] = velocities
            self.stats.feedback_count += 1
        elif cmd_id == CMD_TORQUE_CONTROL and len(payload) >= 24:
            torques = struct.unpack('<6f', payload[0:24])
            result['torques'] = torques
            self.stats.torque_count += 1

        self.stats.valid += 1
        return result

    def find_packet(self, buffer: bytes) -> tuple:
        """在缓冲区中查找完整包，返回(packet, remaining)"""
        idx = buffer.find(bytes([SOF]))
        if idx == -1:
            return None, b''
        if idx > 0:
            buffer = buffer[idx:]

        if len(buffer) < 4:
            return None, buffer

        data_len = buffer[1] | (buffer[2] << 8)
        total_len = 4 + data_len + 2

        if len(buffer) < total_len:
            return None, buffer

        packet = buffer[:total_len]
        remaining = buffer[total_len:]
        return packet, remaining

    def sniff(self, duration: float = 5.0, verbose: bool = True):
        """抓包分析"""
        if not self.ser:
            print("[错误] 未连接")
            return

        print(f"\n抓包中 ({duration}s)...")
        buffer = b''
        start = time.time()
        last_print = start

        while time.time() - start < duration:
            chunk = self.ser.read(256)
            if chunk:
                buffer += chunk
                self.stats.total += len(chunk)

            while True:
                pkt, buffer = self.find_packet(buffer)
                if pkt is None:
                    break
                result = self.parse_packet(pkt)

                # 计算帧间隔
                now = time.time()
                if self.last_feedback_time > 0:
                    interval = (now - self.last_feedback_time) * 1000
                    self.feedback_intervals.append(interval)
                self.last_feedback_time = now

                if verbose and result and 'error' not in result:
                    if result['cmd_id'] == CMD_JOINT_FEEDBACK:
                        pos = result.get('positions', [])
                        print(f"[FB] q=[{pos[0]:6.3f},{pos[1]:6.3f},{pos[2]:6.3f},"
                              f"{pos[3]:6.3f},{pos[4]:6.3f},{pos[5]:6.3f}]")

            # 每秒打印统计
            if time.time() - last_print >= 1.0:
                self._print_live_stats()
                last_print = time.time()

        self._print_summary()

    def _print_live_stats(self):
        hz = len(self.feedback_intervals)
        if self.feedback_intervals:
            avg_ms = sum(self.feedback_intervals) / len(self.feedback_intervals)
            print(f"  [统计] 帧率≈{hz}Hz, 平均间隔={avg_ms:.2f}ms, "
                  f"CRC错误={self.stats.crc8_err}")
        self.feedback_intervals.clear()

    def _print_summary(self):
        print("\n" + "="*50)
        print("  抓包统计")
        print("="*50)
        print(f"  总字节: {self.stats.total}")
        print(f"  有效包: {self.stats.valid}")
        print(f"  反馈包: {self.stats.feedback_count}")
        print(f"  力矩包: {self.stats.torque_count}")
        print(f"  CRC8错误: {self.stats.crc8_err}")
        print(f"  不完整包: {self.stats.incomplete}")
        if self.stats.crc8_err > 0:
            print(f"  [!] CRC错误率: {self.stats.crc8_err*100/max(1,self.stats.valid):.1f}%")
        print("="*50)


def main():
    parser = argparse.ArgumentParser(description='ARV_V1 USB串口调试')
    parser.add_argument('--port', '-p', default='/dev/ttyACM0')
    parser.add_argument('--baud', '-b', type=int, default=921600)
    args = parser.parse_args()

    dbg = SerialDebugger(args.port, args.baud)

    while True:
        print("\n" + "="*40)
        print("  USB串口调试工具 (Seasky协议)")
        print("="*40)
        print("  [1] 检测串口设备")
        print("  [2] 检查设备详情")
        print("  [3] 连接并抓包 (5秒)")
        print("  [4] 长时间监控 (30秒)")
        print("  [0] 退出")
        print("-"*40)

        c = input("选择: ").strip()
        if c == '1':
            devs = dbg.detect_devices()
            print(f"\n检测到 {len(devs)} 个设备:")
            for d in devs:
                info = dbg.check_device_info(d)
                perm = "rw" if info.get('readable') and info.get('writable') else "只读" if info.get('readable') else "无权限"
                print(f"  {d} [{perm}]")
        elif c == '2':
            info = dbg.check_device_info(args.port)
            print(f"\n设备: {args.port}")
            print(f"  存在: {info.get('exists')}")
            print(f"  可读: {info.get('readable')}")
            print(f"  可写: {info.get('writable')}")
            print(f"  VID: {info.get('vendor', 'N/A')}")
            print(f"  PID: {info.get('product', 'N/A')}")
        elif c == '3':
            if dbg.connect():
                dbg.sniff(5.0, verbose=True)
                dbg.ser.close()
        elif c == '4':
            if dbg.connect():
                dbg.sniff(30.0, verbose=False)
                dbg.ser.close()
        elif c == '0':
            break

if __name__ == '__main__':
    main()
