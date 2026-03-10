#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机连接测试程序 (Servo Connection Test)
========================================
自动检测串口并测试舵机控制器连接。
支持 LOBOT 总线舵机协议 (0x55 0x55 帧头, 9600 波特率)。

用法:
    python servo_test.py                # 自动检测串口并测试
    python servo_test.py --port COM13   # 指定串口
    python servo_test.py --scan         # 仅扫描串口
    python servo_test.py --move         # 测试舵机移动
    python servo_test.py --id-scan      # 扫描所有舵机ID
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
import argparse

# 解决 Windows 终端编码问题
if sys.platform == 'win32':
    os.system('')  # 启用 ANSI 转义序列
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# ============================================================
# 协议常量 (Protocol Constants)
# ============================================================
FRAME_HEADER = 0x55
CMD_SERVO_MOVE = 3
CMD_ACTION_GROUP_RUN = 6
CMD_ACTION_GROUP_STOP = 7
CMD_ACTION_GROUP_SPEED = 11
CMD_GET_BATTERY_VOLTAGE = 15
CMD_MULT_SERVO_UNLOAD = 20
CMD_MULT_SERVO_POS_READ = 21

# LX 舵机直连协议指令 (Direct LX Servo Protocol Commands)
LX_CMD_SERVO_MOVE_TIME_WRITE = 1
LX_CMD_SERVO_MOVE_TIME_READ = 2
LX_CMD_ID_WRITE = 13
LX_CMD_ID_READ = 14
LX_CMD_ANGLE_OFFSET_ADJUST = 17
LX_CMD_ANGLE_OFFSET_WRITE = 18
LX_CMD_ANGLE_OFFSET_READ = 19
LX_CMD_ANGLE_LIMIT_WRITE = 20
LX_CMD_ANGLE_LIMIT_READ = 21
LX_CMD_VIN_LIMIT_WRITE = 22
LX_CMD_VIN_LIMIT_READ = 23
LX_CMD_TEMP_MAX_LIMIT_WRITE = 24
LX_CMD_TEMP_MAX_LIMIT_READ = 25
LX_CMD_TEMP_READ = 26
LX_CMD_VIN_READ = 27
LX_CMD_POS_READ = 28
LX_CMD_SERVO_OR_MOTOR_MODE_WRITE = 29
LX_CMD_SERVO_OR_MOTOR_MODE_READ = 30
LX_CMD_LOAD_OR_UNLOAD_WRITE = 31
LX_CMD_LOAD_OR_UNLOAD_READ = 32
LX_CMD_LED_CTRL_WRITE = 33
LX_CMD_LED_CTRL_READ = 34
LX_CMD_LED_ERROR_WRITE = 35
LX_CMD_LED_ERROR_READ = 36

LX_SERVO_BROADCAST_ID = 254  # 广播地址

DEFAULT_BAUD = 9600
LX_BAUD = 115200  # LX 舵机直连默认波特率


# ============================================================
# 串口扫描 (Serial Port Scanner)
# ============================================================
def scan_serial_ports():
    """扫描系统中所有可用的串口"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[X] 未发现任何串口设备")
        return []

    print(f"[SCAN] 发现 {len(ports)} 个串口设备:\n")
    print(f"  {'端口':<10} {'描述':<40} {'硬件ID'}")
    print(f"  {'----':<10} {'----':<40} {'------'}")
    for p in ports:
        print(f"  {p.device:<10} {p.description:<40} {p.hwid}")

    return ports


def find_servo_port():
    """自动查找最可能连接舵机控制器的串口 (优先选 USB 转串口)"""
    ports = list(serial.tools.list_ports.comports())
    usb_keywords = ['CH34', 'CP210', 'FTDI', 'USB', 'Prolific']
    for p in ports:
        desc = (p.description or '').upper()
        for kw in usb_keywords:
            if kw.upper() in desc:
                return p.device
    if ports:
        return ports[0].device
    return None


# ============================================================
# 舵机控制器类 (Servo Controller)
# ============================================================
class ServoController:
    def __init__(self, port, baud=DEFAULT_BAUD, timeout=1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            time.sleep(0.1)
            return True
        except serial.SerialException as e:
            print(f"[X] 无法打开串口 {self.port}: {e}")
            return False

    def disconnect(self):
        """断开串口"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None

    def is_connected(self):
        """检查串口是否已连接"""
        return self.serial is not None and self.serial.is_open

    def _build_move_cmd(self, servo_id, position, move_time):
        """构建单舵机移动指令"""
        buf = bytearray(b'\x55\x55')
        buf.append(0x08)            # 数据长度
        buf.append(CMD_SERVO_MOVE)  # 指令
        buf.append(0x01)            # 舵机数量

        move_time = max(0, min(30000, move_time))
        buf.extend(move_time.to_bytes(2, 'little'))

        servo_id = max(1, min(254, servo_id))
        buf.append(servo_id)

        position = max(0, min(1000, position))
        buf.extend(position.to_bytes(2, 'little'))

        return buf

    def move_servo(self, servo_id, position, move_time=1000):
        """控制单个舵机移动"""
        if not self.is_connected():
            print("[X] 串口未连接")
            return False
        cmd = self._build_move_cmd(servo_id, position, move_time)
        try:
            self.serial.write(cmd)
            return True
        except serial.SerialException as e:
            print(f"[X] 发送指令失败: {e}")
            return False

    def get_battery_voltage(self):
        """读取电池电压"""
        if not self.is_connected():
            return None

        buf = bytearray(b'\x55\x55')
        buf.append(0x02)
        buf.append(CMD_GET_BATTERY_VOLTAGE)

        try:
            self.serial.flushInput()
            self.serial.write(buf)
            time.sleep(0.15)

            if self.serial.in_waiting >= 4:
                resp = self.serial.read(self.serial.in_waiting)
                if len(resp) >= 6 and resp[0] == 0x55 and resp[1] == 0x55:
                    voltage = resp[4] | (resp[5] << 8)
                    return voltage  # mV
            return None
        except serial.SerialException:
            return None

    def move_multiple_servos(self, servos_dict, move_time=1000):
        """控制多个舵机移动
        servos_dict: 字典 {servo_id: position}
        """
        if not self.is_connected():
            return False
        
        count = len(servos_dict)
        if count == 0:
            return True
        count = max(1, min(254, count))
        
        buf = bytearray(b'\x55\x55')
        buf.append(count * 3 + 5)
        buf.append(CMD_SERVO_MOVE)
        buf.append(count)
        
        move_time = max(0, min(30000, move_time))
        buf.extend(move_time.to_bytes(2, 'little'))
        
        for sid, pos in list(servos_dict.items())[:count]:
            sid = max(1, min(254, sid))
            buf.append(sid)
            pos = max(0, min(1000, pos))
            buf.extend(pos.to_bytes(2, 'little'))
            
        try:
            self.serial.write(buf)
            return True
        except serial.SerialException:
            return False

    def run_action_group(self, group_id, times=1):
        """执行动作组 (times=0 为无限循环)"""
        if not self.is_connected():
            return False
            
        buf = bytearray(b'\x55\x55')
        buf.append(0x05)
        buf.append(CMD_ACTION_GROUP_RUN)
        buf.append(group_id & 0xFF)
        buf.extend(times.to_bytes(2, 'little'))
        
        try:
            self.serial.write(buf)
            return True
        except serial.SerialException:
            return False

    def stop_action_group(self):
        """停止动作组运行"""
        if not self.is_connected():
            return False
            
        buf = bytearray(b'\x55\x55\x02')
        buf.append(CMD_ACTION_GROUP_STOP)
        
        try:
            self.serial.write(buf)
            return True
        except serial.SerialException:
            return False

    def set_action_group_speed(self, group_id, speed_percent):
        """设置动作组运行速度 (百分比, 如 200 就是 200%, group_id=255/0xFF 控制所有)"""
        if not self.is_connected():
            return False
            
        buf = bytearray(b'\x55\x55')
        buf.append(0x05)
        buf.append(CMD_ACTION_GROUP_SPEED)
        buf.append(group_id & 0xFF)
        buf.extend(speed_percent.to_bytes(2, 'little'))
        
        try:
            self.serial.write(buf)
            return True
        except serial.SerialException:
            return False

    def unload_servos(self, servo_ids):
        """卸载/掉电指定舵机使其失去扭力"""
        if not self.is_connected():
            return False
            
        count = len(servo_ids)
        if count == 0:
            return True
        count = max(1, min(254, count))
        
        buf = bytearray(b'\x55\x55')
        buf.append(count + 3)
        buf.append(CMD_MULT_SERVO_UNLOAD)
        buf.append(count)
        
        for sid in servo_ids[:count]:
            buf.append(max(1, min(254, sid)))
            
        try:
            self.serial.write(buf)
            return True
        except serial.SerialException:
            return False

    def read_multiple_positions(self, servo_ids):
        """读取多个舵机的当前位置 (控制器板协议)"""
        if not self.is_connected():
            return None
            
        count = len(servo_ids)
        if count == 0:
            return {}
        count = max(1, min(254, count))
        
        buf = bytearray(b'\x55\x55')
        buf.append(count + 3)
        buf.append(CMD_MULT_SERVO_POS_READ)
        buf.append(count)
        
        for sid in servo_ids[:count]:
            buf.append(max(1, min(254, sid)))
            
        try:
            self.serial.flushInput()
            self.serial.write(buf)
            time.sleep(0.2)
            
            if self.serial.in_waiting > 0:
                resp = self.serial.read(self.serial.in_waiting)
                # 解析返回包
                # 0x55 0x55 (N*3+3) 0x15 N (ID POS_L POS_H)...
                if len(resp) >= 6 and resp[0] == 0x55 and resp[1] == 0x55:
                    resp_cmd = resp[3]
                    if resp_cmd == CMD_MULT_SERVO_POS_READ:
                        num = resp[4]
                        expected_len = 5 + num * 3
                        if len(resp) >= expected_len:
                            positions = {}
                            idx = 5
                            for _ in range(num):
                                sid = resp[idx]
                                pos = resp[idx+1] | (resp[idx+2] << 8)
                                positions[sid] = pos
                                idx += 3
                            return positions
            return None
        except serial.SerialException:
            return None

    # --------------------------------------------------------
    # LX 舵机直连协议方法 (Direct LX servo protocol)
    # --------------------------------------------------------
    @staticmethod
    def _lx_checksum(servo_id, length, cmd, params=b''):
        """计算 LX 协议校验和"""
        s = servo_id + length + cmd
        for b in params:
            s += b
        return (~s) & 0xFF

    def _lx_send_cmd(self, servo_id, cmd, params=b''):
        """发送 LX 协议指令"""
        length = len(params) + 3  # length 包含 id, len, cmd, params, 不含 header
        checksum = self._lx_checksum(servo_id, length, cmd, params)
        buf = bytearray([0x55, 0x55, servo_id, length, cmd])
        buf.extend(params)
        buf.append(checksum)
        self.serial.write(buf)

    def _lx_read_response(self, timeout=0.1):
        """读取 LX 协议回复"""
        deadline = time.time() + timeout
        buf = bytearray()

        while time.time() < deadline:
            if self.serial.in_waiting > 0:
                buf.extend(self.serial.read(self.serial.in_waiting))
                # 尝试解析完整帧
                while len(buf) >= 2:
                    # 查找帧头
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == 0x55 and buf[i+1] == 0x55:
                            idx = i
                            break
                    if idx < 0:
                        buf = buf[-1:]  # 保留最后一个字节
                        break
                    if idx > 0:
                        buf = buf[idx:]  # 丢弃前面的垃圾数据
                    if len(buf) < 4:
                        break  # 还不够，继续读
                    pkt_len = buf[3]
                    total_len = 3 + pkt_len  # header(2) + id(1) + [len+cmd+params+checksum]
                    if len(buf) < total_len:
                        break  # 还不够
                    packet = buf[:total_len]
                    return packet
            time.sleep(0.005)
        return None

    def lx_read_id(self, servo_id=LX_SERVO_BROADCAST_ID):
        """读取舵机 ID (LX 直连协议)"""
        if not self.is_connected():
            return None
        try:
            self.serial.flushInput()
            self._lx_send_cmd(servo_id, LX_CMD_ID_READ)
            resp = self._lx_read_response(timeout=0.15)
            if resp and len(resp) >= 7:
                return resp[5]  # 返回的 ID 值
            return None
        except serial.SerialException:
            return None

    def lx_write_id(self, old_id, new_id):
        """修改舵机 ID (LX 直连协议)
        注意：通常建议总线上只接一个舵机时修改 ID，防止冲突。
        """
        if not self.is_connected():
            return False
        new_id = max(0, min(253, new_id))
        try:
            self.serial.flushInput()
            self._lx_send_cmd(old_id, LX_CMD_ID_WRITE, bytes([new_id]))
            # 某些舵机修改ID后不会返回确认包，给点时间保存
            time.sleep(0.1)
            return True
        except serial.SerialException:
            return False

    def lx_read_position(self, servo_id):
        """读取舵机当前位置 (LX 直连协议)"""
        if not self.is_connected():
            return None
        try:
            self.serial.flushInput()
            self._lx_send_cmd(servo_id, LX_CMD_POS_READ)
            resp = self._lx_read_response(timeout=0.15)
            if resp and len(resp) >= 8:
                pos = resp[5] | (resp[6] << 8)
                # 有符号 16 位
                if pos > 32767:
                    pos -= 65536
                return pos
            return None
        except serial.SerialException:
            return None

    def lx_read_vin(self, servo_id):
        """读取舵机输入电压 (LX 直连协议)"""
        if not self.is_connected():
            return None
        try:
            self.serial.flushInput()
            self._lx_send_cmd(servo_id, LX_CMD_VIN_READ)
            resp = self._lx_read_response(timeout=0.15)
            if resp and len(resp) >= 8:
                vin = resp[5] | (resp[6] << 8)
                return vin  # mV
            return None
        except serial.SerialException:
            return None

    def lx_read_temp(self, servo_id):
        """读取舵机温度 (LX 直连协议)"""
        if not self.is_connected():
            return None
        try:
            self.serial.flushInput()
            self._lx_send_cmd(servo_id, LX_CMD_TEMP_READ)
            resp = self._lx_read_response(timeout=0.15)
            if resp and len(resp) >= 7:
                return resp[5]  # 温度 (摄氏度)
            return None
        except serial.SerialException:
            return None

    def lx_move_servo(self, servo_id, position, move_time=1000):
        """控制舵机移动 (LX 直连协议)
        position: 0-1000
        move_time: 0-30000 ms
        """
        if not self.is_connected():
            print("[X] 串口未连接")
            return False
        position = max(0, min(1000, position))
        move_time = max(0, min(30000, move_time))
        params = bytearray()
        params.extend(position.to_bytes(2, 'little'))
        params.extend(move_time.to_bytes(2, 'little'))
        try:
            self._lx_send_cmd(servo_id, LX_CMD_SERVO_MOVE_TIME_WRITE, params)
            return True
        except serial.SerialException as e:
            print(f"[X] 发送指令失败: {e}")
            return False


# ============================================================
# 测试函数 (Test Functions)
# ============================================================
def scan_servo_ids(port):
    """扫描所有舵机ID，尝试两种协议和多种波特率"""
    print(f"\n{'='*55}")
    print(f"  [ID-SCAN] 舵机 ID 扫描 - 串口: {port}")
    print(f"{'='*55}\n")

    found_servos = []

    # --- 方法 1: LX 直连协议 (多种波特率) ---
    bauds_to_try = [115200, 9600]
    for baud in bauds_to_try:
        print(f"\n[方法 1] LX 直连协议 (波特率 {baud})")
        print("-" * 45)

        ctrl = ServoController(port, baud=baud, timeout=0.1)
        if not ctrl.connect():
            print(f"  [X] 无法打开串口 (波特率 {baud})")
            continue

        # 1a. 广播读取 ID (如果只连了一个舵机)
        print(f"  [广播] 发送广播读 ID (ID=254)...", end=" ", flush=True)
        resp_id = ctrl.lx_read_id(LX_SERVO_BROADCAST_ID)
        if resp_id is not None:
            print(f"OK! 检测到舵机, ID = {resp_id}")
            # 读取更多信息
            pos = ctrl.lx_read_position(resp_id)
            vin = ctrl.lx_read_vin(resp_id)
            temp = ctrl.lx_read_temp(resp_id)
            info = f"    -> 位置: {pos}" if pos is not None else ""
            info += f", 电压: {vin}mV" if vin is not None else ""
            info += f", 温度: {temp}C" if temp is not None else ""
            if info:
                print(info)
            found_servos.append({
                'id': resp_id, 'baud': baud, 'protocol': 'LX-direct',
                'position': pos, 'voltage': vin, 'temperature': temp
            })
        else:
            print("无回复")

        # 1b. 逐个扫描 ID 1-20 (读取位置来检测)
        print(f"  [逐个] 扫描 ID 1~20...", flush=True)
        for sid in range(1, 21):
            # 跳过已发现的
            if any(s['id'] == sid and s['baud'] == baud for s in found_servos):
                continue
            pos = ctrl.lx_read_position(sid)
            if pos is not None:
                vin = ctrl.lx_read_vin(sid)
                temp = ctrl.lx_read_temp(sid)
                print(f"    [!] 发现舵机 ID={sid}, 位置={pos}", end="")
                print(f", 电压={vin}mV" if vin is not None else "", end="")
                print(f", 温度={temp}C" if temp is not None else "")
                found_servos.append({
                    'id': sid, 'baud': baud, 'protocol': 'LX-direct',
                    'position': pos, 'voltage': vin, 'temperature': temp
                })
        print(f"    扫描完成 (波特率 {baud})")

        ctrl.disconnect()

    # --- 方法 2: 控制器板协议 (9600) ---
    print(f"\n[方法 2] 控制器板协议 (波特率 9600)")
    print("-" * 45)

    ctrl = ServoController(port, baud=9600, timeout=0.1)
    if ctrl.connect():
        # 尝试读取电池电压 (验证控制器板是否在线)
        print("  [电压] 读取电池电压...", end=" ", flush=True)
        voltage = ctrl.get_battery_voltage()
        if voltage is not None:
            print(f"OK! 电压: {voltage}mV ({voltage/1000:.2f}V)")
            print("  [->] 控制器板在线!")
        else:
            print("无回复 (控制器板可能不在线)")

        ctrl.disconnect()
    else:
        print("  [X] 无法打开串口")

    # --- 结果汇总 ---
    print(f"\n{'='*55}")
    if found_servos:
        print(f"  [结果] 共发现 {len(found_servos)} 个舵机:\n")
        print(f"  {'ID':<6} {'波特率':<10} {'协议':<12} {'位置':<8} {'电压(mV)':<10} {'温度(C)'}")
        print(f"  {'--':<6} {'------':<10} {'----':<12} {'----':<8} {'--------':<10} {'------'}")
        for s in found_servos:
            pos_str = str(s['position']) if s['position'] is not None else '-'
            vin_str = str(s['voltage']) if s['voltage'] is not None else '-'
            temp_str = str(s['temperature']) if s['temperature'] is not None else '-'
            print(f"  {s['id']:<6} {s['baud']:<10} {s['protocol']:<12} {pos_str:<8} {vin_str:<10} {temp_str}")
    else:
        print("  [结果] 未发现任何舵机")
        print("\n  可能的原因:")
        print("    1. 舵机未接电源 (需要给舵机供电)")
        print("    2. 接线有误 (检查 TX/RX 是否正确连接)")
        print("    3. 波特率不匹配 (默认试了 9600 和 115200)")
        print("    4. 舵机 ID 超出扫描范围 (当前扫描 1-20)")
    print(f"{'='*55}")
    return found_servos

def change_servo_id(port, new_id, baud=115200):
    """修改连接的舵机 ID"""
    print(f"\n{'='*50}")
    print(f"  [SET ID] 修改舵机 ID - 串口: {port}")
    print(f"{'='*50}\n")
    print(f"警告：请确保总线上【只连接了一个舵机】，否则所有舵机都会被修改为 ID {new_id}！")
    
    # 尝试不同的波特率来寻找舵机
    test_bauds = [115200, 9600]
    if baud not in test_bauds:
        test_bauds.insert(0, baud)
        
    ctrl = None
    current_id = None
    working_baud = None
    
    print("  [1] 正在通过广播地址(254)查找当前舵机 ID...")
    for baud in test_bauds:
        print(f"      -> 尝试波特率 {baud}...", end=" ", flush=True)
        temp_ctrl = ServoController(port, baud=baud)
        if not temp_ctrl.connect():
            print("失败(无法打开串口)")
            continue
            
        cid = temp_ctrl.lx_read_id(LX_SERVO_BROADCAST_ID)
        if cid is not None:
            print(f"成功! 找到舵机 ID: {cid}")
            current_id = cid
            working_baud = baud
            ctrl = temp_ctrl
            break
        else:
            print("无回复")
            temp_ctrl.disconnect()
            
    if current_id is None or ctrl is None:
        print("\n  [X] 失败! 在所有波特率下均未检测到舵机。")
        return False

    if current_id == new_id:
        print(f"\n  [>] 舵机 ID 已经是 {new_id}，无需修改。")
        ctrl.disconnect()
        return True

    print(f"\n  [2] 正在将舵机 ID 从 {current_id} 修改为 {new_id}...", end=" ", flush=True)
    if ctrl.lx_write_id(current_id, new_id):
        print("写入完成!")
    else:
        print("写入失败!")
        ctrl.disconnect()
        return False
        
    time.sleep(0.5)
    
    print(f"  [3] 验证修改结果...", end=" ", flush=True)
    verify_id = ctrl.lx_read_id(LX_SERVO_BROADCAST_ID)
    if verify_id == new_id:
        print(f"成功! 舵机 ID 已永久修改为: {verify_id}")
    else:
        print(f"失败! 读取到的 ID 仍为: {verify_id}")

    ctrl.disconnect()
    print(f"\n{'='*50}")
    return True


def test_connection(port):
    """测试串口连接"""
    print(f"\n{'='*50}")
    print(f"  [TEST] 舵机连接测试 - 串口: {port}")
    print(f"{'='*50}\n")

    ctrl = ServoController(port)

    # Step 1: 打开串口
    print("[1] 打开串口...", end=" ")
    if ctrl.connect():
        print(f"OK! ({ctrl.baud} baud)")
    else:
        print("FAIL!")
        return False

    # Step 2: 检查串口状态
    print("[2] 检查串口状态...", end=" ")
    if ctrl.is_connected():
        info = ctrl.serial
        print("OK!")
        print(f"    - 端口: {info.port}")
        print(f"    - 波特率: {info.baudrate}")
        print(f"    - 数据位: {info.bytesize}")
        print(f"    - 停止位: {info.stopbits}")
        print(f"    - 校验: {info.parity}")
    else:
        print("FAIL! 端口未正常打开")
        return False

    # Step 3: 尝试读取电池电压
    print("[3] 读取电池电压...", end=" ")
    voltage = ctrl.get_battery_voltage()
    if voltage is not None:
        print(f"OK! 电压: {voltage}mV ({voltage/1000:.2f}V)")
    else:
        print("WARN: 未收到回复 (可能控制器未上电或未连接)")

    # Step 4: 发送舵机居中指令
    print("[4] 发送居中指令 (ID=1, 位置=500, 时间=1000ms)...", end=" ")
    if ctrl.move_servo(1, 500, 1000):
        print("OK! 指令已发送")
        print("    >>> 请观察舵机是否移动到中间位置...")
        time.sleep(2)
    else:
        print("FAIL! 发送失败")

    # 断开连接
    ctrl.disconnect()
    print(f"\n{'='*50}")
    print("  [DONE] 连接测试完成!")
    print(f"{'='*50}")
    return True


def test_movement(port):
    """测试舵机来回移动 (LX 直连协议, 115200 波特率)"""
    print(f"\n{'='*50}")
    print(f"  [MOVE] 舵机移动测试 - 串口: {port}")
    print(f"{'='*50}\n")

    ctrl = ServoController(port, baud=115200)

    if not ctrl.connect():
        print("[X] 无法连接串口")
        return False

    # 先确认舵机在线
    print("[CHECK] 确认舵机在线...", end=" ", flush=True)
    sid = ctrl.lx_read_id(LX_SERVO_BROADCAST_ID)
    if sid is not None:
        pos = ctrl.lx_read_position(sid)
        print(f"OK! ID={sid}, 当前位置={pos}")
    else:
        print("WARN: 广播无回复, 将默认使用 ID=1")
        sid = 1

    print(f"\n将控制 ID={sid} 的舵机在 0 -> 500 -> 1000 之间来回移动")
    print("按 Ctrl+C 停止\n")

    try:
        cycle = 0
        while True:
            cycle += 1

            for target in [0, 500, 1000]:
                print(f"  [循环 {cycle}] -> 位置 {target}...", end=" ", flush=True)
                ctrl.lx_move_servo(sid, target, 1000)
                time.sleep(1.2)  # 等待移动完成
                actual = ctrl.lx_read_position(sid)
                if actual is not None:
                    print(f"OK (实际位置: {actual})")
                else:
                    print("OK (无法读取位置)")
                time.sleep(0.8)

    except KeyboardInterrupt:
        print("\n\n[STOP] 已停止!")
        print("[RESET] 复位到中间位置 500...")
        ctrl.lx_move_servo(sid, 500, 500)
        time.sleep(1)

    ctrl.disconnect()
    print("[DONE] 测试完成!\n")
    return True


def test_all_controller_apis(port, baud=9600):
    """测试所有的控制器板 API (CMD_SERVO_MOVE, 等)"""
    print(f"\n{'='*50}")
    print(f"  [ALL APIs] 控制器板 API 综合测试 - 串口: {port}")
    print(f"{'='*50}\n")
    
    ctrl = ServoController(port, baud=baud)
    if not ctrl.connect():
        print("[X] 无法连接串口")
        return False

    print("[1] 测试控制器连通性 (读取电压)...")
    voltage = ctrl.get_battery_voltage()
    if voltage is not None:
        print(f"    -> OK! 当前电压: {voltage}mV\n")
    else:
        print("    -> FAIL! 未收到回复，请检查连线。继续测试指令发送...\n")

    print("[2] 测试单/多舵机控制 (CMD_SERVO_MOVE)...")
    print("    -> 所有的舵机(1,2) 快速移动到 500 (时间: 500ms)")
    ctrl.move_multiple_servos({1: 500, 2: 500}, 500)
    time.sleep(1)

    print("    -> 舵机 1 移动到 200, 舵机 2 移动到 800 (时间: 1000ms)")
    ctrl.move_multiple_servos({1: 200, 2: 800}, 1000)
    time.sleep(1.5)

    print("    -> 读取舵机 1, 2 位置 (CMD_MULT_SERVO_POS_READ)...")
    positions = ctrl.read_multiple_positions([1, 2])
    print(f"    -> 返回结果: {positions}\n")

    print("[3] 测试动作组控制 (需在控制器端下载过动作组)...")
    print("    -> 调整动作组 0 速度到 150% (CMD_ACTION_GROUP_SPEED)...")
    ctrl.set_action_group_speed(0, 150)
    time.sleep(0.5)

    print("    -> 运行动作组 0 执行 1 次 (CMD_ACTION_GROUP_RUN)...")
    ctrl.run_action_group(0, 1)
    time.sleep(2)

    print("    -> 强行停止动作组 (CMD_ACTION_GROUP_STOP)...")
    ctrl.stop_action_group()
    time.sleep(0.5)
    print("    -> 动作组测试结束\n")

    print("[4] 测试舵机掉电/卸载 (CMD_MULT_SERVO_UNLOAD)...")
    print("    -> 卸载舵机 1, 2。卸载后舵机可以用手掰动。")
    ctrl.unload_servos([1, 2])
    time.sleep(0.5)
    
    ctrl.disconnect()
    print(f"\n{'='*50}")
    print("  [DONE] API 综合测试指令发送完成!")
    print(f"{'='*50}\n")
    return True


# ============================================================
# 主程序 (Main)
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description='舵机连接测试工具 (Servo Connection Tester)')
    parser.add_argument('--port', type=str, default=None,
                        help='串口端口 (例如 COM13 或 /dev/ttyUSB0)')
    parser.add_argument('--scan', action='store_true',
                        help='仅扫描可用串口')
    parser.add_argument('--move', action='store_true',
                        help='测试舵机来回移动')
    parser.add_argument('--id-scan', action='store_true',
                        help='扫描舵机ID (尝试多种协议和波特率)')
    parser.add_argument('--test-all-apis', action='store_true',
                        help='测试所有控制器板的 API 指令')
    parser.add_argument('--change-id', type=int, metavar='NEW_ID', default=None,
                        help='修改已连接舵机的 ID (警告：总线上只能接一个舵机)')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'波特率 (默认 {DEFAULT_BAUD})')

    args = parser.parse_args()

    print("\n" + "="*50)
    print("  LOBOT 舵机控制器测试工具")
    print("="*50)

    # 扫描串口
    if args.scan:
        scan_serial_ports()
        return

    # 确定使用的串口
    port = args.port
    if port is None:
        print("\n[AUTO] 自动检测串口...")
        ports = scan_serial_ports()
        port = find_servo_port()
        if port is None:
            print("\n[X] 未找到合适的串口, 请使用 --port 参数指定")
            sys.exit(1)
        print(f"\n[->] 自动选择串口: {port}")

    if args.change_id is not None:
        change_servo_id(port, args.change_id, baud=args.baud)
    elif args.id_scan:
        scan_servo_ids(port)
    elif args.move:
        test_movement(port)
    elif args.test_all_apis:
        test_all_controller_apis(port, baud=args.baud)
    else:
        test_connection(port)


if __name__ == '__main__':
    main()
