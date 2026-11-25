#!/usr/bin/env python3
"""
EG2-4C2 夹爪控制 - 通过电脑USB串口直接控制（不经过 tl_driver）

寄存器表:
  addr(hex)  addr(dec)  功能              值范围          说明
  ─────────────────────────────────────────────────────────────────
  0x01       1          CMD_SAVE          0/1             1=保存参数到Flash
  0x03       3          CMD_ID            0-254           夹爪设备ID
  0x05       5          CMD_CATCH_MODE    0/1             0=一次夹取 / 1=持续夹取
  0x06       6          CMD_STOP          0/1             1=急停
  0x07       7          CMD_FAULTACK      0/1             1=清除故障
  0x0A       10         CMD_OPENLEN_SET   0-1000          设置开口度(1000=70mm)
  0x0B       11         CMD_SPEED_SET     10-1000         设置速度
  0x0C       12         CMD_FORCE_SET     100-1000        设置夹持力(g)
  0x10       16         CMD_MAX_OPENLEN   0-1000          最大开口度限制
  0x11       17         CMD_MIN_OPENLEN   0-1000          最小开口度限制
  0x3D       61         CMD_OPENLEN_ACT   0-1000          实际开口度(只读)
  0x3E       62         CMD_CURRENT       0-2000          实际电流(只读)
  0x3F       63         CMD_TEMP          0-100           驱动器温度℃(只读)
  0x40       64         CMD_ERRORCODE     -               故障码(只读)
  0x41       65         CMD_STATUS        1-6             状态码(只读)

使用方法:
    sudo python3 zhua.py
"""

import os
import sys
import time
import struct

import serial


# ─────────────────────────── Modbus CRC16 ───────────────────────────

def _crc16(data: bytes) -> int:
    """计算 Modbus RTU CRC16"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ─────────────────────────── 寄存器地址 ───────────────────────────

REG_SAVE = 1          # 0x01
REG_ID = 3            # 0x03
REG_CATCH_MODE = 5    # 0x05
REG_STOP = 6          # 0x06
REG_FAULTACK = 7      # 0x07
REG_OPENLEN_SET = 10  # 0x0A
REG_SPEED_SET = 11    # 0x0B
REG_FORCE_SET = 12    # 0x0C
REG_MAX_OPENLEN = 16  # 0x10
REG_MIN_OPENLEN = 17  # 0x11
REG_OPENLEN_ACT = 61  # 0x3D (只读)
REG_CURRENT = 62      # 0x3E (只读)
REG_TEMP = 63         # 0x3F (只读)
REG_ERRORCODE = 64    # 0x40 (只读)
REG_STATUS = 65       # 0x41 (只读)


class EG2Gripper:
    """EG2-4C2 夹爪控制类 - 直连串口版"""

    def __init__(self, port: str = '/dev/ttyUSB0', slave_id: int = 1,
                 baudrate: int = 115200, debug: bool = False):
        self.slave_id = slave_id
        self.debug = debug
        self._port = port
        self._baudrate = baudrate
        self._bind_driver()
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
            write_timeout=1.0,
        )
        print(f'[初始化] 打开串口 {port} (从站ID={slave_id}, 波特率={baudrate})')

    def _bind_driver(self):
        """绑定 CH340 驱动（Kernel 6.8 需要）"""
        if os.path.exists(self._port):
            return
        # 先通过 new_id 尝试（最简单的方法）
        for path in ['/sys/bus/usb-serial/drivers/ch341-uart/new_id']:
            try:
                with open(path, 'w') as f:
                    f.write('1a86 7523')
                time.sleep(1)
            except Exception:
                pass
        if os.path.exists(self._port):
            return
        # 如果 new_id 没生效，用 unbind+bind 方式
        try:
            import glob
            for uevent in glob.glob('/sys/bus/usb/devices/*/uevent'):
                try:
                    with open(uevent) as f:
                        c = f.read()
                except Exception:
                    continue
                if '1a86' in c and '7523' in c and 'usb_interface' in c:
                    iface = uevent.split('/')[-2]
                    # 从 usbfs 解绑
                    try:
                        with open('/sys/bus/usb/drivers/usbfs/unbind', 'w') as f:
                            f.write(iface)
                    except Exception:
                        pass
                    time.sleep(0.3)
                    # 绑定到 ch341
                    try:
                        with open('/sys/bus/usb/drivers/ch341/bind', 'w') as f:
                            f.write(iface)
                    except Exception:
                        pass
                    time.sleep(1)
                    break
        except Exception:
            pass

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            print('[关闭] 串口已关闭')

    def _reconnect(self):
        """断开时自动重连：重新绑定驱动 + 重新打开串口"""
        print('[重连] 串口断开，正在重连...')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._bind_driver()
        if os.path.exists(self._port):
            try:
                self.ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1.0,
                    write_timeout=1.0,
                )
                print('[重连] 成功')
            except Exception as e:
                print(f'[重连] 失败: {e}')
        else:
            print('[重连] 失败: 设备未出现')

    # ──────────────── 底层 Modbus 读写 ────────────────

    def _send_receive(self, req: bytes, resp_len: int) -> bytes:
        """发送Modbus请求并接收响应（断线自动重连）"""
        for _ in range(3):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(req)
                time.sleep(0.05)
                resp = self.ser.read(resp_len)
                if len(resp) == 0:
                    return None
                return resp
            except Exception:
                self._reconnect()
        return None

    def _write_reg(self, addr: int, value: int) -> bool:
        """
        写单个保持寄存器 (Modbus功能码 0x06)
        帧格式: 从站ID | 0x06 | 地址高8 | 地址低8 | 值高8 | 值低8 | CRC低8 | CRC高8
        """
        frame = struct.pack('>BBHH', self.slave_id, 0x06, addr, value)
        crc = _crc16(frame)
        frame += struct.pack('<H', crc)

        if self.debug:
            print(f'  → 写寄存: addr=0x{addr:02X}({addr}), value={value}')
            print(f'    发送: {frame.hex()}')

        resp = self._send_receive(frame, 8)
        if resp is None:
            print(f'  ✖ 夹爪超时(addr={addr})')
            return False

        if self.debug:
            print(f'    接收: {resp.hex()}')

        if len(resp) < 5:
            print(f'  ✖ 夹爪响应太短: {len(resp)} 字节')
            return False

        if resp[0] != self.slave_id:
            print(f'  ✖ 从站ID不匹配: 期望{self.slave_id}, 收到{resp[0]}')
            return False

        if resp[1] == 0x86:  # 异常响应 (功能码 + 0x80)
            err_code = resp[2] if len(resp) > 2 else '?'
            print(f'  ✖ 夹爪异常: 错误码={err_code}')
            return False

        if resp[1] != 0x06:
            print(f'  ✖ 功能码不匹配: 期望0x06, 收到0x{resp[1]:02X}')
            return False

        return True

    def _read_reg(self, addr: int):
        """
        读保持寄存器 (Modbus功能码 0x03)
        帧格式: 从站ID | 0x03 | 地址高8 | 地址低8 | 数量高8 | 数量低8 | CRC低8 | CRC高8
        响应:   从站ID | 0x03 | 字节数 | 数据高8 | 数据低8 | CRC低8 | CRC高8
        """
        frame = struct.pack('>BBHH', self.slave_id, 0x03, addr, 1)
        crc = _crc16(frame)
        frame += struct.pack('<H', crc)

        if self.debug:
            print(f'  → 读寄存: addr=0x{addr:02X}({addr})')
            print(f'    发送: {frame.hex()}')

        resp = self._send_receive(frame, 7)  # 读1个寄存器响应7字节
        if resp is None:
            print(f'  ✖ 夹爪超时(addr={addr})')
            return None

        if self.debug:
            print(f'    接收: {resp.hex()}')

        if len(resp) < 5:
            print(f'  ✖ 夹爪响应太短: {len(resp)} 字节')
            return None

        if resp[0] != self.slave_id:
            print(f'  ✖ 从站ID不匹配: 期望{self.slave_id}, 收到{resp[0]}')
            return None

        if resp[1] == 0x83:  # 异常
            err_code = resp[2] if len(resp) > 2 else '?'
            print(f'  ✖ 夹爪异常: 错误码={err_code}')
            return None

        if resp[1] != 0x03:
            print(f'  ✖ 功能码不匹配: 期望0x03, 收到0x{resp[1]:02X}')
            return None

        # 解析数据: 字节数(1) + 数据(2)
        value = struct.unpack('>H', resp[3:5])[0]
        if self.debug:
            print(f'  ✓ 值={value}')
        return value

    # ──────────────── 高级控制接口 ────────────────

    def set_speed(self, speed: int):
        speed = max(10, min(1000, speed))
        if self.debug:
            print(f'[设置速度] {speed}')
        return self._write_reg(REG_SPEED_SET, speed)

    def set_force(self, force_g: int):
        force_g = max(100, min(1000, force_g))
        if self.debug:
            print(f'[设置夹持力] {force_g}g')
        return self._write_reg(REG_FORCE_SET, force_g)

    def set_catch_mode(self, continuous: bool):
        mode = '持续夹取' if continuous else '一次夹取'
        if self.debug:
            print(f'[设置夹取模式] {mode}')
        return self._write_reg(REG_CATCH_MODE, 1 if continuous else 0)

    def set_opening(self, value: int):
        value = max(0, min(1000, value))
        if self.debug:
            mm = value * 70 / 1000
            print(f'[开口度] {value} ({mm:.1f}mm)')
        return self._write_reg(REG_OPENLEN_SET, value)

    def open_gripper(self, opening: int = 1000):
        if self.debug:
            print(f'[打开夹爪]')
        return self.set_opening(opening)

    def close_gripper(self, opening: int = 0):
        if self.debug:
            print(f'[闭合夹爪]')
        return self.set_opening(opening)

    def emergency_stop(self):
        print(f'[急停]')
        return self._write_reg(REG_STOP, 1)

    def clear_fault(self):
        print(f'[清除故障]')
        return self._write_reg(REG_FAULTACK, 1)

    def read_position(self):
        return self._read_reg(REG_OPENLEN_ACT)

    def read_status_code(self):
        return self._read_reg(REG_STATUS)

    def read_temperature(self):
        return self._read_reg(REG_TEMP)

    def read_error_code(self):
        return self._read_reg(REG_ERRORCODE)

    def read_current(self):
        print(f'[读取电流]')
        return self._read_reg(REG_CURRENT)

    def read_all_status(self):
        print(f'\n--- 读取全部状态 ---')
        pos = self.read_position()
        st = self.read_status_code()
        temp = self.read_temperature()
        err = self.read_error_code()
        cur = self.read_current()
        print(f'--- 状态汇总 ---')
        return {
            'actual_position': pos,
            'status_code': st,
            'temperature':  temp,
            'error_code':   err,
            'current':      cur,
        }


def main():
    # ── 串口参数（按实际情况修改）──
    SERIAL_PORT = '/dev/ttyUSB0'  # USB转485的设备路径
    SLAVE_ID = 1                  # 夹爪从站ID
    BAUDRATE = 115200             # 波特率

    try:
        gripper = EG2Gripper(
            port=SERIAL_PORT,
            slave_id=SLAVE_ID,
            baudrate=BAUDRATE,
        )
    except Exception as e:
        print(f'  ✖ 夹爪初始化失败: {e}')
        print('  请检查:')
        print(f'    1. 夹爪 USB 是否已连接 (设备: {SERIAL_PORT})')
        print('    2. 是否有权限访问串口 (需 sudo 或加入 dialout 组)')
        print('    3. 如为新电脑，USB 转串口芯片可能不同，请确认 /dev/ttyUSB* 设备')
        return

    print("\n╔══════════════════════════════════════╗")
    print("║     EG2-4C2 夹爪控制终端(直连串口)    ║")
    print("╠══════════════════════════════════════╣")
    print("║  1 - 打开夹爪(全开)                   ║")
    print("║  2 - 闭合夹爪(全闭)                   ║")
    print("║  3 - 打开到指定位置                   ║")
    print("║  4 - 闭合到指定位置                   ║")
    print("║  5 - 设置速度                        ║")
    print("║  6 - 设置夹持力                      ║")
    print("║  7 - 设置夹取模式(持续/一次)           ║")
    print("║  8 - 读取全部状态                     ║")
    print("║  9 - 急停                            ║")
    print("║ 10 - 清除故障                        ║")
    print("║  0 - 退出                            ║")
    print("╚══════════════════════════════════════╝")

    try:
        while True:
            cmd = input("\n>>> 请输入指令: ").strip()

            if cmd == '0':
                break
            elif cmd == '1':
                gripper.open_gripper()
            elif cmd == '2':
                gripper.close_gripper()
            elif cmd == '3':
                try:
                    v = int(input("  开口度(0~1000, 1000=全开70mm): "))
                    gripper.open_gripper(v)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '4':
                try:
                    v = int(input("  开口度(0~1000, 0=全闭): "))
                    gripper.close_gripper(v)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '5':
                try:
                    s = int(input("  速度(10~1000): "))
                    gripper.set_speed(s)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '6':
                try:
                    f = int(input("  夹持力(100~1000g): "))
                    gripper.set_force(f)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '7':
                try:
                    m = int(input("  模式(0=一次夹取, 1=持续夹取): "))
                    gripper.set_catch_mode(m == 1)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '8':
                st = gripper.read_all_status()
                if st:
                    print(f"  实际开口度: {st['actual_position']}")
                    print(f"  状态码    : {st['status_code']}")
                    print(f"  温度      : {st['temperature']} ℃")
                    print(f"  故障码    : {st['error_code']}")
                    print(f"  电流      : {st['current']} mA")
            elif cmd == '9':
                gripper.emergency_stop()
            elif cmd == '10':
                gripper.clear_fault()
            else:
                print("  ✖ 无效指令")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        gripper.close()


if __name__ == '__main__':
    main()
