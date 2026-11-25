#!/usr/bin/env python3
"""
EG2-4C2 夹爪控制 - 通过机械臂 Modbus 通信（夹爪接在机械臂上）

用法:
    ros2 run tl_vision test_gripper_arm
"""

import sys
import time
import rclpy
from rclpy.node import Node
from tl_ros2_interface.srv import ModbusWrite, ModbusRead
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam


# ─────────────────────────── Modbus 寄存器地址 ───────────────────────────
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


class ArmGripper:
    """通过机械臂 Modbus 控制夹爪（可嵌入ROS节点使用）"""

    def __init__(self, node, slave_id=1, port=2, baudrate=115200):
        from tl_ros2_interface.srv import ModbusWrite, ModbusRead
        from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
        import rclpy

        self._node = node
        self._modbus_write = node.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self._modbus_read = node.create_client(ModbusRead, '/tl_driver/modbus_read')

        for cli in [self._modbus_write, self._modbus_read]:
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('Modbus 服务不可用')

        mp = ModbusMasterParam()
        mp.type = 'RTU'
        mp.start_addr = True
        mp.rtu = ModbusRTUParam()
        mp.rtu.slave_id = slave_id
        mp.rtu.port = port
        mp.rtu.baudrate = baudrate
        mp.rtu.data_bit = 8
        mp.rtu.stop_bit = 1
        mp.rtu.check_bit = 'None'
        self._mp = mp

    def _write(self, addr, val, timeout=0.3):
        from tl_ros2_interface.srv import ModbusWrite
        import rclpy
        req = ModbusWrite.Request()
        req.master_id = 1
        req.addr = addr
        req.data = [val]
        req.master_param = self._mp
        f = self._modbus_write.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, f, timeout_sec=timeout)
        except Exception:
            pass
        return True  # 不等待响应，避免阻塞主循环

    def _read(self, addr, timeout=3.0):
        from tl_ros2_interface.srv import ModbusRead
        import rclpy
        req = ModbusRead.Request()
        req.master_id = 1
        req.addr = addr
        req.quantity = 1
        req.master_param = self._mp
        f = self._modbus_read.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, f, timeout_sec=timeout)
        except Exception:
            return None
        if f.done() and f.result() is not None and f.result().success:
            return f.result().data[0] if f.result().data else None
        return None

    def set_speed(self, s): return self._write(REG_SPEED_SET, max(10, min(1000, s)))
    def set_force(self, f): return self._write(REG_FORCE_SET, max(100, min(1000, f)))
    def set_catch_mode(self, c): return self._write(REG_CATCH_MODE, 1 if c else 0)
    def set_opening(self, v): return self._write(REG_OPENLEN_SET, max(0, min(1000, v)))
    def open_gripper(self, v=1000, opening=None): return self.set_opening(opening if opening is not None else v)
    def close_gripper(self, v=0, opening=None): return self.set_opening(opening if opening is not None else v)
    def emergency_stop(self): return self._write(REG_STOP, 1)
    def clear_fault(self): return self._write(REG_FAULTACK, 1)
    def read_position(self): return self._read(REG_OPENLEN_ACT)
    def read_status_code(self): return self._read(REG_STATUS)
    def read_temperature(self): return self._read(REG_TEMP)
    def read_error_code(self): return self._read(REG_ERRORCODE)
    def read_current(self): return self._read(REG_CURRENT)


class ArmGripperNode(Node):
    """通过机械臂 Modbus 控制夹爪的测试节点"""

    def __init__(self):
        super().__init__('test_gripper_arm')

        # ── Modbus RTU 参数（根据机械臂 RS485 端口配置）──
        self.declare_parameter('modbus_slave_id', 1)  # 从站ID（和样例一致）
        self.declare_parameter('modbus_port', 2)        # 串口号（和样例一致，不是1！）
        self.declare_parameter('modbus_baudrate', 115200)
        self.declare_parameter('modbus_data_bit', 8)
        self.declare_parameter('modbus_stop_bit', 1)
        self.declare_parameter('modbus_check_bit', 'None')  # 校验位（和样例一致）

        port = self.get_parameter('modbus_port').value
        baudrate = self.get_parameter('modbus_baudrate').value
        data_bit = self.get_parameter('modbus_data_bit').value
        stop_bit = self.get_parameter('modbus_stop_bit').value
        check_bit = self.get_parameter('modbus_check_bit').value
        slave_id = self.get_parameter('modbus_slave_id').value

        # ── 创建 Modbus 服务客户端 ──
        self._modbus_write_cli = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self._modbus_read_cli = self.create_client(ModbusRead, '/tl_driver/modbus_read')

        # 等待服务可用
        for cli_name, cli in [('modbus_write', self._modbus_write_cli),
                               ('modbus_read', self._modbus_read_cli)]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'{cli_name} 服务不可用，请确保 tl_driver 已启动')
                sys.exit(1)

        # ── 构建 Modbus Master 参数（和原生样例完全一致）──
        self._master_param = ModbusMasterParam()
        self._master_param.type = 'RTU'
        self._master_param.start_addr = True
        self._master_param.rtu = ModbusRTUParam()
        self._master_param.rtu.slave_id = slave_id
        self._master_param.rtu.port = port              # 关键：端口是2！
        self._master_param.rtu.baudrate = baudrate
        self._master_param.rtu.data_bit = data_bit
        self._master_param.rtu.stop_bit = stop_bit
        self._master_param.rtu.check_bit = check_bit    # 'None' 不是 'N'

        self.get_logger().info(f'夹爪测试节点已启动 (slave_id={slave_id}, port={port}, baudrate={baudrate})')

    # ──────────────── Modbus 读写 ────────────────

    def _write_reg(self, addr: int, value: int, timeout: float = 3.0) -> bool:
        """写单个保持寄存器"""
        req = ModbusWrite.Request()
        req.master_id = 1
        req.addr = addr
        req.data = [value]
        req.master_param = self._master_param

        future = self._modbus_write_cli.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        except Exception:
            print(f'  ✖ 写入超时(addr={addr})')
            return False
        resp = future.result()
        if resp.success:
            return True
        else:
            print(f'  ✖ 写入失败: {resp.message}')
            return False

    def _read_reg(self, addr: int, timeout: float = 3.0):
        """读单个保持寄存器"""
        req = ModbusRead.Request()
        req.master_id = 1
        req.addr = addr
        req.quantity = 1
        req.master_param = self._master_param

        future = self._modbus_read_cli.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        except Exception:
            print(f'  ✖ 读取超时(addr={addr})')
            return None
        resp = future.result()
        if resp.success and len(resp.data) > 0:
            return resp.data[0]
        else:
            print(f'  ✖ 读取失败: {resp.message}')
            return None
            return None

    # ──────────────── 夹爪控制接口 ────────────────

    def set_speed(self, speed: int):
        speed = max(10, min(1000, speed))
        print(f'[设置速度] {speed}')
        return self._write_reg(REG_SPEED_SET, speed)

    def set_force(self, force_g: int):
        force_g = max(100, min(1000, force_g))
        print(f'[设置夹持力] {force_g}g')
        return self._write_reg(REG_FORCE_SET, force_g)

    def set_catch_mode(self, continuous: bool):
        mode = '持续夹取' if continuous else '一次夹取'
        print(f'[设置夹取模式] {mode}')
        return self._write_reg(REG_CATCH_MODE, 1 if continuous else 0)

    def set_opening(self, value: int):
        value = max(0, min(1000, value))
        mm = value * 70 / 1000
        print(f'[开口度] {value} ({mm:.1f}mm)')
        return self._write_reg(REG_OPENLEN_SET, value)

    def open_gripper(self, opening: int = 1000):
        print(f'[打开夹爪]')
        return self.set_opening(opening)

    def close_gripper(self, opening: int = 0):
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
            'temperature': temp,
            'error_code': err,
            'current': cur,
        }


def main():
    rclpy.init()
    node = ArmGripperNode()

    print("\n╔══════════════════════════════════════╗")
    print("║  夹爪控制(通过机械臂Modbus)          ║")
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
        while rclpy.ok():
            cmd = input("\n>>> 请输入指令: ").strip()

            if cmd == '0':
                break
            elif cmd == '1':
                node.open_gripper()
            elif cmd == '2':
                node.close_gripper()
            elif cmd == '3':
                try:
                    v = int(input("  开口度(0~1000, 1000=全开70mm): "))
                    node.open_gripper(v)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '4':
                try:
                    v = int(input("  开口度(0~1000, 0=全闭): "))
                    node.close_gripper(v)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '5':
                try:
                    s = int(input("  速度(10~1000): "))
                    node.set_speed(s)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '6':
                try:
                    f = int(input("  夹持力(100~1000g): "))
                    node.set_force(f)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '7':
                try:
                    m = int(input("  模式(0=一次夹取, 1=持续夹取): "))
                    node.set_catch_mode(m == 1)
                except ValueError:
                    print("  ✖ 输入无效")
            elif cmd == '8':
                st = node.read_all_status()
                if st:
                    print(f"  实际开口度: {st['actual_position']}")
                    print(f"  状态码    : {st['status_code']}")
                    print(f"  温度      : {st['temperature']} ℃")
                    print(f"  故障码    : {st['error_code']}")
                    print(f"  电流      : {st['current']} mA")
            elif cmd == '9':
                node.emergency_stop()
            elif cmd == '10':
                node.clear_fault()
            else:
                print("  ✖ 无效指令")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
