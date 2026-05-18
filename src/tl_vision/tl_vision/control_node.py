#!/usr/bin/env python3
import signal
import os
import sys
import time
import select
import termios
import tty
import numpy as np

import rclpy
from rclpy.node import Node

from scipy.spatial.transform import Rotation as R

from tl_ros2_interface.msg import ObjectInfo

nrc_lib_path = os.path.expanduser("~/tl_robot/src/tl_driver/lib")
if nrc_lib_path not in sys.path:
    sys.path.append(nrc_lib_path)

import nrc_interface as nrc

np.set_printoptions(precision=8, suppress=True)

class ObjectToBaseNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('robot_port', '6001')
        self.declare_parameter('camera_object_topic', '/tl_vision/object_3d_pos_camera')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('object_type', 'scissors')
        self.declare_parameter('grasp_offset', 0.10)
        self.declare_parameter('speed', 20.0)
        self.declare_parameter('approach_movetype', 'MOVJ')
        self.declare_parameter(
            'zero_joint',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            'handeye_matrix',
            [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            ]
        )

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.camera_object_topic = self.get_parameter('camera_object_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.target_object_type = str(self.get_parameter('object_type').value)
        self.grasp_offset = float(self.get_parameter('grasp_offset').value)
        self.speed = float(self.get_parameter('speed').value)
        self.approach_movetype = str(self.get_parameter('approach_movetype').value)
        self.zero_joint = list(self.get_parameter('zero_joint').value)

        # 固定运动参数，不作为 ROS 参数
        self.acc = 100.0
        self.dec = 100.0
        self.pl = 5

        self.socket_fd = None
        self.running = True
        self.cleaned = False

        self.latest_object_type = None
        self.latest_base_point = None
        self.latest_stamp = None

        # 键盘控制相关
        self.old_terminal_settings = None
        self.keyboard_fd = None
        self.keyboard_fd_need_close = False
        self.setup_keyboard()

        self.T_tool_camera = self.load_handeye_matrix_from_param()

        self.safe_log_info('========== T_tool_camera ==========')
        self.safe_log_info(f'\n{self.T_tool_camera}')
        self.safe_log_info('===================================')

        self.init_robot()

        self.sub_object_camera = self.create_subscription(
            ObjectInfo,
            self.camera_object_topic,
            self.object_camera_callback,
            10
        )

        self.pub_object_base = self.create_publisher(
            ObjectInfo,
            '/tl_vision/object_3d_pos_base',
            10
        )

        self.safe_log_info('control_node 初始化完成')
        self.safe_log_info(f'订阅: {self.camera_object_topic}')
        self.safe_log_info(f'base_frame_id: {self.base_frame_id}')
        self.safe_log_info(f'机械臂: {self.robot_ip}:{self.robot_port}')
        self.safe_log_info(f'目标类型 object_type: {self.target_object_type}')
        self.safe_log_info(f'grasp_offset: {self.grasp_offset} m')
        self.safe_log_info(f'MoveCmd.velocity speed: {self.speed}')
        self.safe_log_info(f'approach_movetype: {self.approach_movetype}')
        self.safe_log_info('按键: c 执行控制 | z 回零点 | q 退出')

    def safe_log_info(self, msg):
        if rclpy.ok():
            try:
                self.get_logger().info(str(msg))
            except Exception:
                print(msg)
        else:
            print(msg)

    def safe_log_warn(self, msg):
        if rclpy.ok():
            try:
                self.get_logger().warn(str(msg))
            except Exception:
                print(f'[WARN] {msg}')
        else:
            print(f'[WARN] {msg}')

    def safe_log_error(self, msg):
        if rclpy.ok():
            try:
                self.get_logger().error(str(msg))
            except Exception:
                print(f'[ERROR] {msg}')
        else:
            print(f'[ERROR] {msg}')

    def setup_keyboard(self):
        """
        修复 ros2 launch 下 sys.stdin 可能不是 tty，导致按 c 没反应的问题。

        优先使用 sys.stdin。
        如果 sys.stdin 不是 tty，则尝试打开 /dev/tty。
        """
        try:
            if sys.stdin.isatty():
                self.keyboard_fd = sys.stdin.fileno()
                self.keyboard_fd_need_close = False
            else:
                try:
                    self.keyboard_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
                    self.keyboard_fd_need_close = True
                except Exception:
                    self.keyboard_fd = None
                    self.keyboard_fd_need_close = False

            if self.keyboard_fd is None:
                self.safe_log_warn('未检测到可用终端，键盘控制不可用')
                return

            self.old_terminal_settings = termios.tcgetattr(self.keyboard_fd)
            tty.setcbreak(self.keyboard_fd)

            self.safe_log_info('键盘控制已启用: c 控制 | z 回零点 | q 退出')

        except Exception as e:
            print(f'键盘初始化失败: {e}')
            self.keyboard_fd = None
            self.keyboard_fd_need_close = False
            self.old_terminal_settings = None

    def restore_keyboard(self):
        try:
            if self.keyboard_fd is not None and self.old_terminal_settings is not None:
                try:
                    termios.tcsetattr(
                        self.keyboard_fd,
                        termios.TCSADRAIN,
                        self.old_terminal_settings
                    )
                except Exception:
                    pass

                self.old_terminal_settings = None

            if self.keyboard_fd is not None and self.keyboard_fd_need_close:
                try:
                    os.close(self.keyboard_fd)
                except Exception:
                    pass

            self.keyboard_fd = None
            self.keyboard_fd_need_close = False

        except Exception as e:
            print(f'恢复键盘异常: {e}')

    def read_key(self):
        try:
            if self.keyboard_fd is None:
                return None

            dr, _, _ = select.select([self.keyboard_fd], [], [], 0)

            if dr:
                data = os.read(self.keyboard_fd, 1)
                if data:
                    return data.decode(errors='ignore')

        except Exception:
            return None

        return None

    def make_vector_double(self, values, size=None):
        values = list(values)

        if size is None:
            size = len(values)

        try:
            vec = nrc.VectorDouble(size)
            for i in range(size):
                vec[i] = float(values[i]) if i < len(values) else 0.0
            return vec
        except Exception:
            pass

        vec = nrc.VectorDouble()
        for i in range(size):
            value = float(values[i]) if i < len(values) else 0.0
            try:
                vec.push_back(value)
            except Exception:
                vec.append(value)

        return vec

    def load_handeye_matrix_from_param(self):
        try:
            handeye_list = list(self.get_parameter('handeye_matrix').value)

            if len(handeye_list) != 16:
                raise ValueError(
                    f'handeye_matrix 参数长度错误，期望 16，实际 {len(handeye_list)}'
                )

            T_tool_camera = np.array(
                handeye_list,
                dtype=np.float64
            ).reshape(4, 4)

            if not np.all(np.isfinite(T_tool_camera)):
                raise ValueError(
                    f'handeye_matrix 中存在 NaN 或 Inf:\n{T_tool_camera}'
                )

            expected_last_row = np.array(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float64
            )

            if not np.allclose(T_tool_camera[3, :], expected_last_row, atol=1e-6):
                self.safe_log_warn(
                    f'handeye_matrix 最后一行不是 [0, 0, 0, 1]: '
                    f'{T_tool_camera[3, :]}'
                )

            return T_tool_camera

        except Exception as e:
            self.safe_log_error(f'读取 handeye_matrix 失败: {e}')
            self.cleanup_and_exit()

    def init_robot(self):
        self.safe_log_info(f'尝试连接机械臂: {self.robot_ip}:{self.robot_port}')

        try:
            self.socket_fd = nrc.connect_robot(
                self.robot_ip,
                self.robot_port
            )

            if self.socket_fd <= 0:
                self.safe_log_error(
                    f'✗ 机械臂连接失败，返回值: {self.socket_fd}'
                )
                self.cleanup_and_exit()

            self.safe_log_info(
                f'✓ 机械臂连接成功，socketfd: {self.socket_fd}'
            )

            if not self.power_on_robot():
                self.safe_log_error('✗ 机械臂上电失败')
                self.cleanup_and_exit()

            self.safe_log_info('✓ 机械臂上电成功')

            # 按你的要求：
            # 这里不调用 nrc.set_current_mode()
            # 这里不调用 nrc.set_speed()

            time.sleep(1.0)

        except SystemExit:
            raise

        except Exception as e:
            self.safe_log_error(f'✗ 机械臂初始化异常: {e}')
            self.cleanup_and_exit()

    def power_on_robot(self):
        if self.socket_fd is None:
            self.safe_log_error('socket_fd 为空，无法上电')
            return False

        try:
            ret, state = nrc.get_servo_state(self.socket_fd, -1)

            if ret != 0:
                self.safe_log_error(f'获取伺服状态失败，返回码: {ret}')
                return False

            self.safe_log_info(f'当前伺服状态: {state}')

            if state == 0:
                self.safe_log_info('状态0: 执行使能并上电')
                nrc.set_servo_state(self.socket_fd, 1)
                time.sleep(0.1)
                nrc.set_servo_poweron(self.socket_fd)

            elif state == 1:
                self.safe_log_info('状态1: 直接上电')
                nrc.set_servo_poweron(self.socket_fd)

            elif state == 2:
                self.safe_log_info('状态2: 清除错误，重新使能并上电')
                nrc.clear_error(self.socket_fd)
                time.sleep(0.1)
                nrc.set_servo_state(self.socket_fd, 1)
                time.sleep(0.1)
                nrc.set_servo_poweron(self.socket_fd)

            elif state == 3:
                self.safe_log_info('状态3: 机械臂已上电')
                return True

            else:
                self.safe_log_error(f'未知的伺服状态: {state}')
                return False

            time.sleep(0.5)

            ret, state = nrc.get_servo_state(self.socket_fd, -1)

            if ret != 0:
                self.safe_log_error(f'上电后获取伺服状态失败，返回码: {ret}')
                return False

            if state == 3:
                self.safe_log_info('上电成功')
                return True

            self.safe_log_error(f'上电失败，当前状态: {state}')
            return False

        except Exception as e:
            self.safe_log_error(f'上电过程发生异常: {e}')
            return False

    def power_off_robot(self):
        if self.socket_fd is None:
            return True

        try:
            ret, state = nrc.get_servo_state(self.socket_fd, -1)

            if ret != 0:
                print(f'获取伺服状态失败，返回码: {ret}')
                return False

            print(f'当前伺服状态: {state}')

            if state == -1:
                print('机械臂已断开连接')
                return True

            elif state in [0, 1, 2]:
                print(f'状态{state}: 机械臂已下电')
                return True

            elif state == 3:
                print('状态3: 执行下电')
                ret_poweroff = nrc.set_servo_poweroff(self.socket_fd)

                if ret_poweroff is not None and ret_poweroff != 0:
                    print(f'下电指令返回异常: {ret_poweroff}，继续查询实际状态')

                time.sleep(0.8)

                ret, state = nrc.get_servo_state(self.socket_fd, -1)

                if ret != 0:
                    print(f'下电后获取伺服状态失败，返回码: {ret}')
                    return False

                print(f'下电后状态: {state}')

                if state != 3 and state != -1:
                    print('下电成功')
                    return True

                print(f'下电失败，当前状态: {state}')
                return False

            else:
                print(f'未知的伺服状态: {state}')
                return False

        except Exception as e:
            print(f'下电过程发生异常: {e}')
            return False

    def get_robot_pose(self):
        if self.socket_fd is None:
            self.safe_log_error('socket_fd 为空，无法获取机械臂位姿')
            return False, None

        try:
            tcp_pose = nrc.VectorDouble()

            ret = nrc.get_current_position(
                self.socket_fd,
                1,
                tcp_pose
            )

            if ret == 0:
                pose_raw = list(tcp_pose)

                if len(pose_raw) < 6:
                    self.safe_log_error(
                        f'机械臂返回位姿长度不足: {pose_raw}'
                    )
                    return False, None

                pose_converted = [
                    pose_raw[0] / 1000.0,
                    pose_raw[1] / 1000.0,
                    pose_raw[2] / 1000.0,
                    pose_raw[3],
                    pose_raw[4],
                    pose_raw[5],
                ]

                return True, pose_converted

            self.safe_log_error(f'获取位姿失败，返回码: {ret}')
            return False, None

        except Exception as e:
            self.safe_log_error(f'获取位姿失败: {e}')
            return False, None

    def pose_to_tool_rt(self, pose):
        if pose is None or len(pose) < 6:
            raise ValueError(f'pose 长度不足: {pose}')

        t_tool = np.array(
            pose[0:3],
            dtype=np.float64
        ).reshape(3, 1)

        A, B, C = pose[3], pose[4], pose[5]

        rotation = R.from_euler(
            'XYZ',
            [A, B, C],
            degrees=False
        )

        R_tool = rotation.as_matrix()

        return R_tool, t_tool

    def pose_to_T_base_tool(self, pose):
        R_tool, t_tool = self.pose_to_tool_rt(pose)

        T_base_tool = np.eye(4, dtype=np.float64)
        T_base_tool[0:3, 0:3] = R_tool
        T_base_tool[0:3, 3] = t_tool.reshape(3)

        return T_base_tool

    def object_camera_callback(self, msg):
        if not self.running:
            return

        try:
            object_type = msg.type

            P_camera = np.array(
                [
                    msg.pos.point.x,
                    msg.pos.point.y,
                    msg.pos.point.z,
                    1.0
                ],
                dtype=np.float64
            )

            if not np.all(np.isfinite(P_camera)):
                self.safe_log_warn(
                    f'收到非法物体坐标，type={object_type}，跳过'
                )
                return

            success, robot_pose = self.get_robot_pose()

            if not success:
                self.safe_log_warn(
                    f'机械臂位姿获取失败，type={object_type}，跳过本次转换'
                )
                return

            T_base_tool = self.pose_to_T_base_tool(robot_pose)
            T_base_camera = T_base_tool @ self.T_tool_camera
            P_base = T_base_camera @ P_camera

            if not np.all(np.isfinite(P_base)):
                self.safe_log_warn(
                    f'转换后物体坐标非法，type={object_type}，跳过'
                )
                return

            trans_msg = ObjectInfo()
            trans_msg.type = object_type
            trans_msg.pos.header.stamp = msg.pos.header.stamp
            trans_msg.pos.header.frame_id = self.base_frame_id
            trans_msg.pos.point.x = float(P_base[0])
            trans_msg.pos.point.y = float(P_base[1])
            trans_msg.pos.point.z = float(P_base[2])

            self.pub_object_base.publish(trans_msg)

            self.latest_object_type = str(object_type)
            self.latest_base_point = np.array(
                [P_base[0], P_base[1], P_base[2]],
                dtype=np.float64
            )
            self.latest_stamp = time.time()

        except Exception as e:
            self.safe_log_error(f'物体坐标转换失败: {e}')

    def get_target_joint_from_cartesian(self, target_cart):
        origin_pos = self.make_vector_double(target_cart, 7)
        target_pos = self.make_vector_double([], 7)

        origin_coord = 1
        target_coord = 0

        try:
            if hasattr(nrc, 'get_origin_coord_to_target_coord_robot'):
                ret = nrc.get_origin_coord_to_target_coord_robot(
                    self.socket_fd,
                    1,
                    origin_coord,
                    origin_pos,
                    target_coord,
                    target_pos
                )
                return ret, target_pos

            ret = nrc.get_origin_coord_to_target_coord(
                self.socket_fd,
                1,
                origin_coord,
                origin_pos,
                target_coord,
                target_pos
            )
            return ret, target_pos

        except Exception as e:
            self.safe_log_error(f'笛卡尔转关节异常: {e}')
            return -1, target_pos
    
    def make_reachable_pos14(self, point_values, coord=0, angle_unit=0, posture=1):
        values = list(point_values)

        pos = [0.0] * 14

        pos[0] = int(coord)
        pos[1] = int(angle_unit)
        pos[2] = int(posture)
        pos[3] = 0
        pos[4] = 0
        pos[5] = 0
        pos[6] = 0

        for i in range(7):
            if i < len(values):
                pos[7 + i] = float(values[i])
            else:
                pos[7 + i] = 0.0

        return pos

    def check_pos_reachable(self, point_values, coord=0, angle_unit=0, posture=1):
        try:
            pos = self.make_reachable_pos14(
                point_values,
                coord=coord,
                angle_unit=angle_unit,
                posture=posture
            )

            self.safe_log_info(f'可达性检测 pos14: {pos}')
            self.safe_log_info(f'可达性检测 movetype: {self.approach_movetype}')

            ret, result = nrc.get_pos_reachable(
                self.socket_fd,
                pos,
                self.approach_movetype,
                False
            )

            self.safe_log_info(f'get_pos_reachable 返回: ret={ret}, result={result}')

            if ret != 0:
                self.safe_log_warn(f'可达性判断接口失败: ret={ret}')
                return False

            if bool(result):
                self.safe_log_info('点位可达')
                return True

            self.safe_log_warn('点位不可达')
            return False

        except Exception as e:
            self.safe_log_error(f'可达性判断异常: {e}')
            return False

    def movej(self, joint_pos):
        try:
            joint_list = list(joint_pos)

            if len(joint_list) < 7:
                joint_list = joint_list + [0.0] * (7 - len(joint_list))

            move_cmd = nrc.MoveCmd()
            move_cmd.targetPosValue.resize(7)

            for i in range(7):
                move_cmd.targetPosValue[i] = float(joint_list[i])

            try:
                move_cmd.targetPosType = nrc.PosType_data
            except Exception:
                pass

            move_cmd.coord = 0
            move_cmd.velocity = float(self.speed)
            move_cmd.acc = float(self.acc)
            move_cmd.dec = float(self.dec)
            move_cmd.pl = int(self.pl)

            ret = nrc.robot_movej(self.socket_fd, move_cmd)

            if ret != 0:
                self.safe_log_error(f'robot_movej 失败: ret={ret}')
                return False

            self.safe_log_info('robot_movej 指令已发送')
            return True

        except Exception as e:
            self.safe_log_error(f'robot_movej 异常: {e}')
            return False

    def execute_control(self):
        if self.latest_base_point is None:
            self.safe_log_warn('当前没有检测到目标物体')
            return False

        if self.latest_object_type != self.target_object_type:
            self.safe_log_warn(
                f'当前物体类型为 {self.latest_object_type}，'
                f'目标类型为 {self.target_object_type}，不执行控制'
            )
            return False

        p = self.latest_base_point.copy()

        target_cart = [
            float(p[0] * 1000.0),
            float(p[1] * 1000.0),
            float((p[2] + self.grasp_offset) * 1000.0),
            -3.14,
            0.0,
            0.0,
            0.0
        ]

        self.safe_log_info(
            f'目标笛卡尔坐标: '
            f'[{target_cart[0]:.3f}, {target_cart[1]:.3f}, {target_cart[2]:.3f}, '
            f'{target_cart[3]:.3f}, {target_cart[4]:.3f}, {target_cart[5]:.3f}, {target_cart[6]:.3f}]'
        )

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)

        if ret != 0:
            self.safe_log_error(f'笛卡尔转关节失败: ret={ret}')
            return False

        joint_list = list(target_joint)

        if len(joint_list) < 7:
            self.safe_log_error(f'转换后的关节坐标长度不足: {joint_list}')
            return False

        self.safe_log_info(f'目标关节坐标: {joint_list}')

        if not self.check_pos_reachable(target_joint):
            self.safe_log_warn('目标位置不可达')
            return False

        self.safe_log_info('目标位置可达，开始运动')
        return self.movej(target_joint)

    def move_zero(self):
        if len(self.zero_joint) < 7:
            zero_joint = list(self.zero_joint) + [0.0] * (7 - len(self.zero_joint))
        else:
            zero_joint = list(self.zero_joint[:7])

        self.safe_log_info(f'回零点: {zero_joint}')

        if not self.check_pos_reachable(zero_joint):
            self.safe_log_warn('零点位置不可达')
            return False

        joint_pos = self.make_vector_double(zero_joint, 7)
        return self.movej(joint_pos)

    def handle_key(self, key):
        self.safe_log_info(f'收到按键: {repr(key)}')

        if key == 'c':
            self.safe_log_info('按键 c: 执行目标控制')
            self.execute_control()

        elif key == 'z':
            self.safe_log_info('按键 z: 回零点')
            self.move_zero()

        elif key == 'q':
            print('\n按键 q: 退出 control_node')
            self.running = False

    def run(self):
        while self.running:
            try:
                if not rclpy.ok():
                    break

                rclpy.spin_once(self, timeout_sec=0.05)

                key = self.read_key()
                if key is not None:
                    self.handle_key(key)

            except rclpy.executors.ExternalShutdownException:
                break

            except KeyboardInterrupt:
                print('\n用户中断 control_node')
                self.running = False
                break

            except Exception as e:
                if self.running and rclpy.ok():
                    self.safe_log_error(f'主循环异常: {e}')
                else:
                    print(f'主循环退出: {e}')
                break

        self.cleanup()

    def cleanup(self):
        if self.cleaned:
            return

        self.cleaned = True
        self.running = False

        print('\n正在清理 control_node 资源...')

        self.restore_keyboard()

        if self.socket_fd:
            poweroff_ok = self.power_off_robot()

            if not poweroff_ok:
                print('机械臂下电未成功，但继续断开连接')

            try:
                ret = nrc.disconnect_robot(self.socket_fd)

                if ret is not None and ret != 0:
                    print(f'断开机械臂连接返回异常: {ret}')
                else:
                    print('断开机械臂连接')

            except Exception as e:
                print(f'断开连接异常: {e}')

            self.socket_fd = None

        print('control_node 资源清理完成')

    def cleanup_and_exit(self):
        self.cleanup()
        raise SystemExit(1)

def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = ObjectToBaseNode()

        def signal_handler(sig, frame):
            print('\n收到中断信号，正在退出 control_node...')
            if node is not None:
                node.running = False

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        node.run()

    except KeyboardInterrupt:
        print('\n用户中断 control_node')

    except SystemExit:
        pass

    except Exception as e:
        print(f'control_node 程序异常: {e}')

    finally:
        if node is not None:
            try:
                node.cleanup()
            except Exception as e:
                print(f'清理资源异常: {e}')

            try:
                node.destroy_node()
            except Exception as e:
                print(f'销毁节点异常: {e}')

        if rclpy.ok():
            try:
                rclpy.shutdown()
                print('ROS2已关闭')
            except Exception as e:
                print(f'ROS2关闭异常: {e}')

if __name__ == '__main__':
    main()