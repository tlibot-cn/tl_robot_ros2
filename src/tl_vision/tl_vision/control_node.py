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
from rclpy.executors import ExternalShutdownException

from scipy.spatial.transform import Rotation as R

from tl_ros2_interface.msg import ObjectInfo, CartesianPose, MoveCommand
from tl_ros2_interface.srv import CoordTransform, GetPosReachable
from std_srvs.srv import Trigger

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

        self.running = True
        self.cleaned = False

        self.latest_object_type = None
        self.latest_base_point = None
        self.latest_stamp = None

        # TCP 位姿缓存（来自 /tcp_pose 话题）
        self.latest_tcp_pose = None

        # 键盘控制相关
        self.old_terminal_settings = None
        self.keyboard_fd = None
        self.keyboard_fd_need_close = False
        self.setup_keyboard()

        self.T_tool_camera = self.load_handeye_matrix_from_param()

        self.safe_log_info('========== T_tool_camera ==========')
        self.safe_log_info(f'\n{self.T_tool_camera}')
        self.safe_log_info('===================================')

        # 创建 ROS2 服务客户端
        self._create_service_clients()

        # 创建 ROS2 话题订阅与发布
        self._create_topic_sub_pub()

        # 初始化机械臂（通过 ROS2 服务连接 + 上电）
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

    def _create_service_clients(self):
        """创建 tl_driver 服务的 ROS2 客户端。"""
        self.connect_client = self.create_client(Trigger, '/tl_driver/connect_arm')
        self.disconnect_client = self.create_client(Trigger, '/tl_driver/disconnect_arm')
        self.power_on_client = self.create_client(Trigger, '/tl_driver/power_on')
        self.power_off_client = self.create_client(Trigger, '/tl_driver/power_off')
        self.clear_error_client = self.create_client(Trigger, '/tl_driver/clear_error')
        self.coord_transform_client = self.create_client(CoordTransform, '/tl_driver/coord_transform')
        self.get_pos_reachable_client = self.create_client(GetPosReachable, '/tl_driver/get_pos_reachable')

    def _wait_for_services(self):
        """等待所有 tl_driver 服务就绪。"""
        timeout = 30.0
        clients = [
            ('connect_arm', self.connect_client),
            ('disconnect_arm', self.disconnect_client),
            ('power_on', self.power_on_client),
            ('power_off', self.power_off_client),
            ('clear_error', self.clear_error_client),
            ('coord_transform', self.coord_transform_client),
            ('get_pos_reachable', self.get_pos_reachable_client),
        ]

        start_time = time.time()
        for name, client in clients:
            remaining = timeout - (time.time() - start_time)
            if remaining <= 0:
                self.safe_log_error('等待 tl_driver 服务超时')
                return False
            self.safe_log_info(f'等待服务: /tl_driver/{name}')
            if not client.wait_for_service(timeout_sec=remaining):
                self.safe_log_error(f'服务 /tl_driver/{name} 不可用')
                return False

        self.safe_log_info('所有 tl_driver 服务已就绪')
        return True

    def _create_topic_sub_pub(self):
        """创建 TCP 位姿话题订阅与 MoveJ 运动指令发布器。"""
        self.tcp_pose_sub = self.create_subscription(
            CartesianPose,
            '/tcp_pose',
            self.tcp_pose_callback,
            10
        )
        self.movej_pub = self.create_publisher(
            MoveCommand,
            '/tl_driver/moveJ',
            10
        )

    def tcp_pose_callback(self, msg: CartesianPose):
        """缓存来自 /tcp_pose 话题的最新末端位姿。"""
        self.latest_tcp_pose = msg

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
        self.safe_log_info('等待 tl_driver 服务...')

        if not self._wait_for_services():
            self.safe_log_error('✗ tl_driver 服务不可用')
            self.cleanup_and_exit()

        self.safe_log_info(f'尝试连接机械臂: {self.robot_ip}:{self.robot_port}')

        try:
            future = self.connect_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            if not result.success:
                self.safe_log_error(f'✗ 机械臂连接失败: {result.message}')
                self.cleanup_and_exit()

            self.safe_log_info('✓ 机械臂连接成功')

            if not self.power_on_robot():
                self.safe_log_error('✗ 机械臂上电失败')
                self.cleanup_and_exit()

            self.safe_log_info('✓ 机械臂上电成功')

            time.sleep(1.0)

        except SystemExit:
            raise

        except Exception as e:
            self.safe_log_error(f'✗ 机械臂初始化异常: {e}')
            self.cleanup_and_exit()

    def power_on_robot(self):
        try:
            future = self.power_on_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            if result.success:
                self.safe_log_info('上电成功')
                return True

            self.safe_log_error(f'上电失败: {result.message}')
            return False

        except Exception as e:
            self.safe_log_error(f'上电过程发生异常: {e}')
            return False

    def power_off_robot(self):
        try:
            future = self.power_off_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            if result.success:
                self.safe_log_info('下电成功')
                return True

            self.safe_log_warn(f'下电失败: {result.message}')
            return False

        except Exception as e:
            self.safe_log_error(f'下电过程发生异常: {e}')
            return False

    def get_robot_pose(self):
        """从缓存的 /tcp_pose 话题数据获取当前末端位姿。"""
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 尚未接收到数据')
            return False, None

        try:
            # CartesianPose 位置单位为 mm，转换为米
            x = self.latest_tcp_pose.position.x / 1000.0
            y = self.latest_tcp_pose.position.y / 1000.0
            z = self.latest_tcp_pose.position.z / 1000.0

            # 从 CartesianPose.rpy 获取欧拉角（弧度）
            roll = self.latest_tcp_pose.rpy.x
            pitch = self.latest_tcp_pose.rpy.y
            yaw = self.latest_tcp_pose.rpy.z

            return True, [x, y, z, roll, pitch, yaw]

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
        """通过 /tl_driver/coord_transform 服务将笛卡尔位姿转为关节角度。"""
        req = CoordTransform.Request()
        req.origin_coord = 1  # 直角坐标
        req.target_coord = 0  # 关节坐标
        req.form = 0
        req.origin_pos = list(target_cart)
        req.reference_pos = [0.0] * 7

        try:
            future = self.coord_transform_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            if not result.success:
                self.safe_log_error(f'笛卡尔转关节失败: {result.message}')
                return -1, []

            return 0, list(result.target_pos)

        except Exception as e:
            self.safe_log_error(f'笛卡尔转关节异常: {e}')
            return -1, []

    def check_pos_reachable(self, point_values, coord=0, angle_unit=0, posture=1):
        """通过 /tl_driver/get_pos_reachable 服务检测关节位姿是否可达。"""
        # 构建 14 元素位姿数组: [coord, angle_unit, posture, 0,0,0,0, j1..j7]
        pos = [0.0] * 14
        pos[0] = float(coord)
        pos[1] = float(angle_unit)
        pos[2] = float(posture)
        for i in range(len(point_values)):
            pos[7 + i] = float(point_values[i])

        self.safe_log_info(f'可达性检测 pos14: {pos}')
        self.safe_log_info(f'可达性检测 movetype: {self.approach_movetype}')

        req = GetPosReachable.Request()
        req.pos = pos
        req.move_type = self.approach_movetype

        try:
            future = self.get_pos_reachable_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            self.safe_log_info(f'get_pos_reachable 返回: success={result.success}, message={result.message}')

            if not result.success:
                self.safe_log_warn(f'点位不可达: {result.message}')
                return False

            self.safe_log_info('点位可达')
            return True

        except Exception as e:
            self.safe_log_error(f'可达性判断异常: {e}')
            return False

    def movej(self, joint_pos):
        """通过 /tl_driver/moveJ 话题发送关节运动指令。"""
        try:
            joint_list = list(joint_pos)

            if len(joint_list) < 7:
                joint_list = joint_list + [0.0] * (7 - len(joint_list))

            cmd = MoveCommand()
            # target_pos_value 共 14 个元素，前 7 个为关节角度（关节模式）
            cmd.target_pos_value = joint_list[:7] + [0.0] * 7
            cmd.target_pos_type = 0
            cmd.coord = 0  # 关节坐标系
            cmd.velocity = float(self.speed)
            cmd.acc = float(self.acc)
            cmd.dec = float(self.dec)
            cmd.pl = int(self.pl)

            self.movej_pub.publish(cmd)

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

        return self.movej(zero_joint)

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

            except ExternalShutdownException:
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

        # 通过 ROS2 服务下电
        try:
            future = self.power_off_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if not result.success:
                print('机械臂下电未成功，但继续断开连接')
            else:
                print('机械臂下电成功')
        except Exception as e:
            print(f'下电异常: {e}')

        # 通过 ROS2 服务断开连接
        try:
            future = self.disconnect_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result.success:
                print('断开机械臂连接')
            else:
                print(f'断开连接异常: {result.message}')
        except Exception as e:
            print(f'断开连接异常: {e}')

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
