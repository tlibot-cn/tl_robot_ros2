#!/usr/bin/env python3

import signal
import os
import sys
import time
import math
import select
import termios
import tty

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from scipy.spatial.transform import Rotation as R

from tl_ros2_interface.msg import ObjectInfo, CartesianPose
from tl_ros2_interface.srv import CoordTransform, GetPosReachable
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from tl_vision.zhua import EG2Gripper

np.set_printoptions(precision=8, suppress=True)


class ObjectToBaseNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('robot_port', '6001')
        self.declare_parameter('camera_object_topic', '/tl_vision/object_3d_pos_camera')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('object_type', 'scissors')
        self.declare_parameter('grasp_offset', 0.2)
        self.declare_parameter('grasp_forward_offset', 0.0)
        self.declare_parameter('approach_movetype', 'MOVJ')
        self.declare_parameter('servoj_interp_points', 2000)
        self.declare_parameter('tracking_interp_points', 1000)
        self.declare_parameter('servoj_vmax', [300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0])
        self.declare_parameter('servoj_amax', [3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0])
        self.declare_parameter('servoj_jmax', [50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0])
        self.declare_parameter('servoj_loss_timeout', 3.0)       # 丢失超时（秒）
        self.declare_parameter('servoj_arrival_threshold', 0.5)  # 到位判定阈值（度）
        self.declare_parameter('servoj_update_threshold', 0.5)   # 轨迹更新最小偏差，低于此值不更新（减少抖动）
        self.declare_parameter('tracking_min_height', 0.3)       # 视觉跟踪最低高度（米）
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

        # ── 夹爪参数（从 gripper.yaml 读取）──
        self.declare_parameter('gripper_port', '/dev/ttyUSB0')
        self.declare_parameter('gripper_slave_id', 1)
        self.declare_parameter('gripper_baudrate', 115200)
        self.declare_parameter('gripper_speed', 500)
        self.declare_parameter('gripper_force', 500)
        self.declare_parameter('gripper_catch_mode', 0)
        self.declare_parameter('gripper_open_position', 1000)
        self.declare_parameter('gripper_close_position', 0)
        self.declare_parameter('gripper_max_opening', 1000)
        self.declare_parameter('gripper_min_opening', 0)
        self.declare_parameter('gripper_grasp_wait_duration', 3.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.camera_object_topic = self.get_parameter('camera_object_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.target_object_type = str(self.get_parameter('object_type').value)
        self.grasp_offset = float(self.get_parameter('grasp_offset').value)
        self.grasp_forward_offset = float(self.get_parameter('grasp_forward_offset').value)
        self.approach_movetype = str(self.get_parameter('approach_movetype').value)
        self.zero_joint = list(self.get_parameter('zero_joint').value)

        # ── 夹爪参数 ──
        self.gripper_port = str(self.get_parameter('gripper_port').value)
        self.gripper_slave_id = int(self.get_parameter('gripper_slave_id').value)
        self.gripper_baudrate = int(self.get_parameter('gripper_baudrate').value)
        self.gripper_speed = int(self.get_parameter('gripper_speed').value)
        self.gripper_force = int(self.get_parameter('gripper_force').value)
        self.gripper_catch_mode = int(self.get_parameter('gripper_catch_mode').value)
        self.gripper_open_position = int(self.get_parameter('gripper_open_position').value)
        self.gripper_close_position = int(self.get_parameter('gripper_close_position').value)
        self.gripper_max_opening = int(self.get_parameter('gripper_max_opening').value)
        self.gripper_min_opening = int(self.get_parameter('gripper_min_opening').value)
        self.gripper_grasp_wait_duration = float(self.get_parameter('gripper_grasp_wait_duration').value)

        self.running = True
        self.cleaned = False
        self.tracking_mode = False            # 视觉跟随模式（按 c 切换）
        self._last_processed_stamp = 0.0      # 上次处理的 YOLO 时间戳
        self._loss_start_time = None          # 目标丢失起始时间
        self._loss_homed = False              # 丢失后是否已回零

        self.latest_object_type = None
        self.latest_base_point = None
        self.latest_stamp = None

        # TCP 位姿缓存（来自 /tcp_pose 话题）
        self.latest_tcp_pose = None

        # 关节角度缓存（来自 /joint_states 话题）
        self.latest_joint_state = None

        # ServoJ 插值运动相关（从 ROS 参数读取）
        self.servoj_current_target = [0.0] * 7   # 当前定时器实际发送的值
        self.servoj_initialized = False           # 是否已从关节状态初始化
        self.servoj_interp_points = int(self.get_parameter('servoj_interp_points').value)
        self.tracking_interp_points = int(self.get_parameter('tracking_interp_points').value)
        self.servoj_loss_timeout = float(self.get_parameter('servoj_loss_timeout').value)
        self.servoj_arrival_threshold = float(self.get_parameter('servoj_arrival_threshold').value)
        self.servoj_update_threshold = float(self.get_parameter('servoj_update_threshold').value)
        self.tracking_min_height = float(self.get_parameter('tracking_min_height').value)
        self.servoj_current_final = None          # 当前轨迹的目标终点（到位检测用）
        self.servoj_trajectory = []               # 预生成的 N 个插值点
        self.servoj_traj_idx = 0                  # 当前发送到第几个点
        self.servoj_moving = False                # 是否正在发送轨迹中
        self.servoj_vmax = list(self.get_parameter('servoj_vmax').value)    # °/s
        self.servoj_amax = list(self.get_parameter('servoj_amax').value)    # °/s²
        self.servoj_jmax = list(self.get_parameter('servoj_jmax').value)    # °/s³
        self.servoj_timer = None

        # ── 抓取序列状态（到点 → 闭合夹爪 → 等待 3s → 回零）──
        self._grasp_on_arrival = False   # movej 到位后是否执行抓取序列
        self._grasp_triggered = False    # 跟踪模式下是否已触发抓取（防重复）
        self._grasp_home_timer = None    # 延迟回零定时器
        self._servoj_stable_start = None # 到位稳定起始时间（防路过误触发）

        # ── 物体静止检测（跟踪模式下，物体静止 2s → movej 到物体位置抓取）──
        self._object_stable_start = None  # 物体开始静止的时间戳
        self._last_object_pos = None      # 上一次物体基座标位置
        self._object_stable_threshold = 3.0  # 物体静止多久后触发抓取（秒）

        # 键盘控制相关
        self.old_terminal_settings = None
        self.keyboard_fd = None
        self.keyboard_fd_need_close = False
        self.setup_keyboard()

        self.T_tool_camera = self.load_handeye_matrix_from_param()

        self.safe_log_info('========== T_tool_camera ==========')
        self.safe_log_info(f'\n{self.T_tool_camera}')
        self.safe_log_info('===================================')

        # ── 夹爪初始化（参数从 gripper.yaml 读取）──
        self.gripper = None
        try:
            self.gripper = EG2Gripper(
                port=self.gripper_port,
                slave_id=self.gripper_slave_id,
                baudrate=self.gripper_baudrate
            )
            self.gripper.set_speed(self.gripper_speed)
            self.gripper.set_force(self.gripper_force)
            if self.gripper_catch_mode == 1:
                self.gripper.set_catch_mode(True)
            self.safe_log_info('夹爪初始化成功')
            self.safe_log_info(
                f'夹爪参数: port={self.gripper_port}, '
                f'speed={self.gripper_speed}, force={self.gripper_force}g, '
                f'catch_mode={"持续夹取" if self.gripper_catch_mode else "一次夹取"}'
            )
        except Exception as e:
            self.safe_log_warn(f'夹爪初始化失败（可忽略）: {e}')

        # 创建 ROS2 服务客户端（仅保留运行时所需服务）
        self._create_service_clients()

        # 创建 ROS2 话题订阅与发布
        self._create_topic_sub_pub()

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

        # ── 初始化：夹爪张开 ──
        if self.gripper:
            try:
                self.gripper.open_gripper(opening=self.gripper_open_position)
                self.safe_log_info(f'✔ 夹爪已张开（位置: {self.gripper_open_position}）')
            except Exception as e:
                self.safe_log_warn(f'夹爪张开失败: {e}')
        else:
            self.safe_log_warn('夹爪未初始化，跳过张开')

        self.safe_log_info('control_node 初始化完成')
        self.safe_log_info(f'订阅: {self.camera_object_topic}')
        self.safe_log_info(f'base_frame_id: {self.base_frame_id}')
        self.safe_log_info(f'机械臂: {self.robot_ip}:{self.robot_port}')
        self.safe_log_info(f'目标类型 object_type: {self.target_object_type}')
        self.safe_log_info(f'grasp_offset: {self.grasp_offset} m')
        self.safe_log_info(f'approach_movetype: {self.approach_movetype}')
        self.safe_log_info(f'ServoJ 插值点数: {self.servoj_interp_points}')
        self.safe_log_info(f'ServoJ vmax: {self.servoj_vmax}')
        self.safe_log_info('按键: c 视觉跟随 | z 回零 | o 夹爪张开 | p 夹爪闭合 | 3 显示夹爪参数 | q 退出')

    def _create_service_clients(self):
        """创建运行时所需的 tl_driver 服务客户端（坐标变换与可达性检测）。"""
        self.coord_transform_client = self.create_client(CoordTransform, '/tl_driver/coord_transform')
        self.get_pos_reachable_client = self.create_client(GetPosReachable, '/tl_driver/get_pos_reachable')

    def _create_topic_sub_pub(self):
        """创建话题订阅、发布器与 ServoJ 定时器。"""
        self.tcp_pose_sub = self.create_subscription(
            CartesianPose,
            '/tcp_pose',
            self.tcp_pose_callback,
            10
        )
        # 订阅关节状态，获取当前关节角度（tl_driver 已 remap 到 /nano/joint_states）
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/nano/joint_states',
            self.joint_state_callback,
            10
        )
        # ServoJ 关节角度发布器（替换原来的 moveJ 发布器）
        self.servoj_pos_pub = self.create_publisher(
            Float64MultiArray,
            '/tl_driver/set_servoj_pos',
            10
        )
        # 发布关节状态给 robot_state_publisher（用于 RViz 显示）
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        # ServoJ 定时器（10ms = 100Hz，持续发送带插值的关节角度）
        self.servoj_timer = self.create_timer(0.008, self.servoj_timer_callback)

    def tcp_pose_callback(self, msg: CartesianPose):
        """缓存来自 /tcp_pose 话题的最新末端位姿。"""
        self.latest_tcp_pose = msg

    def joint_state_callback(self, msg: JointState):
        """缓存来自 /joint_states 话题的最新关节角度。"""
        self.latest_joint_state = msg

    def servoj_timer_callback(self):
        """定时器回调（100Hz）：从预生成轨迹中取点发送到 /tl_driver/set_servoj_pos。"""
        if not self.running:
            return

        # 首次运行时，用当前关节位置初始化 current_target（弧度→度）
        if not self.servoj_initialized and self.latest_joint_state is not None:
            current_pos_rad = list(self.latest_joint_state.position)
            current_pos_deg = [v * 180.0 / math.pi for v in current_pos_rad]
            self.servoj_current_target = (current_pos_deg + [0.0] * 7)[:7]
            self.servoj_initialized = True

        if not self.servoj_initialized:
            return

        # 从预生成轨迹中取下一个点发送
        if self.servoj_moving:
            if self.servoj_traj_idx < len(self.servoj_trajectory):
                self.servoj_current_target = self.servoj_trajectory[self.servoj_traj_idx]
                self.servoj_traj_idx += 1

                # 到位检测：所有关节与目标终点差 < 阈值，提前结束
                if self.servoj_current_final is not None:
                    max_diff = max(
                        abs(self.servoj_current_target[i] - self.servoj_current_final[i])
                        for i in range(7)
                    )
                    if max_diff < self.servoj_arrival_threshold:
                        self.servoj_current_target = list(self.servoj_current_final)
                        self.servoj_moving = False
                        self.servoj_current_final = None
                        # ServoJ 到位，不输出日志减少刷屏
                        # ── 到位后：抓取序列 ──
                        self._on_servoj_arrival()
                    else:
                        # 还没到位：重置稳定计时
                        self._servoj_stable_start = None
            else:
                self.servoj_moving = False
                self.servoj_current_final = None
                # 轨迹发送完毕，触发到位处理
                self._on_servoj_arrival()

        # 发布 ServoJ 关节角度到 tl_driver
        msg = Float64MultiArray()
        msg.data = self.servoj_current_target
        self.servoj_pos_pub.publish(msg)

        # 发布关节状态到 /joint_states（用于 RViz 显示）
        if self.latest_joint_state is not None:
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = list(self.latest_joint_state.name)
            # servoj_current_target 是度，转回弧度
            current_rad = [v * math.pi / 180.0 for v in self.servoj_current_target]
            js_msg.position = current_rad[:len(js_msg.name)]
            self.joint_state_pub.publish(js_msg)

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

        # 重试机制：ServoJ 运动期间 NRC 可能繁忙，最多重试 3 次
        max_retries = 3
        for attempt in range(max_retries):
            try:
                future = self.coord_transform_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()

                if result.success:
                    return 0, list(result.target_pos)

                if attempt < max_retries - 1:
                    self.safe_log_warn(
                        f'笛卡尔转关节失败(第{attempt+1}次): {result.message}，重试...'
                    )
                    time.sleep(0.05)  # 等 50ms 再试
                else:
                    self.safe_log_error(
                        f'笛卡尔转关节失败(已重试{max_retries}次): {result.message}'
                    )

            except Exception as e:
                if attempt < max_retries - 1:
                    self.safe_log_warn(f'笛卡尔转关节异常(第{attempt+1}次): {e}，重试...')
                    time.sleep(0.05)
                else:
                    self.safe_log_error(f'笛卡尔转关节异常(已重试{max_retries}次): {e}')

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

    def servoj_move(self, joint_pos):
        """预生成 N 个插值点轨迹，定时器依次发送。"""
        try:
            joint_list = list(joint_pos)

            if len(joint_list) < 7:
                joint_list = joint_list + [0.0] * (7 - len(joint_list))

            if not self.servoj_initialized:
                self.safe_log_warn('ServoJ 尚未初始化（未收到关节数据），跳过运动')
                return False

            final_target = joint_list[:7]
            start = list(self.servoj_current_target)

            # 预生成 N 个插值点
            n = self.servoj_interp_points
            self.servoj_trajectory = []
            for step in range(1, n + 1):
                ratio = step / n
                point = [
                    start[i] + (final_target[i] - start[i]) * ratio
                    for i in range(7)
                ]
                self.servoj_trajectory.append(point)

            self.servoj_current_final = final_target  # 记录终点
            self.servoj_traj_idx = 0
            self.servoj_moving = True
            self._servoj_stable_start = None        # 新轨迹，重置稳定计时
            return True

        except Exception as e:
            self.safe_log_error(f'movej 异常: {e}')
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
            float((p[0] + self.grasp_forward_offset) * 1000.0),
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
        if self.servoj_move(target_joint):
            self._grasp_on_arrival = True  # 到点后触发抓取序列
            return True
        return False

    def move_zero(self):
        if len(self.zero_joint) < 7:
            zero_joint = list(self.zero_joint) + [0.0] * (7 - len(self.zero_joint))
        else:
            zero_joint = list(self.zero_joint[:7])

        self.safe_log_info(f'回零点: {zero_joint}')

        if not self.check_pos_reachable(zero_joint):
            self.safe_log_warn('零点位置不可达')
            return False

        return self.servoj_move(zero_joint)

    def _on_servoj_arrival(self):
        """ServoJ 到点：触发抓取序列（仅 movej/servoj_move 路径）"""
        # movej/movel 路径：_grasp_on_arrival 标志（来自 execute_control 或 _trigger_grasp_from_tracking）
        if self._grasp_on_arrival:
            self._grasp_on_arrival = False
            self._grasp_triggered = False
            self._execute_grasp_sequence()
            return

        # 视觉跟随模式下的到位：不触发抓取，由 _trigger_grasp_from_tracking 控制
        # （物体静止 2 秒后的稳定检测才触发抓取，到位只是轨迹走完而已）

    def _trigger_grasp_from_tracking(self):
        """物体静止后：关闭跟踪 → movej 到物体位置 → 到点抓取"""
        if self._grasp_triggered:
            return

        if self.latest_base_point is None:
            self.safe_log_warn('没有物体位置数据，无法抓取')
            return

        self._grasp_triggered = True
        self.tracking_mode = False

        p = self.latest_base_point.copy()
        target_cart = [
            float((p[0] + self.grasp_forward_offset) * 1000.0),  # X：前向偏移
            float(p[1] * 1000.0),
            float((p[2] + self.grasp_offset) * 1000.0),  # Z：抓取高度
            -3.14, 0.0, 0.0, 0.0
        ]

        self.safe_log_info(f'⬇ 前往抓取位置: [{target_cart[0]:.1f}, {target_cart[1]:.1f}, {target_cart[2]:.1f}]')

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0 or len(target_joint) < 7:
            self.safe_log_error('目标位置计算失败，取消抓取')
            self._grasp_triggered = False
            return

        if not self.check_pos_reachable(target_joint):
            self.safe_log_warn('目标位置不可达，取消抓取')
            self._grasp_triggered = False
            return

        if self.servoj_move(list(target_joint[:7])):
            self._grasp_on_arrival = True  # 到位后触发抓取序列
            self.safe_log_info('servoj 到位后自动执行抓取')

    def _execute_grasp_sequence(self):
        """到点后的抓取序列：闭合夹爪 → 等待 gripper_grasp_wait_duration → 回零"""
        # 1. 闭合夹爪（使用 gripper.yaml 中的闭合位置）
        if self.gripper:
            try:
                self.gripper.close_gripper(opening=self.gripper_close_position)
                self.safe_log_info(f'✔ 夹爪已闭合（位置: {self.gripper_close_position}），{self.gripper_grasp_wait_duration} 秒后自动回零')
            except Exception as e:
                self.safe_log_error(f'夹爪闭合失败: {e}')
        else:
            self.safe_log_warn('夹爪未初始化，3 秒后直接回零')

        # 2. 取消之前的定时器（防止重复）
        if self._grasp_home_timer is not None:
            self._grasp_home_timer.cancel()

        # 3. 根据配置等待后回零
        wait_sec = self.gripper_grasp_wait_duration
        self._grasp_home_timer = self.create_timer(wait_sec, self._on_grasp_home_timer)

    def _on_grasp_home_timer(self):
        """抓取等待到，执行回零"""
        self.safe_log_info(f'{self.gripper_grasp_wait_duration} 秒到，执行回零')
        self._grasp_home_timer.cancel()
        self._grasp_home_timer = None
        # 直接设置 ServoJ 轨迹回零（不经过可达性检测，零点是安全的）
        zero_joint = (list(self.zero_joint) + [0.0] * 7)[:7]
        if self.servoj_initialized:
            start = list(self.servoj_current_target)
            n = self.servoj_interp_points
            self.servoj_trajectory = []
            for step in range(1, n + 1):
                ratio = step / n
                point = [start[i] + (zero_joint[i] - start[i]) * ratio for i in range(7)]
                self.servoj_trajectory.append(point)
            self.servoj_current_final = zero_joint
            self.servoj_traj_idx = 0
            self.servoj_moving = True
            self._servoj_stable_start = None
            self.safe_log_info(f'ServoJ 回零轨迹已生成，{n/100:.1f} 秒到达')
        else:
            self.safe_log_warn('ServoJ 未初始化，无法回零')

    def _update_trajectory_from_detection(self):
        """视觉跟随模式下：用最新 YOLO 检测结果重新生成轨迹，覆盖当前轨迹。"""
        now = time.time()

        # ===== 检查 YOLO 是否丢失目标 =====
        # 仅在至少处理过一次有效检测后才进入丢失检测
        if self.latest_stamp is not None and self._last_processed_stamp > 0.0:
            elapsed_no_detection = now - self.latest_stamp

            # 超过 servoj_loss_timeout 秒 → 回零
            if elapsed_no_detection > self.servoj_loss_timeout:
                if not self._loss_homed:
                    self._loss_homed = True
                    self.servoj_moving = False
                    self.tracking_mode = False
                    self.safe_log_warn(f' YOLO 目标丢失超过 {self.servoj_loss_timeout}s，回零')
                    self.move_zero()
                return

            # 连续 0.5 秒没有新检测 → 认为目标丢失，停住等待
            if elapsed_no_detection > 0.5:
                if self._loss_start_time is None:
                    self._loss_start_time = now
                    self.servoj_moving = False
                    self.safe_log_warn(f'YOLO 目标丢失，停在当前位置等待...')
                return

            # 丢失状态中，且 latest_stamp 刚更新了（短时间内 elapsed 又变小了）
            if self._loss_start_time is not None and self.latest_stamp > self._loss_start_time:
                self.safe_log_info('YOLO 目标恢复，继续跟踪')
                self._loss_start_time = None
                self._loss_homed = False
                # 不 return，进入下方正常处理

            # 正常接收中（elapsed < 0.5s 且未丢失），不阻塞，进入下方正常处理

        # ===== 目标回来了（之前丢失过，现在又检测到了，且 latest_stamp 从未更新过）=====
        if self._loss_start_time is not None or self._loss_homed:
            self.safe_log_info('YOLO 目标恢复，继续跟踪')
            self._loss_start_time = None
            self._loss_homed = False

        # ===== 正常处理：有检测数据 =====
        if self.latest_base_point is None:
            return
        if self.latest_object_type != self.target_object_type:
            return
        if self.latest_stamp == self._last_processed_stamp:
            return

        p = self.latest_base_point.copy()
        target_z_mm = float((p[2] + self.grasp_offset) * 1000.0)
        min_z_mm = float(self.tracking_min_height * 1000.0)
        if target_z_mm < min_z_mm:
            target_z_mm = min_z_mm
        target_cart = [
            float((p[0] + self.grasp_forward_offset) * 1000.0),
            float(p[1] * 1000.0),
            target_z_mm,
            -3.14, 0.0, 0.0, 0.0
        ]

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0:
            self._last_processed_stamp = self.latest_stamp
            return
        if len(target_joint) < 7:
            self._last_processed_stamp = self.latest_stamp
            return

        joint_list = list(target_joint[:7])

        for i in range(7):
            if abs(joint_list[i]) > 180.0:
                self.safe_log_warn(f'关节{i+1}角度异常: {joint_list[i]:.2f}°，跳过更新')
                return

        # ── 死区阈值 + 物体静止检测 ──
        now = time.time()
        if self.servoj_current_final is not None:
            max_diff = max(
                abs(joint_list[i] - self.servoj_current_final[i])
                for i in range(7)
            )
            if max_diff < self.servoj_update_threshold:
                # 检测方向反转：新目标与旧目标在旧轨迹方向的相反侧
                is_reversal = False
                if self.servoj_moving:
                    for i in range(7):
                        old_dir = self.servoj_current_final[i] - self.servoj_current_target[i]
                        new_dir = joint_list[i] - self.servoj_current_target[i]
                        if abs(old_dir) > 2.0 and old_dir * new_dir < 0:
                            is_reversal = True
                            break

                if is_reversal:
                    # 方向反转 → 立即覆盖轨迹，不等待阈值
                    self._object_stable_start = None
                else:
                    # 物体静止 → 立即停止当前轨迹，防止超调
                    if self.servoj_moving:
                        self.servoj_moving = False
                        self.servoj_current_final = None
                        self.servoj_trajectory = []
                    # 累计静止时间
                    if not self._grasp_triggered:
                        if self._object_stable_start is None:
                            self._object_stable_start = now
                        elif now - self._object_stable_start >= self._object_stable_threshold:
                            self.safe_log_info(
                                f'物体已静止 {self._object_stable_threshold:.0f} 秒，执行抓取'
                            )
                            self._object_stable_start = None
                            self._trigger_grasp_from_tracking()
                            return
                    return
            else:
                # 物体移动了 → 重置静止计时
                self._object_stable_start = None

        start = list(self.servoj_current_target)
        # 根据移动距离自动调整点数，限制每步最大 0.1°，防止初始冲出去
        max_joint_diff = max(abs(joint_list[i] - start[i]) for i in range(7))
        n = max(self.tracking_interp_points, int(max_joint_diff / 0.1))
        n = min(n, 1000)
        self.servoj_trajectory = []
        for step in range(1, n + 1):
            ratio = step / n
            point = [
                start[i] + (joint_list[i] - start[i]) * ratio
                for i in range(7)
            ]
            self.servoj_trajectory.append(point)

        self.servoj_current_final = joint_list  # 记录终点，到位检测用
        self.servoj_traj_idx = 0
        self.servoj_moving = True
        self._servoj_stable_start = None        # 目标更新，重置稳定计时
        self._last_processed_stamp = self.latest_stamp

        self.safe_log_info(
            f'[跟随更新] 目标关节: {[f"{j:.2f}" for j in joint_list]}'
        )

    def handle_key(self, key):
        self.safe_log_info(f'收到按键: {repr(key)}')

        if key == 'c':
            self.tracking_mode = not self.tracking_mode
            if self.tracking_mode:
                self._grasp_triggered = False
                self._object_stable_start = None
                self.safe_log_info('开启视觉跟随模式（物体静止 2 秒后自动抓取）')
                self._update_trajectory_from_detection()
            else:
                self.safe_log_info('关闭视觉跟随模式')

        elif key == 'z':
            self.tracking_mode = False
            self.safe_log_info('按键 z: 回零点')
            self.move_zero()

        elif key == 'o':
            if self.gripper:
                self.gripper.open_gripper(opening=self.gripper_open_position)
                self.safe_log_info(f'夹爪张开（位置: {self.gripper_open_position}）')
            else:
                self.safe_log_warn('夹爪未初始化')

        elif key == 'p':
            if self.gripper:
                self.gripper.close_gripper(opening=self.gripper_close_position)
                self.safe_log_info(f'夹爪闭合（位置: {self.gripper_close_position}）')
            else:
                self.safe_log_warn('夹爪未初始化')

        elif key == '3':
            if self.gripper:
                self.safe_log_info(
                    f'夹爪参数 — '
                    f'port: {self.gripper_port} | '
                    f'speed: {self.gripper_speed} | '
                    f'force: {self.gripper_force}g | '
                    f'open: {self.gripper_open_position} | '
                    f'close: {self.gripper_close_position} | '
                    f'catch_mode: {"持续" if self.gripper_catch_mode else "一次"} | '
                    f'wait: {self.gripper_grasp_wait_duration}s'
                )
            else:
                self.safe_log_warn('夹爪未初始化')

        elif key == 'q':
            print('\n按键 q: 退出 control_node')
            self.running = False

    def run(self):
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.05)

                # 视觉跟随模式下，每帧检查是否有新 YOLO 检测
                if self.tracking_mode:
                    self._update_trajectory_from_detection()

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

        # 先恢复键盘设置（最重要，否则终端卡死）
        self.restore_keyboard()

        # 取消延迟回零定时器
        if self._grasp_home_timer is not None:
            try:
                self._grasp_home_timer.cancel()
            except Exception:
                pass
            self._grasp_home_timer = None

        # 快速关闭夹爪（加超时保护）
        if self.gripper:
            try:
                self.gripper.close()
            except Exception:
                pass
            self.gripper = None
            print('夹爪已关闭')

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
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()