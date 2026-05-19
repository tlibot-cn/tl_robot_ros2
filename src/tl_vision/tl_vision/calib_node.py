#!/usr/bin/env python3
import signal
import os
import sys
import time
import numpy as np
import cv2
import pyrealsense2 as rs
import rclpy
import yaml

from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import Trigger
from tl_ros2_interface.msg import CartesianPose

np.set_printoptions(precision=8, suppress=True)

_tl_vision_share = get_package_share_directory('tl_vision')
_default_eye_hand_data = os.path.join(_tl_vision_share, 'eye_hand_data')

class HandEyeCalibrationNode(Node):
    """手眼标定节点：online 实时采集 / offline 离线计算"""

    def __init__(self):
        super().__init__('calib_demo')

        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('chessboard_xx', 9)
        self.declare_parameter('chessboard_yy', 6)
        self.declare_parameter('chessboard_L', 0.025)
        self.declare_parameter('save_path', _default_eye_hand_data)
        self.declare_parameter('save_result_file', True)
        self.declare_parameter('display_scale', 2.0)
        self.declare_parameter('calculation_mode', 'online')
        self.declare_parameter('data_file', os.path.join(_default_eye_hand_data, 'handeye_samples.npz'))
        self.declare_parameter('handeye_method', 'TSAI')

        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.chessboard_xx = self.get_parameter('chessboard_xx').value
        self.chessboard_yy = self.get_parameter('chessboard_yy').value
        self.chessboard_L = self.get_parameter('chessboard_L').value
        self.save_path = self.get_parameter('save_path').value
        self.save_result_file = self.get_parameter('save_result_file').value
        self.display_scale = self.get_parameter('display_scale').value
        self.calculation_mode = self.get_parameter('calculation_mode').value
        self.data_file = self.get_parameter('data_file').value
        self.handeye_method = self.get_parameter('handeye_method').value

        if not self.save_path:
            self.save_path = os.path.join(
                os.getcwd(),
                f'calibration_data_{time.strftime("%Y%m%d_%H%M%S")}'
            )

        os.makedirs(self.save_path, exist_ok=True)

        if not self.data_file:
            self.data_file = os.path.join(self.save_path, 'handeye_samples.npz')

        # ROS2 service clients (tl_driver wraps NRC SDK)
        self._connect_cli = self.create_client(Trigger, '/tl_driver/connect_arm')
        self._power_on_cli = self.create_client(Trigger, '/tl_driver/power_on')
        self._power_off_cli = self.create_client(Trigger, '/tl_driver/power_off')
        self._clear_error_cli = self.create_client(Trigger, '/tl_driver/clear_error')
        self._disconnect_cli = self.create_client(Trigger, '/tl_driver/disconnect_arm')

        # /tcp_pose 订阅 — 替代 nrc.get_current_position()
        self._tcp_pose = None
        self._tcp_pose_received = False
        self._tcp_pose_sub = self.create_subscription(
            CartesianPose,
            '/tcp_pose',
            self._tcp_pose_callback,
            10
        )

        self.running = True
        self.pipeline = None
        self.cleaned = False
        self.robot_connected = False
        self.obj_points_mem = []
        self.img_points_mem = []
        self.pose_mem = []
        self.R_tool_mem = []
        self.t_tool_mem = []

        # solvePnP 后得到的 target2cam
        self.R_target2cam_mem = []
        self.t_target2cam_mem = []

        # 当前帧角点状态
        self.image_size = None
        self.current_corners = None
        self.current_corners_valid = False
        self.current_gray = None

        # RealSense 官方内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.intrinsics_ready = False

        # 最近一次标定结果
        self.last_result = None

        mode = str(self.calculation_mode).lower()
        if mode == 'online':
            self.init_camera()
            self.init_robot()

        elif mode == 'offline':
            self.safe_log_info('当前为 offline 模式，不初始化相机和机械臂')
            self.load_samples_from_file(self.data_file)

        else:
            self.safe_log_error(f'未知 calculation_mode: {self.calculation_mode}')
            self.cleanup_and_exit()

        self.safe_log_info('手眼标定节点初始化完成')
        self.safe_log_info(f'模式: {self.calculation_mode}')
        self.safe_log_info(f'机械臂通过 tl_driver ROS2 服务连接')
        self.safe_log_info(
            f'标定板配置: {self.chessboard_xx}x{self.chessboard_yy}, '
            f'格子大小: {self.chessboard_L} m'
        )
        self.safe_log_info(f'数据文件: {self.data_file}')
        self.safe_log_info(f'结果保存路径: {self.save_path}')
        self.safe_log_info(f'手眼算法: {self.handeye_method}')

        if mode == 'online':
            self.safe_log_info('操作说明:')
            self.safe_log_info('  s : 当前角点有效时采集一组数据，并保存到文件')
            self.safe_log_info('  c : 使用内存数据计算标定结果')
            self.safe_log_info('  r : 清空已采集数据')
            self.safe_log_info('  q : 退出程序')
        else:
            self.safe_log_info('offline 模式：将直接使用保存数据计算标定结果')

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

    def _call_service(self, client, request, timeout_sec=10.0):
        """同步 ROS2 服务调用，带超时"""
        if not client.service_is_ready():
            self.safe_log_error(f'服务未就绪: {client.srv_name}')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.done() and future.result() is not None:
            return future.result()
        self.safe_log_error(f'服务调用超时: {client.srv_name}')
        return None

    def _wait_for_services(self, timeout_sec=10.0):
        """等待所有必需服务就绪"""
        services = [
            (self._connect_cli, 'connect_arm'),
            (self._power_on_cli, 'power_on'),
            (self._power_off_cli, 'power_off'),
            (self._disconnect_cli, 'disconnect_arm'),
        ]
        self.safe_log_info('等待 tl_driver 服务就绪...')
        for cli, name in services:
            if not cli.wait_for_service(timeout_sec=timeout_sec):
                self.safe_log_error(f'服务 {name} 未就绪（超时 {timeout_sec}s）')
                return False
        self.safe_log_info('所有 tl_driver 服务已就绪')
        return True

    def _tcp_pose_callback(self, msg):
        """订阅 /tcp_pose 话题，缓存最新位姿"""
        self._tcp_pose = msg
        self._tcp_pose_received = True

    def init_camera(self):
        """初始化 RealSense 相机，并读取官方内参"""
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(
            rs.stream.color,
            int(self.camera_width),
            int(self.camera_height),
            rs.format.bgr8,
            int(self.camera_fps)
        )

        try:
            profile = self.pipeline.start(config)

            # 等待几帧，让相机稳定
            for _ in range(10):
                self.pipeline.wait_for_frames(timeout_ms=1000)

            color_profile = profile.get_stream(
                rs.stream.color
            ).as_video_stream_profile()

            intr = color_profile.get_intrinsics()

            self.camera_matrix = np.array([
                [intr.fx, 0.0, intr.ppx],
                [0.0, intr.fy, intr.ppy],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)

            # RealSense coeffs 通常是 [k1, k2, p1, p2, k3]
            self.dist_coeffs = np.array(intr.coeffs, dtype=np.float64).reshape(-1, 1)

            self.image_size = (int(intr.width), int(intr.height))
            self.intrinsics_ready = True

            self.safe_log_info('✓ 相机初始化成功')
            self.safe_log_info('使用 RealSense 官方内参:')
            self.safe_log_info(f'camera_matrix:\n{self.camera_matrix}')
            self.safe_log_info(f'dist_coeffs:\n{self.dist_coeffs.flatten()}')
            self.safe_log_info(f'image_size: {self.image_size}')

        except Exception as e:
            self.safe_log_error(f'✗ 相机连接或读取内参异常: {e}')
            self.cleanup_and_exit()

    def init_robot(self):
        """通过 tl_driver ROS2 服务初始化机械臂连接并上电"""
        if not self._wait_for_services():
            self.safe_log_error('tl_driver 服务未就绪，请先启动 tl_driver 节点')
            self.cleanup_and_exit()

        self.safe_log_info('通过 /tl_driver/connect_arm 连接机械臂...')
        result = self._call_service(self._connect_cli, Trigger.Request())
        if result is None or not result.success:
            self.safe_log_error(f'机械臂连接失败: {result.message if result else "无响应"}')
            self.cleanup_and_exit()

        self.safe_log_info('机械臂连接成功')
        self.robot_connected = True

        self.safe_log_info('通过 /tl_driver/power_on 上电...')
        result = self._call_service(self._power_on_cli, Trigger.Request())
        if result is None or not result.success:
            self.safe_log_error(f'机械臂上电失败: {result.message if result else "无响应"}')
            self.cleanup_and_exit()

        self.safe_log_info('机械臂上电成功')
        time.sleep(1)

    def power_on_robot(self):
        """机械臂上电（tl_driver 服务内部处理所有伺服状态逻辑）"""
        if not self._power_on_cli.service_is_ready():
            self.safe_log_error('power_on 服务未就绪')
            return False
        result = self._call_service(self._power_on_cli, Trigger.Request())
        if result and result.success:
            self.safe_log_info('上电成功')
            return True
        self.safe_log_error(f'上电失败: {result.message if result else "无响应"}')
        return False

    def power_off_robot(self):
        """机械臂下电（通过 tl_driver 服务）"""
        if not self._power_off_cli.service_is_ready():
            return True
        result = self._call_service(self._power_off_cli, Trigger.Request())
        return result is not None and result.success

    def get_robot_pose(self):
        """
        获取机械臂当前位姿（从 /tcp_pose 话题缓存）。
        返回格式：[x, y, z, A, B, C]，x/y/z 单位 m，A/B/C 单位 rad。
        """
        if self._tcp_pose is None:
            self.safe_log_error('尚未收到 /tcp_pose 数据')
            return False, None

        msg = self._tcp_pose
        pose_converted = [
            msg.position.x / 1000.0,  # mm → m
            msg.position.y / 1000.0,
            msg.position.z / 1000.0,
            msg.rpy.x,               # rad，不变
            msg.rpy.y,
            msg.rpy.z
        ]
        return True, pose_converted

    def pose_to_tool_rt(self, pose):
        """
        将机械臂 pose [x, y, z, A, B, C] 转成 R_tool, t_tool。
        """
        if len(pose) < 6:
            raise ValueError(f'pose 长度不足: {pose}')

        t_tool = np.array(pose[0:3], dtype=np.float64).reshape(3, 1)

        A, B, C = pose[3], pose[4], pose[5]

        # 固定 ABC 欧拉角解释方式，不使用参数
        rotation = R.from_euler('XYZ', [A, B, C], degrees=False)

        R_tool = rotation.as_matrix()

        return R_tool, t_tool

    def get_chessboard_object_points(self):
        """生成棋盘格三维内角点，单位：米"""
        objp = np.zeros(
            (int(self.chessboard_xx) * int(self.chessboard_yy), 3),
            np.float32
        )

        objp[:, :2] = np.mgrid[
            0:int(self.chessboard_xx),
            0:int(self.chessboard_yy)
        ].T.reshape(-1, 2)

        objp = float(self.chessboard_L) * objp
        return objp

    def detect_chessboard_corners(self, color_image):
        """实时检测棋盘格角点"""
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        if self.image_size is None:
            self.image_size = gray.shape[::-1]

        pattern_size = (
            int(self.chessboard_xx),
            int(self.chessboard_yy)
        )

        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH |
            cv2.CALIB_CB_NORMALIZE_IMAGE |
            cv2.CALIB_CB_FAST_CHECK
        )

        ret, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags
        )

        if ret:
            criteria = (
                cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS,
                30,
                0.001
            )

            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                criteria
            )

            self.current_corners = corners2 if corners2 is not None else corners
            self.current_corners_valid = True
            self.current_gray = gray

        else:
            self.current_corners = None
            self.current_corners_valid = False
            self.current_gray = gray

        return self.current_corners_valid, self.current_corners

    def save_samples_to_file(self, file_path=None):
        """保存采集数据到 npz 文件"""
        if file_path is None:
            file_path = self.data_file

        try:
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

            if self.camera_matrix is None or self.dist_coeffs is None:
                self.safe_log_warn('相机内参为空，保存数据时不会完整')
                camera_matrix = np.empty((0,))
                dist_coeffs = np.empty((0,))
            else:
                camera_matrix = self.camera_matrix
                dist_coeffs = self.dist_coeffs

            np.savez_compressed(
                file_path,
                obj_points=np.array(self.obj_points_mem, dtype=np.float32),
                img_points=np.array(self.img_points_mem, dtype=np.float32),
                poses=np.array(self.pose_mem, dtype=np.float64),
                camera_matrix=np.array(camera_matrix, dtype=np.float64),
                dist_coeffs=np.array(dist_coeffs, dtype=np.float64),
                image_size=np.array(self.image_size if self.image_size is not None else [0, 0]),
                chessboard_xx=np.array([int(self.chessboard_xx)]),
                chessboard_yy=np.array([int(self.chessboard_yy)]),
                chessboard_L=np.array([float(self.chessboard_L)])
            )

            self.safe_log_info(
                f'采集数据已保存: {file_path}, '
                f'数量: {len(self.img_points_mem)}'
            )
            return True

        except Exception as e:
            self.safe_log_error(f'保存采集数据失败: {e}')
            return False

    def load_samples_from_file(self, file_path):
        """从 npz 文件读取采集数据"""
        try:
            if not os.path.exists(file_path):
                self.safe_log_error(f'离线数据文件不存在: {file_path}')
                return False

            data = np.load(file_path, allow_pickle=True)

            obj_points = data['obj_points']
            img_points = data['img_points']
            poses = data['poses']

            self.obj_points_mem = [p.astype(np.float32) for p in obj_points]
            self.img_points_mem = [p.astype(np.float32) for p in img_points]
            self.pose_mem = [p.astype(np.float64).tolist() for p in poses]

            if 'camera_matrix' in data:
                cm = data['camera_matrix']
                if cm.size > 0:
                    self.camera_matrix = cm.astype(np.float64)

            if 'dist_coeffs' in data:
                dc = data['dist_coeffs']
                if dc.size > 0:
                    self.dist_coeffs = dc.astype(np.float64).reshape(-1, 1)

            if 'image_size' in data:
                img_size = data['image_size'].astype(int).tolist()
                if len(img_size) >= 2 and img_size[0] > 0:
                    self.image_size = tuple(img_size[:2])

            if 'chessboard_xx' in data:
                self.chessboard_xx = int(data['chessboard_xx'][0])
            if 'chessboard_yy' in data:
                self.chessboard_yy = int(data['chessboard_yy'][0])
            if 'chessboard_L' in data:
                self.chessboard_L = float(data['chessboard_L'][0])

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                self.intrinsics_ready = True
            else:
                self.intrinsics_ready = False

            # 根据 pose 重新生成 R_tool / t_tool
            self.R_tool_mem.clear()
            self.t_tool_mem.clear()

            for pose in self.pose_mem:
                R_tool, t_tool = self.pose_to_tool_rt(pose)
                self.R_tool_mem.append(R_tool)
                self.t_tool_mem.append(t_tool)

            self.safe_log_info(f'离线数据读取成功: {file_path}')
            self.safe_log_info(f'样本数量: {len(self.img_points_mem)}')
            self.safe_log_info(f'camera_matrix:\n{self.camera_matrix}')
            self.safe_log_info(f'dist_coeffs:\n{self.dist_coeffs.flatten() if self.dist_coeffs is not None else None}')
            self.safe_log_info(f'image_size: {self.image_size}')

            return True

        except Exception as e:
            self.safe_log_error(f'读取离线数据失败: {e}')
            return False

    def capture_data_to_memory(self):
        """当前角点有效时，采集角点和机械臂位姿到内存，并保存到文件"""

        if not self.current_corners_valid or self.current_corners is None:
            self.safe_log_warn('当前未检测到有效棋盘格角点，不采集')
            return False

        success, pose = self.get_robot_pose()

        if not success:
            self.safe_log_error('采集失败: 无法获取机械臂位姿')
            return False

        if len(pose) < 6:
            self.safe_log_error(f'采集失败: 位姿长度不足: {pose}')
            return False

        if not self.intrinsics_ready:
            self.safe_log_error('采集失败: 相机官方内参未准备好')
            return False

        try:
            objp = self.get_chessboard_object_points()

            R_tool, t_tool = self.pose_to_tool_rt(pose)

            self.obj_points_mem.append(objp.copy())
            self.img_points_mem.append(self.current_corners.copy())
            self.pose_mem.append(list(pose))

            self.R_tool_mem.append(R_tool.copy())
            self.t_tool_mem.append(t_tool.copy())

            pose_str = ','.join([f"{p:.6f}" for p in pose])

            self.safe_log_info(
                f'✓ 已采集第 {len(self.img_points_mem)} 组有效数据 | 位姿: {pose_str}'
            )

            # 每采集一组就自动保存，避免中途退出丢数据
            self.save_samples_to_file(self.data_file)

            return True

        except Exception as e:
            self.safe_log_error(f'内存采集失败: {e}')
            return False

    def reset_memory_data(self):
        """清空内存采集数据"""
        self.obj_points_mem.clear()
        self.img_points_mem.clear()
        self.pose_mem.clear()
        self.R_tool_mem.clear()
        self.t_tool_mem.clear()
        self.R_target2cam_mem.clear()
        self.t_target2cam_mem.clear()
        self.last_result = None
        self.safe_log_warn('已清空所有内存采集数据')

        try:
            if os.path.exists(self.data_file):
                os.remove(self.data_file)
                self.safe_log_warn(f'已删除数据文件: {self.data_file}')
        except Exception as e:
            self.safe_log_warn(f'删除数据文件失败: {e}')

    def get_handeye_method(self):
        method_name = str(self.handeye_method).upper()

        methods = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        if method_name not in methods:
            self.safe_log_warn(
                f'未知 handeye_method={self.handeye_method}，默认使用 TSAI'
            )
            return cv2.CALIB_HAND_EYE_TSAI

        return methods[method_name]

    def compute_target2cam_by_pnp(self):
        """
        使用 RealSense 官方内参和每组角点，通过 solvePnP 计算：
            R_target2cam, t_target2cam
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.safe_log_error('相机官方内参为空，无法 solvePnP')
            return False

        if len(self.obj_points_mem) != len(self.img_points_mem):
            self.safe_log_error('obj_points 和 img_points 数量不一致')
            return False

        self.R_target2cam_mem.clear()
        self.t_target2cam_mem.clear()

        total_reproj_error = 0.0
        total_points = 0

        for i, (objp, imgp) in enumerate(zip(self.obj_points_mem, self.img_points_mem)):
            objp = np.asarray(objp, dtype=np.float32)
            imgp = np.asarray(imgp, dtype=np.float32)

            ok, rvec, tvec = cv2.solvePnP(
                objp,
                imgp,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not ok:
                self.safe_log_error(f'第 {i + 1} 组 solvePnP 失败')
                return False

            R_target2cam, _ = cv2.Rodrigues(rvec)

            self.R_target2cam_mem.append(R_target2cam.astype(np.float64))
            self.t_target2cam_mem.append(tvec.astype(np.float64).reshape(3, 1))

            # 计算该帧重投影误差
            projected, _ = cv2.projectPoints(
                objp,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs
            )

            projected = projected.reshape(-1, 2)
            imgp_2d = imgp.reshape(-1, 2)

            err = np.linalg.norm(projected - imgp_2d, axis=1)
            total_reproj_error += np.sum(err)
            total_points += len(err)

        mean_reproj_error = total_reproj_error / max(total_points, 1)

        self.safe_log_info(
            f'solvePnP 完成，平均重投影误差: {mean_reproj_error:.6f} px'
        )

        return True

    def calculate_calibration_from_memory(self):
        """使用内存数据计算手眼标定，不做内参标定，使用官方内参 + solvePnP"""

        self.safe_log_info('开始计算手眼标定...')
        self.safe_log_info('注意：不进行 cv2.calibrateCamera 内参标定，使用 RealSense 官方内参')

        num = len(self.img_points_mem)

        if num < 3:
            self.safe_log_error(f'有效数据数量不足: {num}，至少需要3组，建议15组以上')
            return False

        if not (
            len(self.obj_points_mem) == len(self.img_points_mem) ==
            len(self.R_tool_mem) == len(self.t_tool_mem)
        ):
            self.safe_log_error(
                '内存数据数量不一致: '
                f'obj={len(self.obj_points_mem)}, '
                f'img={len(self.img_points_mem)}, '
                f'R={len(self.R_tool_mem)}, '
                f't={len(self.t_tool_mem)}'
            )
            return False

        if self.camera_matrix is None or self.dist_coeffs is None:
            self.safe_log_error('相机官方内参为空，无法计算')
            return False

        try:
            # 1. solvePnP 求每帧 target2cam
            if not self.compute_target2cam_by_pnp():
                return False

            # 2. 手眼标定
            method = self.get_handeye_method()

            R_cam2tool, t_cam2tool = cv2.calibrateHandEye(
                self.R_tool_mem,
                self.t_tool_mem,
                self.R_target2cam_mem,
                self.t_target2cam_mem,
                method=method
            )

            rotation = R.from_matrix(R_cam2tool)
            quaternion = rotation.as_quat()

            result = {
                'rotation_matrix': R_cam2tool.tolist(),
                'translation_vector': t_cam2tool.flatten().tolist(),
                'quaternion_xyzw': quaternion.tolist(),
                'camera_matrix': self.camera_matrix.tolist(),
                'distortion_coefficients': self.dist_coeffs.flatten().tolist(),
                'valid_count': int(num),
                'image_width': int(self.image_size[0]) if self.image_size else 0,
                'image_height': int(self.image_size[1]) if self.image_size else 0,
                'chessboard_xx': int(self.chessboard_xx),
                'chessboard_yy': int(self.chessboard_yy),
                'chessboard_L': float(self.chessboard_L),
                'handeye_method': str(self.handeye_method),
                'calculation_mode': str(self.calculation_mode),
                'note': 'Using RealSense official intrinsics and solvePnP; no calibrateCamera.'
            }

            self.last_result = result

            self.safe_log_info('========== 手眼标定结果 ==========')
            self.safe_log_info(f'模式: {self.calculation_mode}')
            self.safe_log_info(f'有效数据数量: {num}')
            self.safe_log_info(f'手眼算法: {self.handeye_method}')
            self.safe_log_info(f'旋转矩阵:\n{R_cam2tool}')
            self.safe_log_info(f'平移向量:\n{t_cam2tool.flatten()}')
            self.safe_log_info(f'四元数 (x, y, z, w):\n{quaternion}')
            self.safe_log_info('==================================')

            if self.save_result_file:
                os.makedirs(self.save_path, exist_ok=True)
                result_file = os.path.join(self.save_path, "calibration_result.yaml")

                with open(result_file, 'w') as f:
                    yaml.dump(result, f, default_flow_style=False)

                self.safe_log_info(f'结果已保存到: {result_file}')
            else:
                self.safe_log_info('save_result_file=False，未保存结果文件')

            return True

        except Exception as e:
            self.safe_log_error(f'手眼标定计算失败: {e}')
            return False

    def run(self):
        mode = str(self.calculation_mode).lower()

        if mode == 'offline':
            self.calculate_calibration_from_memory()
            self.running = False
            self.cleanup()
            return

        self.run_online()

    def run_online(self):
        """online 主循环：实时捕获图像、检测角点并处理键盘输入"""

        window_name = "Hand-Eye Calibration"

        while self.running:
            try:
                if not rclpy.ok():
                    break

                # 获取相机帧
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
                if not frames:
                    continue

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())

                # 实时检测棋盘格角点
                found_corners, corners = self.detect_chessboard_corners(color_image)

                # 显示图像
                vis_img = color_image.copy()

                if found_corners:
                    cv2.drawChessboardCorners(
                        vis_img,
                        (int(self.chessboard_xx), int(self.chessboard_yy)),
                        corners,
                        found_corners
                    )

                # 画面状态
                status_text = 'CORNERS OK' if found_corners else 'NO CORNERS'
                status_color = (0, 255, 0) if found_corners else (0, 0, 255)

                info_text_1 = (
                    f'Valid Captured: {len(self.img_points_mem)} | {status_text}'
                )
                info_text_2 = 's: capture/save | c: calculate | r: reset | q: quit'

                cv2.putText(
                    vis_img,
                    info_text_1,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    status_color,
                    1
                )

                cv2.putText(
                    vis_img,
                    info_text_2,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1
                )

                # 缩放显示
                if float(self.display_scale) != 1.0:
                    display_img = cv2.resize(
                        vis_img,
                        None,
                        fx=float(self.display_scale),
                        fy=float(self.display_scale),
                        interpolation=cv2.INTER_AREA
                    )
                else:
                    display_img = vis_img

                cv2.imshow(window_name, display_img)

                # 处理键盘输入
                k = cv2.waitKey(1) & 0xFF

                if k == ord('s'):
                    self.safe_log_info('触发采集命令')
                    self.capture_data_to_memory()

                elif k == ord('c'):
                    self.safe_log_info('触发标定计算')
                    self.calculate_calibration_from_memory()

                elif k == ord('r'):
                    self.reset_memory_data()

                elif k == ord('q'):
                    self.safe_log_info('退出程序')
                    self.running = False
                    break

                # 处理 ROS 回调
                if rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0)

            except rclpy.executors.ExternalShutdownException:
                break

            except KeyboardInterrupt:
                print('\n用户中断程序')
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
        """清理资源"""
        if self.cleaned:
            return

        self.cleaned = True
        self.running = False

        print('\n正在清理资源...')

        try:
            cv2.destroyAllWindows()
        except Exception as e:
            print(f'关闭 OpenCV 窗口异常: {e}')

        if self.pipeline:
            try:
                self.pipeline.stop()
                print('相机已停止')
            except Exception as e:
                print(f'停止相机异常: {e}')
            self.pipeline = None

        if self.robot_connected:
            print('通过 /tl_driver/power_off 下电...')
            try:
                result = self._call_service(self._power_off_cli, Trigger.Request())
                if result and result.success:
                    print('机械臂下电成功')
                else:
                    print(f'机械臂下电失败: {result.message if result else "无响应"}')
            except Exception as e:
                print(f'下电异常: {e}')

            print('通过 /tl_driver/disconnect_arm 断开连接...')
            try:
                result = self._call_service(self._disconnect_cli, Trigger.Request())
                if result and result.success:
                    print('断开机械臂连接')
                else:
                    print(f'断开连接失败: {result.message if result else "无响应"}')
            except Exception as e:
                print(f'断开连接异常: {e}')

            self.robot_connected = False

        print('资源清理完成')

    def cleanup_and_exit(self):
        """清理资源并退出"""
        self.cleanup()
        raise SystemExit(1)

def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = HandEyeCalibrationNode()

        def signal_handler(sig, frame):
            print('\n收到中断信号，正在退出...')
            if node is not None:
                node.running = False

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        node.run()

    except KeyboardInterrupt:
        print('\n用户中断程序')

    except SystemExit:
        pass

    except Exception as e:
        print(f'程序异常: {e}')

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
