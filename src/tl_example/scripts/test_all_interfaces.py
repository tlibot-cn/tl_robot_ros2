#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_all_interfaces.py
@brief 驱动快速自检（Quick Test）— 依次遍历测试 tl_driver 全部接口

参照 tx.cpp（C++ 版驱动快速自检）的分组与调用逻辑实现，覆盖：

  §2  连接管理      connect_arm / power_on / power_off（disconnect_arm 不自动调用，需手动断开）
  §3  日志管理      log_download
  §4  信息查询      get_speed / get_controller_id / get_robot_state / get_library_version /
                    get_robot_joint_param / get_joint_temperature / get_joint_voltage /
                    get_motor_current / get_joint_software_version / get_nexmotion_lib_version /
                    get_current_coord / get_coord_num / get_dh_param / get_all_job_filename /
                    get_pos_reachable / get_current_motor_torque / get_current_line_joint_speed
                    + 话题 /joint_states、/tcp_pose、/arm_status
  §5  基础设置      set_speed / coord_transform
  §6  作业运动      job_insert_moveJ / job_insert_moveL / job_insert_imove / job_insert_moveC
  §8  坐标系与工具  set_tool_param / set_user_coord / set_coord_num / set_current_coord
  §9  示教操作      stop_jogging / set_drag_mode / get_drag_status / track_save
  §10 错误/零位    clear_error
  §11 全局路点      get_global_pos / set_global_pos
  §12 模式与IO     set_current_mode / get_current_mode / set_digital_output / get_digital_input_output
  §13 Modbus       modbus_write / modbus_read
  §14 队列运动      queue_motion_set_status / queue_motion_stop
  §15 ServoJ       close_servoj
  §16 位姿转换      get_quat2rpy / get_rpy2quat / get_rpy2r / get_tr2r / get_r2tr

危险接口（会触发运动、改动标定/IP、删除文件等）默认跳过，仅打印提示：
  disconnect_arm（需用户手动断开）/
  moveJ / moveL / start_jogging / queue_motion_movej / open_servoj / job_run / job_delete /
  track_playback / set_axis_zero_pos / set_dh_param / set_robot_joint_param /
  set_controller_ip / restore_default_dh_param / set_default_cartesian_param

@usage
  方式一（推荐，需已 source install/setup.bash）:
    python3 test_all_interfaces.py
  方式二（需已构建 tl_example 包）:
    ros2 run tl_example test_all_interfaces
@attention 请先启动 tl_driver 节点，如:
    ros2 launch tl_driver tl_tcb610_driver.launch.py
"""

import sys
import os
import glob
import time
import ctypes

# ---- 自动路径引导（未手动 source 环境时也能找到本地 install 与 ROS 系统包）----
_script_dir = os.path.dirname(os.path.abspath(__file__))


def _add_python_paths(base_dir):
    """将 base_dir 下生成的 Python 模块目录加入 sys.path"""
    if not base_dir or not os.path.isdir(base_dir):
        return
    for pat in ("local/lib/python3*/dist-packages", "lib/python3*/site-packages"):
        for d in glob.glob(os.path.join(base_dir, pat)):
            if d not in sys.path:
                sys.path.insert(0, d)


# 本地 colcon install 包（scripts 目录 -> 仓库根：../../..）
_repo_root = os.path.abspath(os.path.join(_script_dir, "..", "..", ".."))
_install_dir = os.path.join(_repo_root, "install")
if os.path.isdir(_install_dir):
    for _pkg in sorted(os.listdir(_install_dir)):
        _add_python_paths(os.path.join(_install_dir, _pkg))
# ROS 系统包（rclpy / std_srvs / sensor_msgs 等）
_add_python_paths("/opt/ros/humble")


def _preload_local_shared_libs(install_dir):
    """预加载本地 install 各包的动态库，使未 source 环境时 rosidl 类型支持库可正常加载

    运行时修改 LD_LIBRARY_PATH 对已运行进程无效，改用 ctypes 预加载，
    将本地消息接口库（如 libtl_ros2_interface__rosidl_*.so）及其依赖载入全局符号表，
    解决 "libtl_ros2_interface__rosidl_generator_py.so: cannot open shared object" 类错误。
    """
    if not install_dir or not os.path.isdir(install_dir):
        return
    for so in sorted(glob.glob(os.path.join(install_dir, "*", "lib", "*.so*"))):
        try:
            ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
        except Exception:
            # 依赖可能尚未加载；后续依赖就位后 import 时仍可解析，此处忽略单个失败
            pass


_preload_local_shared_libs(_install_dir)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from tl_ros2_interface.msg import (
    ArmStatus, CartesianPose, MoveCommand, ToolParam, ModbusMasterParam,
    ModbusTCPParam, ModbusRTUParam, JobFileName, ServolMove,
)
from tl_ros2_interface.srv import (
    CoordTransform, GetAllJobFileName, GetCoordNum, GetCurrentCoord,
    GetCurrentLineJointSpeed, GetCurrentMode, GetCurrentMotorTorque, GetDHParam,
    GetDigitalInputOutput, GetGlobalPos, GetJointSoftwareVersion,
    GetJointTemperature, GetJointVoltage, GetMotorCurrent, GetPosReachable,
    GetPosTransform, GetRobotJointParam, GetRobotState, GetSpeed,
    JobInsertMove, JobRun, Jogging, LogDownload, ModbusRead, ModbusWrite,
    OpenServoJ, QueueMotionMoveJ, QueueMotionSetStatus, RestoreDefaultDHParam,
    SetAxisZeroPos, SetControllerIP, SetCoordNum, SetCurrentCoord,
    SetCurrentMode, SetDHParam, SetDigitalOutput, SetDragMode, SetGlobalPos,
    SetRobotJointParam, SetSpeed, SetToolParam, SetUserCoord,
    TrackPlayback, TrackSave,
)


# ============================================================
# 工具
# ============================================================
def make_move_cmd(target_pos_value=None, coord=0, velocity=30.0, acc=20.0, dec=20.0, pl=0):
    """构造 MoveCommand"""
    cmd = MoveCommand()
    cmd.target_pos_type = 0
    cmd.target_pos_value = target_pos_value if target_pos_value is not None else [0.0] * 7
    cmd.target_pos_name = ""
    cmd.coord = coord
    cmd.velocity = velocity
    cmd.velocity_sync = 0.0
    cmd.acc = acc
    cmd.dec = dec
    cmd.pl = pl
    cmd.time = 0
    cmd.tool_num = 0
    cmd.user_num = 0
    cmd.posidtype = 0
    cmd.configuration = 0
    cmd.spin = 0
    cmd.para_sync = False
    return cmd


class DriverQuickTest(Node):
    """驱动快速自检节点 —— 参照 tx.cpp 依次测试 tl_driver 全部接口"""

    def __init__(self):
        super().__init__("test_all_interfaces")
        self.logger = self.get_logger()

        # 话题最近数据
        self.arm_run_state = "unknown"
        self.joint_state = None
        self.tcp_pose = None

        self._create_topic_subs()

    # ------------------------------------------------------------
    # 话题订阅
    # ------------------------------------------------------------
    def _create_topic_subs(self):
        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(ArmStatus, "/arm_status", self._on_arm_status, reliable)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, reliable)
        self.create_subscription(CartesianPose, "/tcp_pose", self._on_tcp_pose, reliable)

    def _on_arm_status(self, msg):
        self.arm_run_state = msg.run_state

    def _on_joint_state(self, msg):
        self.joint_state = msg

    def _on_tcp_pose(self, msg):
        self.tcp_pose = msg

    def spin_for(self, seconds):
        """短暂 spin 以处理订阅回调"""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------
    # 服务调用辅助
    # ------------------------------------------------------------
    def wait_service(self, name, srv_type, timeout_s=3.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout_s):
            self.logger.error(f"  服务 {name} 未就绪（{timeout_s}s 超时）")
            return None
        self.logger.info(f"  服务 {name} 已就绪")
        return cli

    def _call_with_timeout(self, cli, req, label, timeout_s=10.0):
        """通过 call_async + spin_until_future_complete 同步调用服务，避免阻塞挂起

        @return 响应对象；超时/异常/服务未就绪返回 None
        """
        if cli is None or not cli.service_is_ready():
            self.logger.warn(f"  [{label}] 服务未就绪，跳过")
            return None
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if future.result() is None:
            self.logger.warn(f"  ✗ {label} 调用超时/异常")
            return None
        return future.result()

    def call_trigger(self, cli, label):
        """调用 Trigger 服务并打印结果"""
        resp = self._call_with_timeout(cli, Trigger.Request(), label)
        if resp is None:
            return False
        if resp.success:
            self.logger.info(f"  ✓ {label} 成功" + (f" | {resp.message}" if resp.message else ""))
            return True
        else:
            self.logger.warn(f"  ✗ {label} 失败: {resp.message}")
            return False

    def call_srv(self, cli, req, label):
        """泛型服务调用，返回响应；打印 success/message"""
        resp = self._call_with_timeout(cli, req, label)
        if resp is None:
            return None
        if resp.success:
            self.logger.info(f"  ✓ {label} 成功")
        else:
            self.logger.warn(f"  ✗ {label} 失败: {resp.message}")
        return resp

    def skip(self, label, reason):
        self.logger.warn(f"  ⚠ [{label}] 已跳过：{reason}")

    def print_arr(self, prefix, arr, max_show=16):
        s = ", ".join(f"{v:.4g}" for v in arr[:max_show])
        if len(arr) > max_show:
            s += ", ..."
        self.logger.info(f"      {prefix}[{len(arr)}] = [{s}]")

    # ------------------------------------------------------------
    # §4 信息查询 —— 话题
    # ------------------------------------------------------------
    def test_topics(self):
        self.logger.info("\n========== §4 信息查询（话题）==========")
        self.logger.info("  --- /arm_status（运行状态）---")
        self.spin_for(2.0)
        self.logger.info(f"      run_state = {self.arm_run_state}")

        self.logger.info("  --- /joint_states（关节角度, rad）---")
        self.spin_for(2.0)
        if self.joint_state is not None:
            self.logger.info(f"      关节数 = {len(self.joint_state.position)}")
            self.print_arr("      position(rad) ", self.joint_state.position)
        else:
            self.logger.warn("      未收到 /joint_states")

        self.logger.info("  --- /tcp_pose（末端位姿）---")
        self.spin_for(2.0)
        if self.tcp_pose is not None:
            p = self.tcp_pose.position
            r = self.tcp_pose.rpy
            self.logger.info(f"      pos = [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}] m")
            self.logger.info(f"      rpy = [{r.x:.3f}, {r.y:.3f}, {r.z:.3f}] rad")
            self.logger.info(f"      arm_angle = {self.tcp_pose.arm_angle:.3f}")
        else:
            self.logger.warn("      未收到 /tcp_pose")

    # ------------------------------------------------------------
    # 各分组测试
    # ------------------------------------------------------------
    def test_info_query(self):
        self.logger.info("\n========== §4 信息查询（服务）==========")

        cli = self.wait_service("/tl_driver/get_speed", GetSpeed)
        resp = self.call_srv(cli, GetSpeed.Request(), "get_speed")
        if resp and resp.success:
            self.logger.info(f"      speed = {resp.speed:.1f}%")

        cli = self.wait_service("/tl_driver/get_robot_state", GetRobotState)
        req = GetRobotState.Request()
        req.channel = 1
        req.stop = False
        req.mode = 0
        req.interval = 100
        req.io_state = False
        req.position = 0
        req.detail_motion_pos = False
        req.pos_sum = 1
        resp = self.call_srv(cli, req, "get_robot_state")
        if resp and resp.success:
            self.logger.info(f"      信息: {resp.message}")

        cli = self.wait_service("/tl_driver/get_robot_joint_param", GetRobotJointParam)
        req = GetRobotJointParam.Request()
        req.id = 1
        resp = self.call_srv(cli, req, "get_robot_joint_param(id=1)")
        if resp and resp.success:
            p = resp.param
            self.logger.info(f"      减速比={p.reduction_ratio:.3f} 限位[{p.neg_sw_limit:.1f},{p.pos_sw_limit:.1f}] 额定速度={p.rated_vel:.1f}")

        cli = self.wait_service("/tl_driver/get_joint_temperature", GetJointTemperature)
        resp = self.call_srv(cli, GetJointTemperature.Request(), "get_joint_temperature")
        if resp and resp.success:
            self.print_arr("      temperature(℃) ", resp.temperatures)

        cli = self.wait_service("/tl_driver/get_joint_voltage", GetJointVoltage)
        resp = self.call_srv(cli, GetJointVoltage.Request(), "get_joint_voltage")
        if resp and resp.success:
            self.print_arr("      joint_voltage(V) ", resp.joint_voltage)
            self.print_arr("      positioner_voltage(V) ", resp.positioner_voltage)

        cli = self.wait_service("/tl_driver/get_motor_current", GetMotorCurrent)
        resp = self.call_srv(cli, GetMotorCurrent.Request(), "get_motor_current")
        if resp and resp.success:
            self.print_arr("      motor_current ", resp.motor_current)

        cli = self.wait_service("/tl_driver/get_joint_software_version", GetJointSoftwareVersion)
        req = GetJointSoftwareVersion.Request()
        req.axis_num = 1
        self.call_srv(cli, req, "get_joint_software_version(axis=1)")

        cli = self.wait_service("/tl_driver/get_current_coord", GetCurrentCoord)
        resp = self.call_srv(cli, GetCurrentCoord.Request(), "get_current_coord")
        if resp and resp.success:
            self.logger.info(f"      coord = {resp.coord}")

        cli = self.wait_service("/tl_driver/get_coord_num", GetCoordNum)
        resp = self.call_srv(cli, GetCoordNum.Request(), "get_coord_num")
        if resp and resp.success:
            self.logger.info(f"      tool_num={resp.tool_num} user_num={resp.user_num}")

        cli = self.wait_service("/tl_driver/get_dh_param", GetDHParam)
        self.call_srv(cli, GetDHParam.Request(), "get_dh_param")

        cli = self.wait_service("/tl_driver/get_all_job_filename", GetAllJobFileName)
        resp = self.call_srv(cli, GetAllJobFileName.Request(), "get_all_job_filename")
        if resp and resp.success:
            for f in resp.robots_file:
                for name in f.file_name:
                    self.logger.info(f"      file: {name}")

        cli = self.wait_service("/tl_driver/get_pos_reachable", GetPosReachable)
        req = GetPosReachable.Request()
        req.pos = [0.0] * 7
        req.move_type = "MOVJ"
        self.call_srv(cli, req, "get_pos_reachable")

        cli = self.wait_service("/tl_driver/get_current_motor_torque", GetCurrentMotorTorque)
        resp = self.call_srv(cli, GetCurrentMotorTorque.Request(), "get_current_motor_torque")
        if resp and resp.success:
            self.print_arr("      motor_torque ", [float(v) for v in resp.motor_torque])
            self.print_arr("      motor_torque_sync ", [float(v) for v in resp.motor_torque_sync])

        cli = self.wait_service("/tl_driver/get_current_line_joint_speed", GetCurrentLineJointSpeed)
        resp = self.call_srv(cli, GetCurrentLineJointSpeed.Request(), "get_current_line_joint_speed")
        if resp and resp.success:
            self.logger.info(f"      line_speed = {resp.line_speed:.2f} mm/s")
            self.print_arr("      joint_speed ", resp.joint_speed)

    def test_basic_set(self):
        self.logger.info("\n========== §5 基础设置 ==========")
        cli = self.wait_service("/tl_driver/set_speed", SetSpeed)
        req = SetSpeed.Request()
        req.speed = 30.0
        self.call_srv(cli, req, "set_speed(30)")

        cli = self.wait_service("/tl_driver/coord_transform", CoordTransform)
        req = CoordTransform.Request()
        req.origin_coord = 0
        req.target_coord = 1
        req.form = 0
        req.origin_pos = [0.0] * 7
        req.reference_pos = []
        resp = self.call_srv(cli, req, "coord_transform(0->1)")
        if resp and resp.success:
            self.print_arr("      target_pos ", resp.target_pos)

    def test_jobs(self):
        self.logger.info("\n========== §6 作业（仅插入，运行/删除跳过）==========")
        # 4 个插入指令服务
        insert_cmds = [
            ("/tl_driver/job_insert_moveJ", "job_insert_moveJ"),
            ("/tl_driver/job_insert_moveL", "job_insert_moveL"),
            ("/tl_driver/job_insert_imove", "job_insert_imove"),
            ("/tl_driver/job_insert_moveC", "job_insert_moveC"),
        ]
        for srv_name, label in insert_cmds:
            cli = self.wait_service(srv_name, JobInsertMove)
            req = JobInsertMove.Request()
            req.line = 0
            req.cmd = make_move_cmd()
            self.call_srv(cli, req, label)

        # 危险：作业运行/删除
        for srv_name, label in [("/tl_driver/job_run", "job_run"), ("/tl_driver/job_delete", "job_delete")]:
            cli = self.wait_service(srv_name, JobRun, timeout_s=1.0)
            if cli:
                self.skip(label, "会运行/删除作业文件，仅做就绪检查")
            else:
                self.logger.warn(f"  [{label}] 服务未就绪")

    def test_coord_tool(self):
        self.logger.info("\n========== §8 坐标系与工具 ==========")
        cli = self.wait_service("/tl_driver/set_tool_param", SetToolParam)
        req = SetToolParam.Request()
        req.tool_num = 1
        req.param = ToolParam()
        self.call_srv(cli, req, "set_tool_param(tool=1)")

        cli = self.wait_service("/tl_driver/set_user_coord", SetUserCoord)
        req = SetUserCoord.Request()
        req.user_num = 1
        req.pos = CartesianPose()
        self.call_srv(cli, req, "set_user_coord(user=1)")

        cli = self.wait_service("/tl_driver/set_coord_num", SetCoordNum)
        req = SetCoordNum.Request()
        req.tool_num = 1
        req.user_num = 1
        self.call_srv(cli, req, "set_coord_num(tool=1,user=1)")

        cli = self.wait_service("/tl_driver/set_current_coord", SetCurrentCoord)
        req = SetCurrentCoord.Request()
        req.coord = 1
        self.call_srv(cli, req, "set_current_coord(1)")

    def test_jog_track(self):
        self.logger.info("\n========== §9 示教 / §10 错误 ==========")
        cli = self.wait_service("/tl_driver/stop_jogging", Jogging)
        req = Jogging.Request()
        req.axis = 1
        req.direction = False
        self.call_srv(cli, req, "stop_jogging")

        cli = self.wait_service("/tl_driver/set_drag_mode", SetDragMode)
        req = SetDragMode.Request()
        req.mode = 0
        self.call_srv(cli, req, "set_drag_mode(0)")

        cli = self.wait_service("/tl_driver/track_save", TrackSave)
        req = TrackSave.Request()
        req.traj_name = "test_traj"
        self.call_srv(cli, req, "track_save")

        cli = self.wait_service("/tl_driver/clear_error", Trigger)
        self.call_trigger(cli, "clear_error")

        # 危险：start_jogging / track_playback
        cli = self.wait_service("/tl_driver/start_jogging", Jogging, timeout_s=1.0)
        if cli:
            self.skip("start_jogging", "会触发运动")
        cli = self.wait_service("/tl_driver/track_playback", TrackPlayback, timeout_s=1.0)
        if cli:
            self.skip("track_playback", "会触发运动")

    def test_global_pos(self):
        self.logger.info("\n========== §11 全局路点 ==========")
        cli = self.wait_service("/tl_driver/set_global_pos", SetGlobalPos)
        req = SetGlobalPos.Request()
        req.pos_name = "GP0001"
        req.pos_info = [1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 300.0, 0.0, 200.0, 3.14, 0.0, 0.0, 0.0]
        self.call_srv(cli, req, "set_global_pos(GP0001)")

        cli = self.wait_service("/tl_driver/get_global_pos", GetGlobalPos)
        req = GetGlobalPos.Request()
        req.pos_name = "GP0001"
        resp = self.call_srv(cli, req, "get_global_pos(GP0001)")
        if resp and resp.success:
            self.print_arr("      pos ", resp.pos)

    def test_mode_io(self):
        self.logger.info("\n========== §12 模式与 IO ==========")
        cli = self.wait_service("/tl_driver/set_current_mode", SetCurrentMode)
        req = SetCurrentMode.Request()
        req.mode = 1
        self.call_srv(cli, req, "set_current_mode(1)")

        cli = self.wait_service("/tl_driver/get_current_mode", GetCurrentMode)
        resp = self.call_srv(cli, GetCurrentMode.Request(), "get_current_mode")
        if resp and resp.success:
            self.logger.info(f"      mode = {resp.mode}")

        cli = self.wait_service("/tl_driver/set_digital_output", SetDigitalOutput)
        req = SetDigitalOutput.Request()
        req.port = 1
        req.value = 1
        self.call_srv(cli, req, "set_digital_output(port=1,val=1)")
        req.value = 0
        self.call_srv(cli, req, "set_digital_output(port=1,val=0)")

        cli = self.wait_service("/tl_driver/get_digital_input_output", GetDigitalInputOutput)
        resp = self.call_srv(cli, GetDigitalInputOutput.Request(), "get_digital_input_output")
        if resp and resp.success:
            self.print_arr("      input ", [float(v) for v in resp.input])
            self.print_arr("      output ", [float(v) for v in resp.output])

    def test_modbus(self):
        self.logger.info("\n========== §13 Modbus（RTU）==========")

        # ---- RTU 主站参数（请自行配置实际参数）----
        mp = ModbusMasterParam()
        mp.type = "RTU"                # 主站类型：RTU
        mp.start_addr = True          # false:起始地址为1；true:起始地址为0
        mp.tcp = ModbusTCPParam()      # TCP 参数（本次用 RTU，无需配置）
        mp.rtu = ModbusRTUParam()
        mp.rtu.slave_id = 1            # 从站号（请自行修改）
        mp.rtu.port = 2               # 端口（请自行修改）
        mp.rtu.baudrate = 115200         # 波特率（请自行修改）
        mp.rtu.data_bit = 8            # 数据位（请自行修改）
        mp.rtu.stop_bit = 1            # 停止位（请自行修改）
        mp.rtu.check_bit = "None"      # 校验位 None/Even/Odd（请自行修改）

        master_id = 1                  # 配方 id（最多保存 9 个）

        # ---- 批量写：每个元素为 (起始地址, 寄存器值列表)，请自行填写 ----
        batch_writes = [
            # (addr, data)
            (10, [1000]),            # 示例：从地址 0 开始连续写 3 个寄存器
            # (10, [0x1234]),          # 示例2：从地址 10 写 1 个寄存器
            # (20, [1, 0, 1, 0, 1]),   # 示例3：从地址 20 连续写 5 个寄存器
        ]
        cli = self.wait_service("/tl_driver/modbus_write", ModbusWrite)
        for addr, data in batch_writes:
            req = ModbusWrite.Request()
            req.master_id = master_id
            req.addr = addr
            req.data = data
            req.master_param = mp
            self.call_srv(cli, req, f"modbus_write(addr={addr}, n={len(data)})")

        # ---- 读测试（可选，请自行配置起始地址/数量）----
        cli = self.wait_service("/tl_driver/modbus_read", ModbusRead)
        req = ModbusRead.Request()
        req.master_id = master_id
        req.addr = 10                   # 起始地址（请自行修改）
        req.quantity = 5              # 读取数量（请自行修改）
        req.master_param = mp
        resp = self.call_srv(cli, req, "modbus_read")
        if resp and resp.success:
            self.print_arr("      data ", [float(v) for v in resp.data])

    def test_queue(self):
        self.logger.info("\n========== §14 队列运动 ==========")
        cli = self.wait_service("/tl_driver/queue_motion_set_status", QueueMotionSetStatus)
        req = QueueMotionSetStatus.Request()
        req.status = True
        self.call_srv(cli, req, "queue_motion_set_status(True)")
        req.status = False
        self.call_srv(cli, req, "queue_motion_set_status(False)")

        cli = self.wait_service("/tl_driver/queue_motion_stop", Trigger)
        self.call_trigger(cli, "queue_motion_stop")

        cli = self.wait_service("/tl_driver/queue_motion_movej", QueueMotionMoveJ, timeout_s=1.0)
        if cli:
            self.skip("queue_motion_movej", "会触发运动")

    def test_servoj(self):
        self.logger.info("\n========== §15 ServoJ ==========")
        cli = self.wait_service("/tl_driver/close_servoj", Trigger)
        self.call_trigger(cli, "close_servoj")

        cli = self.wait_service("/tl_driver/open_servoj", OpenServoJ, timeout_s=1.0)
        if cli:
            self.skip("open_servoj", "会触发运动")

    def test_pose_transform(self):
        self.logger.info("\n========== §16 位姿转换 ==========")
        cases = [
            ("/tl_driver/get_quat2rpy", [0.0, 0.0, 0.0, 1.0]),
            ("/tl_driver/get_rpy2quat", [0.0, 0.0, 0.0]),
            ("/tl_driver/get_rpy2r", [0.0, 0.0, 0.0]),
            ("/tl_driver/get_tr2r", [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
            ("/tl_driver/get_r2tr", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
        ]
        for srv_name, inp in cases:
            cli = self.wait_service(srv_name, GetPosTransform)
            req = GetPosTransform.Request()
            req.input = [float(v) for v in inp]
            resp = self.call_srv(cli, req, srv_name.split("/")[-1])
            if resp and resp.success:
                self.print_arr("      output ", resp.output)

    def test_dangerous(self):
        self.logger.info("\n========== 极高风险接口汇总（均跳过）==========")
        for label, reason in [
            ("moveJ/moveL 话题", "会触发运动"),
            ("set_axis_zero_pos", "改动零点标定"),
            ("set_dh_param", "改动 DH 参数"),
            ("set_robot_joint_param", "改动关节参数"),
            ("set_controller_ip", "修改控制器网络"),
            ("restore_default_dh_param", "恢复出厂 DH"),
            ("set_default_cartesian_param", "重置笛卡尔参数"),
        ]:
            self.skip(label, reason)

    # ------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------
    def run(self):
        self.logger.info("============================================================")
        self.logger.info("  驱动快速自检节点启动：依次测试 tl_driver 全部接口")
        self.logger.info("============================================================")

        # 核心服务就绪检查
        cli_connect = self.wait_service("/tl_driver/connect_arm", Trigger)
        cli_version = self.wait_service("/tl_driver/get_library_version", Trigger)
        if cli_connect is None or cli_version is None:
            self.logger.error("tl_driver 核心服务未就绪，请先启动 tl_driver 节点")
            return

        # ============================================================
        # 测试全部接口（危险和运动接口已跳过）
        # ============================================================
        self.logger.info("\n========== §2 连接管理 ==========")
        self.call_trigger(cli_version, "get_library_version（库版本）")
        cli_nex = self.wait_service("/tl_driver/get_nexmotion_lib_version", Trigger)
        self.call_trigger(cli_nex, "get_nexmotion_lib_version（算法库版本）")
        cli_id = self.wait_service("/tl_driver/get_controller_id", Trigger)
        self.call_trigger(cli_id, "get_controller_id（控制器序列号）")
        self.call_trigger(cli_connect, "connect_arm（连接机械臂）")

        self.test_topics()  # §4 话题（连接后才有数据）

        self.logger.info("\n========== §3 日志管理 ==========")
        cli = self.wait_service("/tl_driver/log_download", LogDownload)
        req = LogDownload.Request()
        req.count = 1
        req.directory_path = "/tmp"
        self.call_srv(cli, req, "log_download")

        self.test_info_query()
        self.test_basic_set()
        self.test_jobs()
        self.test_coord_tool()
        self.test_jog_track()
        self.test_global_pos()
        self.test_mode_io()
        self.test_modbus()
        self.test_queue()
        self.test_servoj()
        self.test_pose_transform()
        self.test_dangerous()

        self.logger.info("\n========== 结束 ==========")
        # disconnect_arm 不自动调用（由用户手动断开）
        self.skip("disconnect_arm", "不自动调用，请手动断开")

        self.logger.info("\n============================================================")
        self.logger.info("  驱动快速自检完成")
        self.logger.info("============================================================")


def main(args=None):
    rclpy.init(args=args)
    node = DriverQuickTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
