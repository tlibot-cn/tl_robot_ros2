# -*- coding: utf-8 -*-
# Auto-generated self_interface.py using ctypes
# DO NOT edit manually.
# Author: [hexiangyang]
# Date: 2026-05-22
# Description: tl_arm_robot control interface using ctype
import ctypes
import sys
import os
import time
# 加载动态库
_lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), '_nrc_host.so'))

# ---------- 基础类型 ----------
SOCKETFD = ctypes.c_int
Result = ctypes.c_int
BOOL = ctypes.c_bool

# ---------- 常量 ----------
TIMEOUT = -1
EXCEPTION = -2
OPERATION_NOT_ALLOWED = -3
PARAM_ERR = -4
DISCONNECT = -5
RECEIVE_FAILED = -6
SUCCESS = 0

# ---------- 结构体定义 ----------
class CRobotDHParam(ctypes.Structure):
    _fields_ = [
        ("L1", ctypes.c_double), ("L2", ctypes.c_double), ("L3", ctypes.c_double),
        ("L4", ctypes.c_double), ("L5", ctypes.c_double), ("L6", ctypes.c_double),
        ("L7", ctypes.c_double), ("L8", ctypes.c_double), ("L9", ctypes.c_double),
        ("L10", ctypes.c_double), ("L11", ctypes.c_double), ("L12", ctypes.c_double),
        ("L13", ctypes.c_double), ("L14", ctypes.c_double), ("L15", ctypes.c_double),
        ("L16", ctypes.c_double), ("L17", ctypes.c_double), ("L18", ctypes.c_double),
        ("L19", ctypes.c_double), ("L20", ctypes.c_double),
        ("Couple_Coe_1_2", ctypes.c_double), ("Couple_Coe_2_3", ctypes.c_double),
        ("Couple_Coe_3_2", ctypes.c_double), ("Couple_Coe_3_4", ctypes.c_double),
        ("Couple_Coe_4_5", ctypes.c_double), ("Couple_Coe_4_6", ctypes.c_double),
        ("Couple_Coe_5_6", ctypes.c_double), ("dynamicLimit_max", ctypes.c_int),
        ("dynamicLimit_min", ctypes.c_int), ("pitch", ctypes.c_double),
        ("sliding_lead_value", ctypes.c_double), ("uplift_lead_value", ctypes.c_double),
        ("spray_distance", ctypes.c_double), ("threeAxisDirection", ctypes.c_int),
        ("fiveAxisDirection", ctypes.c_int), ("twoAxisConversionRatio", ctypes.c_double),
        ("threeAxisConversionRatio", ctypes.c_double), ("amplificationRatio", ctypes.c_double),
        ("conversionratio_x", ctypes.c_double), ("conversionratio_y", ctypes.c_double),
        ("conversionratio_z", ctypes.c_double), ("conversionratio_J1", ctypes.c_double),
        ("conversionratio_J2", ctypes.c_double), ("conversionratio_J3", ctypes.c_double),
        ("upsideDown", ctypes.c_int), ("hanyu", ctypes.c_int)
    ]

class CRobotJointParam(ctypes.Structure):
    _fields_ = [
        ("reducRatio", ctypes.c_double), ("encoderResolution", ctypes.c_double),
        ("posSWLimit", ctypes.c_double), ("negSWLimit", ctypes.c_double),
        ("ratedRotSpeed", ctypes.c_double), ("deRatedRotSpeed", ctypes.c_double),
        ("maxRotSpeed", ctypes.c_double), ("maxDeRotSpeed", ctypes.c_double),
        ("ratedVel", ctypes.c_double), ("deRatedVel", ctypes.c_double),
        ("maxAcc", ctypes.c_double), ("maxDecel", ctypes.c_double),
        ("direction", ctypes.c_int)
    ]

class CIndependentAxisParam(ctypes.Structure):
    _fields_ = [
        ("axis_num", ctypes.c_int), ("angular_distance_conversion_ratio_value", ctypes.c_double),
        ("is_track", ctypes.c_bool), ("encoder_bit_linedit_value", ctypes.c_int),
        ("inverse_limit_value", ctypes.c_double), ("maximum_positive_speed_value", ctypes.c_double),
        ("maximum_reverse_speed_value", ctypes.c_double), ("motor_dir", ctypes.c_int),
        ("positive_limit_value", ctypes.c_double), ("rated_positive_speed_value", ctypes.c_double),
        ("rated_reverse_speed_value", ctypes.c_double), ("reduction_ratio_value", ctypes.c_double),
        ("max_acc_value", ctypes.c_double), ("max_dec_value", ctypes.c_double),
        ("select_bind_servo", ctypes.c_int)
    ]

class CIndependentAxisRun(ctypes.Structure):
    _fields_ = [
        ("axis_num", ctypes.c_int), ("vel", ctypes.c_double), ("dir", ctypes.c_int),
        ("acc", ctypes.c_double), ("dec", ctypes.c_double)
    ]

# 其他结构体（如ToolParam, MoveCmd等）可根据需要继续添加
# 由于数量较多，只定义核心结构，其余可通过指针或数组传递

# ---------- 辅助函数 ----------
def _make_func(name, argtypes, restype=ctypes.c_int):
    func = getattr(_lib, name + '_c', None)
    if func is None:
        # 尝试查找不带_c后缀的符号（部分C++函数可能直接导出）
        func = getattr(_lib, name, None)
    if func is None:
        raise AttributeError(f"Function {name} not found in library")
    func.argtypes = argtypes
    func.restype = restype
    return func

# ---------- 连接/断开 ----------
def connect_robot(ip: str, port: str) -> int:
    return _make_func('connect_robot', [ctypes.c_char_p, ctypes.c_char_p])(ip.encode(), port.encode())

def disconnect_robot(socketFd: int) -> int:
    return _make_func('disconnect_robot', [SOCKETFD])(socketFd)

def get_connection_status(socketFd: int) -> int:
    return _make_func('get_connection_status', [SOCKETFD])(socketFd)

# ---------- 伺服控制 ----------
def clear_error(socketFd: int) -> int:
    return _make_func('clear_error', [SOCKETFD])(socketFd)

def clear_error_robot(socketFd: int, robotNum: int) -> int:
    return _make_func('clear_error_robot', [SOCKETFD, ctypes.c_int])(socketFd, robotNum)

def set_servo_state(socketFd: int, state: int) -> int:
    return _make_func('set_servo_state', [SOCKETFD, ctypes.c_int])(socketFd, state)

def set_servo_state_robot(socketFd: int, robotNum: int, state: int) -> int:
    return _make_func('set_servo_state_robot', [SOCKETFD, ctypes.c_int, ctypes.c_int])(socketFd, robotNum, state)

def get_servo_state(socketFd: int) -> int:
    status = ctypes.c_int()
    ret = _make_func('get_servo_state', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(status))
    return status.value if ret == SUCCESS else ret

def get_servo_state_robot(socketFd: int, robotNum: int) -> int:
    status = ctypes.c_int()
    ret = _make_func('get_servo_state_robot', [SOCKETFD, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, robotNum, ctypes.byref(status))
    return status.value if ret == SUCCESS else ret

def set_servo_poweron(socketFd: int) -> int:
    return _make_func('set_servo_poweron', [SOCKETFD])(socketFd)

def set_servo_poweron_robot(socketFd: int, robotNum: int) -> int:
    return _make_func('set_servo_poweron_robot', [SOCKETFD, ctypes.c_int])(socketFd, robotNum)

def set_servo_poweroff(socketFd: int) -> int:
    return _make_func('set_servo_poweroff', [SOCKETFD])(socketFd)

def set_servo_poweroff_robot(socketFd: int, robotNum: int) -> int:
    return _make_func('set_servo_poweroff_robot', [SOCKETFD, ctypes.c_int])(socketFd, robotNum)

# ---------- 运动控制 ----------
def robot_movej(socketFd: int, coord: int, vel: float, acc: float, dec: float, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('robot_movej', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_double)])(
        socketFd, coord, vel, acc, dec, arr)

def robot_movel(socketFd: int, coord: int, vel: float, acc: float, dec: float, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('robot_movel', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_double)])(
        socketFd, coord, vel, acc, dec, arr)

def get_current_position(socketFd: int, coord: int) -> list:
    pos = (ctypes.c_double * 7)()
    ret = _make_func('get_current_position', [SOCKETFD, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, coord, pos)
    if ret == SUCCESS:
        return [pos[i] for i in range(7)]
    return []

def get_current_position_robot(socketFd: int, robotNum: int, coord: int) -> list:
    pos = (ctypes.c_double * 7)()
    ret = _make_func('get_current_position_robot', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, robotNum, coord, pos)
    if ret == SUCCESS:
        return [pos[i] for i in range(7)]
    return []

def set_speed(socketFd: int, speed: int) -> int:
    return _make_func('set_speed', [SOCKETFD, ctypes.c_int])(socketFd, speed)

def get_speed(socketFd: int) -> int:
    sp = ctypes.c_int()
    ret = _make_func('get_speed', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(sp))
    return sp.value if ret == SUCCESS else ret

# ---------- 运动控制（补充）----------
def get_joint_position(socketFd: int, axisNum: int = 0) -> list:
    """获取关节位置，axisNum=0 表示获取所有关节，返回7个double"""
    pos = (ctypes.c_double * 7)()
    ret = _make_func('get_joint_position', [SOCKETFD, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, axisNum, pos)
    return [pos[i] for i in range(7)] if ret == SUCCESS else []

def get_current_extra_position(socketFd: int) -> list:
    """获取外部轴当前位置，返回5个double"""
    pos = (ctypes.c_double * 5)()
    ret = _make_func('get_current_extra_position', [SOCKETFD, ctypes.POINTER(ctypes.c_double)])(socketFd, pos)
    return [pos[i] for i in range(5)] if ret == SUCCESS else []

def set_current_coord(socketFd: int, coord: int) -> int:
    return _make_func('set_current_coord', [SOCKETFD, ctypes.c_int])(socketFd, coord)

def get_current_coord(socketFd: int) -> int:
    coord = ctypes.c_int()
    ret = _make_func('get_current_coord', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(coord))
    return coord.value if ret == SUCCESS else ret

def get_current_mode(socketFd: int) -> int:
    mode = ctypes.c_int()
    ret = _make_func('get_current_mode', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(mode))
    return mode.value if ret == SUCCESS else ret

def get_robot_running_state(socketFd: int) -> int:
    state = ctypes.c_int()
    ret = _make_func('get_robot_running_state', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(state))
    return state.value if ret == SUCCESS else ret

def set_global_sync_position(socketFd: int, posName: str, posInfo: list) -> int:
    """设置全局GE点位，posInfo长度21"""
    if len(posInfo) != 21:
        raise ValueError("global sync position must have 21 elements")
    arr = (ctypes.c_double * 21)(*posInfo[:21])
    return _make_func('set_global_sync_position', [SOCKETFD, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)])(socketFd, posName.encode(), arr)

def get_global_sync_position(socketFd: int, posName: str) -> list:
    arr = (ctypes.c_double * 21)()
    ret = _make_func('get_global_sync_position', [SOCKETFD, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)])(socketFd, posName.encode(), arr)
    return [arr[i] for i in range(21)] if ret == SUCCESS else []

# ---------- 全局点位/变量 ----------
def set_global_position(socketFd: int, posName: str, posInfo: list) -> int:
    name = posName.encode()
    arr = (ctypes.c_double * 14)(*posInfo[:14])
    return _make_func('set_global_position', [SOCKETFD, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)])(socketFd, name, arr)

def get_global_position(socketFd: int, posName: str) -> list:
    name = posName.encode()
    arr = (ctypes.c_double * 14)()
    ret = _make_func('get_global_position', [SOCKETFD, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)])(socketFd, name, arr)
    if ret == SUCCESS:
        return [arr[i] for i in range(14)]
    return []

def set_global_variant(socketFd: int, varName: str, value: float) -> int:
    name = varName.encode()
    return _make_func('set_global_variant', [SOCKETFD, ctypes.c_char_p, ctypes.c_double])(socketFd, name, value)

def get_global_variant(socketFd: int, varName: str) -> float:
    name = varName.encode()
    val = ctypes.c_double()
    ret = _make_func('get_global_variant', [SOCKETFD, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)])(socketFd, name, ctypes.byref(val))
    return val.value if ret == SUCCESS else 0.0
# ---------- 坐标转换 ----------
def origin_to_target_coord(socketFd: int, originCoord: int, originPos: list, targetCoord: int) -> list:
    """原坐标值转换为其他坐标值，返回转换后的坐标列表（7个double）"""
    if len(originPos) != 7:
        raise ValueError("originPos must have 7 elements")
    o = (ctypes.c_double * 7)(*originPos[:7])
    t = (ctypes.c_double * 7)()
    ret = _make_func('get_origin_coord_to_target_coord', [SOCKETFD, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, originCoord, o, targetCoord, t)
    if ret == SUCCESS:
        return [t[i] for i in range(7)]
    return []   # 失败返回空列表

# ---------- 机器人切换 / DH / 关节参数 ----------
def set_robot_switch(socketFd: int, robot: int) -> int:
    return _make_func('set_robot_switch', [SOCKETFD, ctypes.c_int])(socketFd, robot)

def get_robot_switch(socketFd: int) -> int:
    robot = ctypes.c_int()
    ret = _make_func('get_robot_switch', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(robot))
    return robot.value if ret == SUCCESS else ret

def get_robot_dh_param(socketFd: int, param: CRobotDHParam) -> int:
    """获取机器人DH参数，成功返回0，失败返回错误码，参数通过param结构体返回"""
    return _make_func('get_robot_dh_param', [SOCKETFD, ctypes.POINTER(CRobotDHParam)])(socketFd, ctypes.byref(param))

def get_robot_joint_param(socketFd: int, joint_id: int, param: CRobotJointParam) -> int:
    return _make_func('get_robot_joint_param', [SOCKETFD, ctypes.c_int, ctypes.POINTER(CRobotJointParam)])(socketFd, joint_id, ctypes.byref(param))

# ---------- SDO ----------
def set_axis_sdo(socketFd: int, axisNum: int, index: int, subindex: int, value: int, size: int) -> int:
    return _make_func('set_axis_sdo', [SOCKETFD, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_uint])(socketFd, axisNum, index, subindex, value, size)
# ---------- 作业操作 ----------
def job_run(socketFd: int, jobName: str) -> int:
    name = jobName.encode()
    return _make_func('job_run', [SOCKETFD, ctypes.c_char_p])(socketFd, name)

def job_pause(socketFd: int) -> int:
    return _make_func('job_pause', [SOCKETFD])(socketFd)

def job_stop(socketFd: int) -> int:
    return _make_func('job_stop', [SOCKETFD])(socketFd)

def job_continue(socketFd: int) -> int:
    return _make_func('job_continue', [SOCKETFD])(socketFd)

# ---------- 作业操作（完整）----------
def robot_go_home(socketFd: int) -> int:
    return _make_func('robot_go_home', [SOCKETFD])(socketFd)

def job_get_all_jobfile_name(socketFd: int, robotNum: int, buffer, maxFileNum: int) -> int:
    """获取所有作业文件名，buffer应为二维字符数组，例如 (ctypes.c_char * 64 * maxFileNum)()"""
    return _make_func('job_get_all_jobfile_name', [SOCKETFD, ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t])(socketFd, robotNum, buffer, maxFileNum)

def job_upload_by_directory(socketFd: int, directoryPath: str) -> int:
    return _make_func('job_upload_by_directory', [SOCKETFD, ctypes.c_char_p])(socketFd, directoryPath.encode())

def job_upload_by_file(socketFd: int, filePath: str) -> int:
    return _make_func('job_upload_by_file', [SOCKETFD, ctypes.c_char_p])(socketFd, filePath.encode())

def job_sync_job_file(socketFd: int) -> int:
    return _make_func('job_sync_job_file', [SOCKETFD])(socketFd)

def job_download_by_directory(socketFd: int, directoryPath: str, isCover: bool) -> int:
    return _make_func('job_download_by_directory', [SOCKETFD, ctypes.c_char_p, ctypes.c_bool])(socketFd, directoryPath.encode(), isCover)

def log_download_by_quantity(socketFd: int, counts: int, directoryPath: str) -> int:
    return _make_func('log_download_by_quantity', [SOCKETFD, ctypes.c_int, ctypes.c_char_p])(socketFd, counts, directoryPath.encode())

def backup_system(socketFd: int) -> int:
    return _make_func('backup_system', [SOCKETFD])(socketFd)

def job_create(socketFd: int, jobName: str) -> int:
    return _make_func('job_create', [SOCKETFD, ctypes.c_char_p])(socketFd, jobName.encode())

def job_delete(socketFd: int, jobName: str) -> int:
    return _make_func('job_delete', [SOCKETFD, ctypes.c_char_p])(socketFd, jobName.encode())

def job_open(socketFd: int, jobName: str) -> int:
    return _make_func('job_open', [SOCKETFD, ctypes.c_char_p])(socketFd, jobName.encode())

def job_get_command_total_lines(socketFd: int) -> int:
    total = ctypes.c_int()
    ret = _make_func('job_get_command_total_lines', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(total))
    return total.value if ret == SUCCESS else ret

def job_get_command_content_by_line(socketFd: int, line: int, buf_size: int = 256) -> str:
    buf = ctypes.create_string_buffer(buf_size)
    ret = _make_func('job_get_command_content_by_line', [SOCKETFD, ctypes.c_int, ctypes.c_char_p])(socketFd, line, buf)
    return buf.value.decode(errors='ignore') if ret == SUCCESS else ""

def job_delete_command_by_line(socketFd: int, line: int) -> int:
    return _make_func('job_delete_command_by_line', [SOCKETFD, ctypes.c_int])(socketFd, line)

def job_step(socketFd: int, jobName: str, line: int) -> int:
    return _make_func('job_step', [SOCKETFD, ctypes.c_char_p, ctypes.c_int])(socketFd, jobName.encode(), line)

def job_run_times(socketFd: int, index: int) -> int:
    return _make_func('job_run_times', [SOCKETFD, ctypes.c_int])(socketFd, index)

def job_break_point_run(socketFd: int, jobName: str) -> int:
    return _make_func('job_break_point_run', [SOCKETFD, ctypes.c_char_p])(socketFd, jobName.encode())

def job_get_current_file(socketFd: int, buf_size: int = 64) -> str:
    buf = ctypes.create_string_buffer(buf_size)
    ret = _make_func('job_get_current_file', [SOCKETFD, ctypes.c_char_p])(socketFd, buf)
    return buf.value.decode(errors='ignore') if ret == SUCCESS else ""

def job_get_current_line(socketFd: int) -> int:
    line = ctypes.c_int()
    ret = _make_func('job_get_current_line', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(line))
    return line.value if ret == SUCCESS else ret

def job_insert_moveJ(socketFd: int, line: int, vel: float, acc: float, dec: float, pl: int, posName: str) -> int:
    return _make_func('job_insert_moveJ', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_char_p])(socketFd, line, vel, acc, dec, pl, posName.encode())

def job_insert_moveL(socketFd: int, line: int, vel: float, acc: float, dec: float, pl: int, posName: str) -> int:
    return _make_func('job_insert_moveL', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_char_p])(socketFd, line, vel, acc, dec, pl, posName.encode())

def job_insert_moveC(socketFd: int, line: int, vel: float, acc: float, dec: float, pl: int, posName: str) -> int:
    return _make_func('job_insert_moveC', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_char_p])(socketFd, line, vel, acc, dec, pl, posName.encode())
# ---------- 队列运动 ----------
def queue_motion_set_status(socketFd: int, status: bool) -> int:
    return _make_func('queue_motion_set_status', [SOCKETFD, ctypes.c_bool])(socketFd, status)

def queue_motion_send_to_controller(socketFd: int, size: int, isContinue: bool = False) -> int:
    return _make_func('queue_motion_send_to_controller', [SOCKETFD, ctypes.c_int, ctypes.c_bool])(socketFd, size, isContinue)

def queue_motion_push_back_moveJ(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('queue_motion_push_back_moveJ', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(
        socketFd, coord, vel, acc, dec, pl, arr)

def queue_motion_push_back_moveL(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('queue_motion_push_back_moveL', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(
        socketFd, coord, vel, acc, dec, pl, arr)

def queue_motion_push_back_timer(socketFd: int, time: float) -> int:
    return _make_func('queue_motion_push_back_timer', [SOCKETFD, ctypes.c_double])(socketFd, time)

def queue_motion_push_back_dout(socketFd: int, port: int, value: bool) -> int:
    return _make_func('queue_motion_push_back_dout', [SOCKETFD, ctypes.c_int, ctypes.c_bool])(socketFd, port, value)
# ---------- 队列运动（补充）----------
def queue_motion_suspend(socketFd: int) -> int:
    return _make_func('queue_motion_suspend', [SOCKETFD])(socketFd)

def queue_motion_restart(socketFd: int) -> int:
    return _make_func('queue_motion_restart', [SOCKETFD])(socketFd)

def queue_motion_push_back_moveC(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('queue_motion_push_back_moveC', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, coord, vel, acc, dec, pl, arr)

def queue_motion_push_back_moveCA(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('queue_motion_push_back_moveCA', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, coord, vel, acc, dec, pl, arr)

def queue_motion_push_back_moveS(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    arr = (ctypes.c_double * 7)(*targetPos[:7])
    return _make_func('queue_motion_push_back_moveS', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, coord, vel, acc, dec, pl, arr)

def queue_motion_push_back_moveJ_extra(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    """外部轴关节运动，targetPos长度应为14（本体7+外部轴7）"""
    if len(targetPos) != 14:
        raise ValueError("moveJ_extra targetPos must have 14 elements")
    arr = (ctypes.c_double * 14)(*targetPos[:14])
    return _make_func('queue_motion_push_back_moveJ_extra', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, coord, vel, acc, dec, pl, arr)

def queue_motion_push_back_moveL_extra(socketFd: int, coord: int, vel: float, acc: float, dec: float, pl: int, targetPos: tuple) -> int:
    if len(targetPos) != 14:
        raise ValueError("moveL_extra targetPos must have 14 elements")
    arr = (ctypes.c_double * 14)(*targetPos[:14])
    return _make_func('queue_motion_push_back_moveL_extra', [SOCKETFD, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, coord, vel, acc, dec, pl, arr)
# ---------- 独立轴控制 ----------
def new_independent_axis_param(socketFd: int, axis_num: int) -> int:
    return _make_func('new_independent_axis_param', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def modify_independent_axis_param(socketFd: int, param: CIndependentAxisParam) -> int:
    return _make_func('modify_independent_axis_param', [SOCKETFD, ctypes.POINTER(CIndependentAxisParam)])(socketFd, ctypes.byref(param))

def get_independent_axis_param(socketFd: int, axis_num: int) -> CIndependentAxisParam:
    param = CIndependentAxisParam()
    ret = _make_func('get_independent_axis_param', [SOCKETFD, ctypes.c_int, ctypes.POINTER(CIndependentAxisParam)])(socketFd, axis_num, ctypes.byref(param))
    return param if ret == SUCCESS else None

def get_axis_sum(socketFd: int) -> int:
    total = ctypes.c_int()
    ret = _make_func('get_axis_sum', [SOCKETFD, ctypes.POINTER(ctypes.c_int)])(socketFd, ctypes.byref(total))
    return total.value if ret == SUCCESS else ret

def delete_axis_sum(socketFd: int, axis_num: int) -> int:
    return _make_func('delete_axis_sum', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def get_axis_position(socketFd: int, axis_num: int) -> float:
    pos = ctypes.c_double()
    ret = _make_func('get_axis_position', [SOCKETFD, ctypes.c_int, ctypes.POINTER(ctypes.c_double)])(socketFd, axis_num, ctypes.byref(pos))
    return pos.value if ret == SUCCESS else 0.0

def independent_axis_zero_calibration(socketFd: int, axis_num: int) -> int:
    return _make_func('independent_axis_zero_calibration', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def set_independent_axis_PV_run(socketFd: int, param: CIndependentAxisRun) -> int:
    return _make_func('set_independent_axis_PV_run', [SOCKETFD, ctypes.POINTER(CIndependentAxisRun)])(socketFd, ctypes.byref(param))

def set_independent_axis_PV_stop(socketFd: int, axis_num: int) -> int:
    return _make_func('set_independent_axis_PV_stop', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def cancel_independent_axis_move(socketFd: int, axis_num: int) -> int:
    return _make_func('cancel_independent_axis_move', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def independent_axis_homing(socketFd: int, axis_num: int) -> int:
    return _make_func('independent_axis_homing', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def independent_axis_homing_stop(socketFd: int, axis_num: int) -> int:
    return _make_func('independent_axis_homing_stop', [SOCKETFD, ctypes.c_int])(socketFd, axis_num)

def independent_axis_jog(socketFd: int, param: CIndependentAxisRun) -> int:
    return _make_func('independent_axis_jog', [SOCKETFD, ctypes.POINTER(CIndependentAxisRun)])(socketFd, ctypes.byref(param))
# ---------- 模式设置 ----------
def set_current_mode(socketFd: int, mode: int) -> int:
    """设置机器人当前模式 0:示教 1:远程 2:运行"""
    return _make_func('set_current_mode', [SOCKETFD, ctypes.c_int])(socketFd, mode)

_wrapper = ctypes.CDLL(os.path.join(os.path.dirname(__file__), 'libservoJ_wrapper.so'))

def open_servoJ(socketFd: int, vmax: list, amax: list, jmax: list) -> int:
    n_v = len(vmax)
    n_a = len(amax)
    n_j = len(jmax)
    arr_v = (ctypes.c_double * n_v)(*vmax)
    arr_a = (ctypes.c_double * n_a)(*amax)
    arr_j = (ctypes.c_double * n_j)(*jmax)
    _wrapper.open_servoJ_wrapper.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                             ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                             ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    _wrapper.open_servoJ_wrapper.restype = ctypes.c_int
    return _wrapper.open_servoJ_wrapper(socketFd, arr_v, n_v, arr_a, n_a, arr_j, n_j)

def close_servoJ(socketFd: int) -> int:
    _wrapper.close_servoJ_wrapper.argtypes = [ctypes.c_int]
    _wrapper.close_servoJ_wrapper.restype = ctypes.c_int
    return _wrapper.close_servoJ_wrapper(socketFd)

def set_servoJ_pos(socketFd: int, q: list) -> int:
    n_q = len(q)
    arr_q = (ctypes.c_double * n_q)(*q)
    _wrapper.set_servoJ_pos_wrapper.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    _wrapper.set_servoJ_pos_wrapper.restype = ctypes.c_int
    return _wrapper.set_servoJ_pos_wrapper(socketFd, arr_q, n_q)
# ==================== Modbus 接口（通过包装库） ====================
_modbus_lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), 'libmodbus_wrapper.so'))

def _modbus_func(name, argtypes, restype=ctypes.c_int):
    func = getattr(_modbus_lib, name + '_wrapper', None)
    if func is None:
        raise AttributeError(f"Modbus wrapper function {name}_wrapper not found")
    func.argtypes = argtypes
    func.restype = restype
    return func

# 末端 IO
def set_terminal_io_can_param(socketFd: int, id: int, port: int, arbitration_baudrate: int, data_baudrate: int) -> int:
    return _modbus_func('set_terminal_io_can_param', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int])(socketFd, id, port, arbitration_baudrate, data_baudrate)

def set_terminal_io_485_param(socketFd: int, id: int, port: int, baudrate: int) -> int:
    return _modbus_func('set_terminal_io_485_param', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int])(socketFd, id, port, baudrate)

def terminal_io_read_modbus_data(socketFd: int, func_code: int, start_addr: int, read_addr_num: int) -> list:
    arr = (ctypes.c_int * read_addr_num)()
    ret = _modbus_func('terminal_io_read_modbus_data', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, func_code, start_addr, read_addr_num, arr)
    if ret == SUCCESS:
        return [arr[i] for i in range(read_addr_num)]
    return []

def terminal_io_write_modbus_data(socketFd: int, func_code: int, start_addr: int, write_data: list) -> int:
    n = len(write_data)
    arr = (ctypes.c_int * n)(*write_data)
    return _modbus_func('terminal_io_write_modbus_data', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, func_code, start_addr, n, arr)

# 主站配置
def modbus_set_master_parameter_tcp(socketFd: int, id: int, ip: str, port: int, startAddress: bool = False) -> int:
    """设置 Modbus TCP 主站参数"""
    return _modbus_func('modbus_set_master_parameter_simple', [
        SOCKETFD, ctypes.c_int,
        ctypes.c_char_p,          # type
        ctypes.c_bool,            # startAddress
        ctypes.c_char_p,          # ip
        ctypes.c_int,             # port
        ctypes.c_int,             # slaveId (忽略)
        ctypes.c_int,             # rtu_port (忽略)
        ctypes.c_int,             # baudrate (忽略)
        ctypes.c_char_p,          # checkBit (忽略)
        ctypes.c_int,             # dataBit (忽略)
        ctypes.c_int              # stopBit (忽略)
    ])(socketFd, id, b"TCP", startAddress, ip.encode(), port, 0, 0, 0, b"None", 8, 1)
def modbus_set_master_parameter_rtu(socketFd: int, id: int, slaveId: int, rtu_port: int, baudrate: int,
                                    checkBit: str = "None", dataBit: int = 8, stopBit: int = 1,
                                    startAddress: bool = False) -> int:
    """设置 Modbus RTU 主站参数"""
    return _modbus_func('modbus_set_master_parameter_rtu_simple', [
        SOCKETFD, ctypes.c_int,
        ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_bool
    ])(socketFd, id, slaveId, rtu_port, baudrate, checkBit.encode(), dataBit, stopBit, startAddress)

def modbus_open_master(socketFd: int, id: int) -> int:
    return _modbus_func('modbus_open_master', [SOCKETFD, ctypes.c_int])(socketFd, id)

def modbus_get_master_connection_status(socketFd: int, id: int) -> int:
    status = ctypes.c_int()
    ret = _modbus_func('modbus_get_master_connection_status', [SOCKETFD, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, ctypes.byref(status))
    return status.value if ret == SUCCESS else ret

# 读写函数（返回列表）
def modbus_read_coil_status(socketFd: int, id: int, address: int, quantity: int) -> list:
    arr = (ctypes.c_int * quantity)()
    ret = _modbus_func('modbus_read_coil_status', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, address, quantity, arr)
    if ret == SUCCESS:
        return [arr[i] for i in range(quantity)]
    return []

def modbus_read_input_status(socketFd: int, id: int, address: int, quantity: int) -> list:
    arr = (ctypes.c_int * quantity)()
    ret = _modbus_func('modbus_read_input_status', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, address, quantity, arr)
    if ret == SUCCESS:
        return [arr[i] for i in range(quantity)]
    return []

def modbus_read_holding_registers(socketFd: int, id: int, address: int, quantity: int) -> list:
    arr = (ctypes.c_int * quantity)()
    ret = _modbus_func('modbus_read_holding_registers', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, address, quantity, arr)
    if ret == SUCCESS:
        return [arr[i] for i in range(quantity)]
    return []

def modbus_read_input_registers(socketFd: int, id: int, address: int, quantity: int) -> list:
    arr = (ctypes.c_int * quantity)()
    ret = _modbus_func('modbus_read_input_registers', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, address, quantity, arr)
    if ret == SUCCESS:
        return [arr[i] for i in range(quantity)]
    return []

# 单点写入
def modbus_write_signal_coil_status(socketFd: int, id: int, address: int, data: int) -> int:
    return _modbus_func('modbus_write_signal_coil_status', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int])(socketFd, id, address, data)

def modbus_write_signal_holding_registers(socketFd: int, id: int, address: int, data: int) -> int:
    return _modbus_func('modbus_write_signal_holding_registers', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int])(socketFd, id, address, data)

# 多点写入
def modbus_write_multiple_coil_status(socketFd: int, id: int, address: int, data: list) -> int:
    n = len(data)
    arr = (ctypes.c_int * n)(*data)
    return _modbus_func('modbus_write_multiple_coil_status', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, address, n, arr)

def modbus_write_multiple_holding_registers(socketFd: int, id: int, address: int, data: list) -> int:
    n = len(data)
    arr = (ctypes.c_int * n)(*data)
    return _modbus_func('modbus_write_multiple_holding_registers', [SOCKETFD, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)])(socketFd, id, address, n, arr)


# ---------- 其他函数可根据需要继续添加 ----------
# 由于头文件中函数数量庞大，以上仅列出了最常用的函数。
# 如果需要全部函数，请参照上述模式为每个 `_c` 函数添加 Python 包装。
# 所有导出函数的名称和参数类型已在头文件中定义。

# 为了完整性，这里提供一个自动生成所有函数的方法（使用库中的符号表），
# 但限于篇幅，此文件已包含核心功能。

# 导出所有 public 符号
__all__ = [
    # 连接/断开
    'connect_robot', 'disconnect_robot', 'get_connection_status',
    # 错误清除
    'clear_error', 'clear_error_robot',
    # 伺服控制
    'set_servo_state', 'get_servo_state', 'set_servo_poweron', 'set_servo_poweroff',
    # 运动控制
    'robot_movej', 'robot_movel', 'get_current_position', 'set_speed', 'get_speed',
    'get_joint_position', 'get_current_extra_position',
    'set_current_coord', 'get_current_coord', 'set_current_mode', 'get_current_mode',
    'get_robot_running_state',
    # 全局点位/变量
    'set_global_position', 'get_global_position', 'set_global_sync_position', 'get_global_sync_position',
    'set_global_variant', 'get_global_variant',
    # 坐标转换
    'origin_to_target_coord',
    # 机器人切换/DH/关节参数
    'set_robot_switch', 'get_robot_switch', 'get_robot_dh_param', 'get_robot_joint_param',
    # SDO
    'set_axis_sdo',
    # 独立轴
    'new_independent_axis_param', 'modify_independent_axis_param', 'get_independent_axis_param',
    'get_axis_sum', 'delete_axis_sum', 'get_axis_position',
    'independent_axis_zero_calibration', 'set_independent_axis_PV_run', 'set_independent_axis_PV_stop',
    'cancel_independent_axis_move', 'independent_axis_homing', 'independent_axis_homing_stop',
    'independent_axis_jog',
    # 作业操作
    'robot_go_home',
    'job_get_all_jobfile_name', 'job_upload_by_directory', 'job_upload_by_file', 'job_sync_job_file',
    'job_download_by_directory', 'log_download_by_quantity', 'backup_system',
    'job_create', 'job_delete', 'job_open', 'job_run', 'job_pause', 'job_stop', 'job_continue',
    'job_step', 'job_run_times', 'job_break_point_run',
    'job_get_command_total_lines', 'job_get_command_content_by_line', 'job_delete_command_by_line',
    'job_get_current_file', 'job_get_current_line',
    'job_insert_moveJ', 'job_insert_moveL', 'job_insert_moveC',
    # 队列运动
    'queue_motion_set_status', 'queue_motion_send_to_controller',
    'queue_motion_suspend', 'queue_motion_restart', 'queue_motion_stop_not_power_off',
    'queue_motion_push_back_moveJ', 'queue_motion_push_back_moveL', 'queue_motion_push_back_moveC',
    'queue_motion_push_back_moveCA', 'queue_motion_push_back_moveS',
    'queue_motion_push_back_moveJ_extra', 'queue_motion_push_back_moveL_extra',
    'queue_motion_push_back_timer', 'queue_motion_push_back_dout',
    # 伺服J（需要包装库）
    'open_servoJ', 'close_servoJ', 'set_servoJ_pos',
    # 常量
    'SUCCESS', 'TIMEOUT', 'EXCEPTION', 'OPERATION_NOT_ALLOWED',
    'PARAM_ERR', 'DISCONNECT', 'RECEIVE_FAILED'
]
__all__ += [
    'set_terminal_io_can_param', 'set_terminal_io_485_param',
    'terminal_io_read_modbus_data', 'terminal_io_write_modbus_data',
    'modbus_set_master_parameter', 'modbus_open_master', 'modbus_get_master_connection_status',
    'modbus_read_coil_status', 'modbus_read_input_status',
    'modbus_read_holding_registers', 'modbus_read_input_registers',
    'modbus_write_signal_coil_status', 'modbus_write_signal_holding_registers',
    'modbus_write_multiple_coil_status', 'modbus_write_multiple_holding_registers'
]
# ==================== 高级封装函数（二次封装） ====================
import time

def power_on(socketFd: int) -> int:
    """
    伺服上电（自动处理状态转换）
    返回最终伺服状态
    """
    state = get_servo_state(socketFd)
    if state == 0:          # 停止 → 先设为就绪，再上电
        set_servo_state(socketFd, 1)
        set_servo_poweron(socketFd)
    elif state == 1:        # 就绪 → 直接上电
        set_servo_poweron(socketFd)
    elif state == 2:        # 报警 → 清错，设就绪，上电
        clear_error(socketFd)
        set_servo_state(socketFd, 1)
        set_servo_poweron(socketFd)
    elif state == 3:        # 运行中，无需操作
        pass
    else:
        print(f"当前状态 {state}，无法上电，请检查")
        return state
    final_state = get_servo_state(socketFd)
    print(f"[上电完成] 伺服状态: {final_state}")
    return final_state

def power_off(socketFd: int) -> int:
    """
    伺服下电
    返回最终伺服状态
    """
    state = get_servo_state(socketFd)
    if state == 3:          # 运行中 → 直接下电
        set_servo_poweroff(socketFd)
    else:
        print(f"当前状态 {state}，无需下电或无法下电")
    final_state = get_servo_state(socketFd)
    print(f"[下电完成] 伺服状态: {final_state}")
    return final_state

def queue_moveJ(socketFd: int, coord: int, vel: float, acc: float, dec: float,
                pl: int, pos_list: list, wait_time: float = 10) -> None:
    """
    使用队列模式执行一次 MoveJ 运动（自动开启/关闭队列）

    参数:
        socketFd: 连接句柄
        coord: 坐标系 (0:关节, 1:直角)
        vel: 速度 (%)
        acc: 加速度 (%)
        dec: 减速度 (%)
        pl: 平滑系数 (0~5)
        pos_list: 目标位置列表 (7个浮点数)
        wait_time: 等待运动完成的时间(秒)
    """
    # 1. 开启队列模式
    queue_motion_set_status(socketFd, True)
    time.sleep(2)

    # 2. 添加 MoveJ 指令到队列
    queue_motion_push_back_moveJ(socketFd, coord, vel, acc, dec, pl, tuple(pos_list))

    # 3. 发送队列到控制器并开始执行
    queue_motion_send_to_controller(socketFd, 1, False)

    # 4. 等待运动完成（简化处理，实际建议轮询机器人状态）
    time.sleep(wait_time)

    # 5. 关闭队列模式
    queue_motion_set_status(socketFd, False)

    print("队列 MoveJ 执行完成")

def queue_moveL(socketFd: int, coord: int, vel: float, acc: float, dec: float,
                pl: int, pos_list: list, wait_time: float = 10) -> None:
    """
    使用队列模式执行一次 MoveL 运动（自动开启/关闭队列）

    参数:
        socketFd: 连接句柄
        coord: 坐标系 (0:关节, 1:直角)
        vel: 速度 (mm/s)
        acc: 加速度 (%)
        dec: 减速度 (%)
        pl: 平滑系数 (0~5)
        pos_list: 目标位置列表 (7个浮点数)
        wait_time: 等待运动完成的时间(秒)
    """
    queue_motion_set_status(socketFd, True)
    time.sleep(2)
    queue_motion_push_back_moveL(socketFd, coord, vel, acc, dec, pl, tuple(pos_list))
    queue_motion_send_to_controller(socketFd, 1, False)
    time.sleep(wait_time)
    queue_motion_set_status(socketFd, False)
    print("队列 MoveL 执行完成")

def queue_moveC(socketFd: int, coord: int, vel: float, acc: float, dec: float,
                pl: int, pos_list: list, wait_time: float = 10) -> None:
    """
    使用队列模式执行一次 MoveC 圆弧运动（自动开启/关闭队列）
    注：此处先执行一段 MoveL 定位，再执行两次 MoveC，构成完整圆弧路径。
    """
    queue_motion_set_status(socketFd, True)
    time.sleep(2)
    # 先 MoveL 到达起点
    queue_motion_push_back_moveL(socketFd, coord, vel, acc, dec, pl, tuple(pos_list))
    # 两次 MoveC 形成圆弧
    queue_motion_push_back_moveC(socketFd, coord, vel, acc, dec, pl, tuple(pos_list))
    queue_motion_push_back_moveC(socketFd, coord, vel, acc, dec, pl, tuple(pos_list))
    queue_motion_send_to_controller(socketFd, 1, False)
    time.sleep(wait_time)
    queue_motion_set_status(socketFd, False)
    print("队列 MoveC 执行完成")

def queue_moveS(socketFd: int, coord: int, vel: float, acc: float, dec: float,
                pl: int, pos_list1: list, pos_list2: list, pos_list3: list,
                wait_time: float = 10) -> None:
    """
    使用队列模式执行 S 形运动（自动开启/关闭队列）
    顺序：MoveL → MoveS → MoveS
    """
    queue_motion_set_status(socketFd, True)
    time.sleep(2)
    queue_motion_push_back_moveL(socketFd, coord, vel, acc, dec, pl, tuple(pos_list1))
    queue_motion_push_back_moveS(socketFd, coord, vel, acc, dec, pl, tuple(pos_list2))
    queue_motion_push_back_moveS(socketFd, coord, vel, acc, dec, pl, tuple(pos_list3))
    queue_motion_send_to_controller(socketFd, 1, False)
    time.sleep(wait_time)
    queue_motion_set_status(socketFd, False)
    print("队列 MoveS 执行完成")

def queue_arc_with_moveJ(socketFd: int, coord: int, vel: float, acc: float, dec: float,
                         pl: int, moveJ_pos: list, moveC1_pos: list,
                         moveC2_pos: list, wait_time: float = 15) -> None:
    """
    队列圆弧运动：先 MoveJ 定位，再执行两个 MoveC 圆弧
    """
    queue_motion_set_status(socketFd, True)
    time.sleep(2)
    queue_motion_push_back_moveJ(socketFd, coord, vel, acc, dec, pl, tuple(moveJ_pos))
    queue_motion_push_back_moveC(socketFd, coord, vel, acc, dec, pl, tuple(moveC1_pos))
    queue_motion_push_back_moveC(socketFd, coord, vel, acc, dec, pl, tuple(moveC2_pos))
    queue_motion_send_to_controller(socketFd, 3, False)   # 至少发送3条指令
    time.sleep(wait_time)
    queue_motion_set_status(socketFd, False)
    print("队列圆弧运动 (MoveJ + 2×MoveC) 执行完成")


# 5.22更新
    # # ==================== 数学转换接口（四元数/欧拉角/旋转矩阵） ====================
_math_lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), 'libmath_wrapper.so'))

def _math_func(name, argtypes, restype=ctypes.c_int):
    func = getattr(_math_lib, name + '_wrapper', None)
    if func is None:
        raise AttributeError(f"Math wrapper function {name}_wrapper not found")
    func.argtypes = argtypes
    func.restype = restype
    return func

# 辅助函数：将 Python 列表转换为 ctypes double 数组
def _to_c_array(lst):
    return (ctypes.c_double * len(lst))(*lst)

def get_quat2rpy(socketFd: int, quat: list) -> tuple:
    """四元数转欧拉角（RPY）"""
    if len(quat) != 4:
        raise ValueError("quat must have 4 elements (qx, qy, qz, qw)")
    rpy = (ctypes.c_double * 3)()
    ret = _math_func('get_quat2rpy', [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_double), ctypes.c_int])(
        socketFd, _to_c_array(quat), 4, rpy, 3)
    if ret == SUCCESS:
        return ret, rpy[0], rpy[1], rpy[2]
    else:
        return ret, 0.0, 0.0, 0.0

def get_rpy2quat(socketFd: int, rpy: list) -> tuple:
    """欧拉角（RPY）转四元数"""
    if len(rpy) != 3:
        raise ValueError("rpy must have 3 elements (rx, ry, rz)")
    quat = (ctypes.c_double * 4)()
    ret = _math_func('get_rpy2quat', [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_double), ctypes.c_int])(
        socketFd, _to_c_array(rpy), 3, quat, 4)
    if ret == SUCCESS:
        return ret, quat[0], quat[1], quat[2], quat[3]
    else:
        return ret, 0.0, 0.0, 0.0, 0.0

def get_rpy2r(socketFd: int, rpy: list) -> tuple:
    """欧拉角（RPY）转旋转矩阵（9个元素，行主序）"""
    if len(rpy) != 3:
        raise ValueError("rpy must have 3 elements")
    r_mat = (ctypes.c_double * 9)()
    ret = _math_func('get_rpy2r', [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                   ctypes.POINTER(ctypes.c_double), ctypes.c_int])(
        socketFd, _to_c_array(rpy), 3, r_mat, 9)
    if ret == SUCCESS:
        return (ret,) + tuple(r_mat[i] for i in range(9))
    else:
        return (ret,) + (0.0,) * 9

def get_tr2r(socketFd: int, tr_mat: list) -> tuple:
    """位姿矩阵（16个元素，行主序）转旋转矩阵（9个元素）"""
    if len(tr_mat) != 16:
        raise ValueError("tr_mat must have 16 elements")
    r_mat = (ctypes.c_double * 9)()
    ret = _math_func('get_tr2r', [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                  ctypes.POINTER(ctypes.c_double), ctypes.c_int])(
        socketFd, _to_c_array(tr_mat), 16, r_mat, 9)
    if ret == SUCCESS:
        return (ret,) + tuple(r_mat[i] for i in range(9))
    else:
        return (ret,) + (0.0,) * 9

def get_r2tr(socketFd: int, r_mat: list) -> tuple:
    """旋转矩阵（9个元素，行主序）转位姿矩阵（16个元素）"""
    if len(r_mat) != 9:
        raise ValueError("r_mat must have 9 elements")
    tr_mat = (ctypes.c_double * 16)()
    ret = _math_func('get_r2tr', [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                  ctypes.POINTER(ctypes.c_double), ctypes.c_int])(
        socketFd, _to_c_array(r_mat), 9, tr_mat, 16)
    if ret == SUCCESS:
        return (ret,) + tuple(tr_mat[i] for i in range(16))
    else:
        return (ret,) + (0.0,) * 16
