#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""flange_jog_trajectory.py — 工具手偏心时法兰绕 TCP 点动的轨迹（两个半圆 = 半球）

背景
----
工具手 2 沿末端 z 轴偏置 r（默认 10 mm）：工具系原点在 TCP，法兰中心在工具系下
的固定向量 d0 = (0, 0, -r)（法兰位于 TCP 的 -z 侧）。点动旋转绕 TCP 进行，因此
法兰中心始终落在以 TCP 为球心、半径 r 的球面上：

    绕工具 X 旋转 θ ∈ [-sweep/2, +sweep/2]  → y-z 平面内半圆  d = Rx(θ)·d0
    绕工具 Y 旋转 φ ∈ [-sweep/2, +sweep/2]  → x-z 平面内半圆  d = Ry(φ)·d0

两段半圆共用起点 d0（球面最低点），合起来即用户观察到的「两个半圆组成的半球」
线框；默认 sweep=180°，每条弧就是一个标准半圆。

工具系下的轨迹与 TCP 姿态无关。要换算到基座系（base_link），需要 TCP 姿态 R：
  p = p_tcp + R·d，R_flange = R·R_jog
可用 --from-live 从 /tcp_pose 读取当前姿态，或 --rpy-deg 显式给出。

无工具复现
----------
工具切到 0 后法兰中心就是 TCP，绕 TCP 点动只改姿态、法兰不平移，所以无法靠点动
复现。等价做法是把上面算出的「法兰位姿序列」当笛卡尔轨迹执行（位置在 10 mm 球面上、
姿态按 R·R_jog 变化）。`--solve-only` 做 IK/FK 校验不动臂，`--execute` 走
/tl_driver/ik + /tl_driver/set_servoj_pos 实际运动；脚本会检查 tool_num==0，
否则 IK 会把法兰位姿当 TCP 位姿，整体偏 r。

输出（默认 scripts/flange_jog_results/<时间戳>/）
    flange_path.csv   每点：弧段、转角、工具系偏移、基座系位置、法兰姿态
    flange_path.ply   点云（intensity = 0/1 表示 X/Y 弧）
    preview.png       3D 预览（matplotlib 可用时）
    actual_tcp.csv / actual_joints.csv   实采（仅 --execute）

用法
    # 工具系轨迹（不需要位姿），输出到默认目录
    python3 scripts/flange_jog_trajectory.py

    # 基座系轨迹：给出该位姿处的 TCP 姿态（度，XYZ 外旋）
    python3 scripts/flange_jog_trajectory.py --rpy-deg 180 0 0

    # 机械臂正处于目标位姿时，直接从 /tcp_pose 取姿态
    source install/setup.bash
    python3 scripts/flange_jog_trajectory.py --from-live

    # 自定义球心 / 半径 / 扫掠角（45° → 两段 1/4 圆弧）
    python3 scripts/flange_jog_trajectory.py --center 539.53 -0.5 224.07 \\
        --radius 10 --sweep-deg 90

    # 无工具复现：工具 0、法兰停在球面起点，先只做 IK/FK 校验
    python3 scripts/flange_jog_trajectory.py --solve-only --from-live \\
        --center 539.53 -0.5 224.07

    # 校验通过后实跑（ServoJ 100 Hz）
    python3 scripts/flange_jog_trajectory.py --execute --from-live

    # 最稳的复现方式：带工具点动时录关节，切工具 0 后回放（不经过 IK）
    python3 scripts/flange_jog_trajectory.py --record-joints /tmp/jog.csv
    python3 scripts/flange_jog_trajectory.py --replay-joints /tmp/jog.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = SCRIPT_DIR / "flange_jog_results"
DEFAULT_CENTER_MM = (539.53, -0.5, 224.07)
DEFAULT_RADIUS_MM = 10.0

# 执行参数（与 scripts/sphere_ik_bench.py 保持同一套口径）
DEFAULT_RATE = 100.0  # ServoJ 下发频率 Hz
DEFAULT_SPEED_MPS = 0.02  # 沿弧线速度 m/s
DEFAULT_MAX_JOINT_STEP = 0.5  # 相邻采样点关节最大变化 rad（用于抓分支跳变）
DEFAULT_MAX_RESIDUAL_MM = 5.0  # IK 回代 FK 位置残差上限
DEFAULT_MAX_ROT_ERR_DEG = 2.0  # IK 回代 FK 姿态误差上限
PREPOSE_VELOCITY = 20.0  # 预定位 MoveJ 速度 %
PREPOSE_ACC = 20.0  # 预定位 MoveJ 加减速 %
PREPOSE_GOAL_TIMEOUT_S = 10.0
PREPOSE_MARGIN_S = 10.0
SERVOJ_SETTLE_S = 2.0


# ---------------------------------------------------------------- 旋转工具


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_R(rx: float, ry: float, rz: float) -> np.ndarray:
    """XYZ 外旋（与 tl_driver 一致）：R = Rz·Ry·Rx。"""
    return rot_z(rz) @ rot_y(ry) @ rot_x(rx)


def R_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """R = Rz·Ry·Rx 的逆解，返回 (rx, ry, rz)。

    ry=±90° 附近 (cy→0) 万向节锁：固定 rz=0，把剩余转角并入 rx。
    """
    sy = max(-1.0, min(1.0, -R[2, 0]))
    ry = math.asin(sy)
    if abs(math.cos(ry)) > 1e-8:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        rz = 0.0
    return rx, ry, rz


def R_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """返回 (x, y, z, w)。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


# ---------------------------------------------------------------- 轨迹生成


def build_arcs(radius_mm: float, sweep_deg: float, samples: int):
    """工具系内两段半圆：X 弧在 y-z 平面，Y 弧在 x-z 平面，共用起点 (0,0,-r)。

    返回 [(seg_name, jog_R_list, d_list), ...]，d 单位 mm，形状 (samples, 3)。
    """
    r = float(radius_mm)
    sweep = math.radians(float(sweep_deg))
    theta = np.linspace(-sweep / 2.0, sweep / 2.0, samples)
    zeros = np.zeros_like(theta)

    # Rx(θ)·(0,0,-r) = (0, r·sinθ, -r·cosθ)
    d_x = np.stack([zeros, r * np.sin(theta), -r * np.cos(theta)], axis=1)
    # Ry(φ)·(0,0,-r) = (-r·sinφ, 0, -r·cosφ)
    d_y = np.stack([-r * np.sin(theta), zeros, -r * np.cos(theta)], axis=1)

    arcs = []
    for name, d in (("arc_x", d_x), ("arc_y", d_y)):
        rots = [rot_x(a) if name == "arc_x" else rot_y(a) for a in theta]
        arcs.append((name, rots, d))
    return theta, arcs


def build_rows(theta, arcs, center_mm, R_base=None):
    """展开成 CSV 行。R_base 为 None 时输出工具系坐标（frame=tool）。"""
    p_tcp = np.asarray(center_mm, dtype=float)
    rows = []
    for seg_idx, (name, rots, d) in enumerate(arcs):
        for ang, R_jog, dvec in zip(theta, rots, d):
            dvec = np.asarray(dvec, dtype=float)
            if R_base is None:
                frame = "tool"
                p_mm = dvec
                R_flange = R_jog
                p_tcp_out = (0.0, 0.0, 0.0)
            else:
                frame = "base"
                R_flange = R_base @ R_jog
                p_tcp_out = tuple(p_tcp / 1000.0)
                p_mm = p_tcp + R_base @ dvec
            rx, ry, rz = R_to_rpy(R_flange)
            qx, qy, qz, qw = R_to_quat(R_flange)
            rows.append(
                {
                    "segment": name,
                    "seg_index": seg_idx,
                    "theta_deg": math.degrees(ang),
                    "frame": frame,
                    "tcp_x_m": p_tcp_out[0],
                    "tcp_y_m": p_tcp_out[1],
                    "tcp_z_m": p_tcp_out[2],
                    "d_x_mm": dvec[0],
                    "d_y_mm": dvec[1],
                    "d_z_mm": dvec[2],
                    "x_m": p_mm[0] / 1000.0,
                    "y_m": p_mm[1] / 1000.0,
                    "z_m": p_mm[2] / 1000.0,
                    "x_mm": p_mm[0],
                    "y_mm": p_mm[1],
                    "z_mm": p_mm[2],
                    "rx_rad": rx,
                    "ry_rad": ry,
                    "rz_rad": rz,
                    "rx_deg": math.degrees(rx),
                    "ry_deg": math.degrees(ry),
                    "rz_deg": math.degrees(rz),
                    "qx": qx,
                    "qy": qy,
                    "qz": qz,
                    "qw": qw,
                }
            )
    return rows


# ---------------------------------------------------------------- 输出


def write_csv(rows, path: Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_ply(rows, path: Path, radius_mm: float) -> None:
    pts = np.array([[r["x_mm"], r["y_mm"], r["z_mm"]] for r in rows], dtype=float)
    segs = np.array([r["seg_index"] for r in rows], dtype=int)
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property int intensity\nend_header\n")
        for p, s in zip(pts, segs):
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {s}\n")


def write_preview(rows, path: Path, radius_mm: float, frame: str) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - 环境缺 matplotlib
        print(f"[warn] 跳过预览图：{exc}")
        return False

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    r = float(radius_mm)

    # 参考：球面线框 + 半球面（法兰所在 -z 侧）
    u = np.linspace(0, 2 * np.pi, 60)
    v = np.linspace(0, np.pi, 30)
    xs = r * np.outer(np.cos(u), np.sin(v))
    ys = r * np.outer(np.sin(u), np.sin(v))
    zs = r * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, color="0.75", linewidth=0.5, alpha=0.5)
    hx = r * np.outer(np.cos(u), np.sin(v[:16]))
    hy = r * np.outer(np.sin(u), np.sin(v[:16]))
    hz = r * np.outer(np.ones_like(u), np.cos(v[:16]))
    ax.plot_surface(hx, hy, hz, color="tab:blue", alpha=0.10, linewidth=0)

    colors = {"arc_x": "tab:red", "arc_y": "tab:green"}
    for name in ("arc_x", "arc_y"):
        pts = np.array([[q["x_mm"], q["y_mm"], q["z_mm"]] for q in rows if q["segment"] == name])
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=colors[name], lw=2.2, label=name)

    ax.scatter([0], [0], [0], color="black", s=30, label="TCP")
    ax.set_xlabel("x / mm")
    ax.set_ylabel("y / mm")
    ax.set_zlabel("z / mm")
    ax.set_title(
        f"Flange jog trajectory  r={r:.1f} mm  frame={frame}\n" "two semicircles on one sphere"
    )
    ax.legend(loc="upper right")
    lim = r * 1.25
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return True


# ---------------------------------------------------------------- 位姿来源


def pose_from_live(timeout_s: float = 5.0):
    """从 /tcp_pose 读取一帧 (position_m, rpy_rad)；失败返回 None。"""
    try:
        import rclpy
        from rclpy.node import Node
        from tl_ros2_interface.msg import CartesianPose
    except Exception as exc:
        print(f"[warn] 无法导入 ROS 消息（{exc}），--from-live 不可用")
        return None

    rclpy.init(args=None)
    node = Node("flange_jog_pose_probe")
    holder = {}

    def _cb(msg):
        holder["msg"] = msg

    node.create_subscription(CartesianPose, "/tcp_pose", _cb, 10)
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    try:
        while "msg" not in holder:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    msg = holder.get("msg")
    if msg is None:
        print("[warn] 超时未收到 /tcp_pose")
        return None
    p = (msg.position.x, msg.position.y, msg.position.z)
    rpy = (msg.rpy.x, msg.rpy.y, msg.rpy.z)
    return p, rpy


# ---------------------------------------------------------------- 实机执行


def _vec7(value: float, dof: int) -> list[float]:
    """ServoJ 约束向量必须固定 7 元素；6 轴第 7 位补 0。"""
    return [float(value)] * dof + [0.0] * (7 - dof)


class ArmRunner:
    """按法兰位姿轨迹执行：IK 求解 + ServoJ 流式下发。

    前提：工具手切换到 0（tool_num=0）。此时 /tcp_pose 与 IK 的位姿都指法兰本身；
    若仍挂着 10 mm 工具手，脚本算出的法兰位姿会被当成 TCP 位姿，整体偏 10 mm。
    """

    def __init__(self, args):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from std_srvs.srv import Trigger
        from tl_ros2_interface.action import Move
        from tl_ros2_interface.msg import CartesianPose, ErrorInfo
        from tl_ros2_interface.srv import Fk, GetCoordNum, GetRobotJointParam, Ik, OpenServoJ

        rclpy.init(args=None)
        self.args = args
        self._rclpy = rclpy
        self.node = rclpy.create_node("flange_jog_trajectory")
        self.logger = self.node.get_logger()
        self._cb = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        self._tcp = None
        self._joints = None
        self._errors: list[str] = []
        self._stop = False
        self._recording = False
        self._tcp_rec: list[tuple] = []
        self._joint_rec: list[tuple] = []
        # DLS 用本地 FK 做数值雅可比（要求本地模型与当前工具一致）
        self._dls_source = 1

        self.node.create_subscription(
            CartesianPose, "/tcp_pose", self._on_tcp, 20, callback_group=self._cb
        )
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joints, 20, callback_group=self._cb
        )
        self.node.create_subscription(
            ErrorInfo, "/tl_driver/error_info", self._on_error, 100, callback_group=self._cb
        )
        self.servoj_pub = self.node.create_publisher(
            Float64MultiArray, "/tl_driver/set_servoj_pos", 10
        )

        self._Ik, self._Fk = Ik, Fk
        self._OpenServoJ, self._Trigger = OpenServoJ, Trigger
        self._Move = Move
        self._GetCoordNum, self._GetRobotJointParam = GetCoordNum, GetRobotJointParam

        self.ik_cli = self.node.create_client(Ik, "/tl_driver/ik", callback_group=self._cb)
        self.fk_cli = self.node.create_client(Fk, "/tl_driver/fk", callback_group=self._cb)
        self.open_cli = self.node.create_client(
            OpenServoJ, "/tl_driver/open_servoj", callback_group=self._cb
        )
        self.close_cli = self.node.create_client(
            Trigger, "/tl_driver/close_servoj", callback_group=self._cb
        )
        self.coord_cli = self.node.create_client(
            GetCoordNum, "/tl_driver/get_coord_num", callback_group=self._cb
        )
        self.joint_param_cli = self.node.create_client(
            GetRobotJointParam, "/tl_driver/get_robot_joint_param", callback_group=self._cb
        )
        self.move_cli = ActionClient(
            self.node, Move, "/tl_driver/move_joint", callback_group=self._cb
        )

        self._old_handlers = {}
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                self._old_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, self._on_signal)
            except (ValueError, OSError):
                self._old_handlers.pop(signum, None)

    # -------------------------------------------------------------- 回调

    def _on_tcp(self, msg) -> None:
        self._tcp = msg
        if self._recording:
            self._tcp_rec.append(
                (
                    time.time(),
                    msg.position.x,
                    msg.position.y,
                    msg.position.z,
                    msg.rpy.x,
                    msg.rpy.y,
                    msg.rpy.z,
                )
            )

    def _on_joints(self, msg) -> None:
        self._joints = msg
        if self._recording:
            self._joint_rec.append((time.time(), *[float(v) for v in msg.position]))

    def _on_error(self, msg) -> None:
        # 回调异常会杀掉执行器线程，这里必须保证不抛
        try:
            self._errors.append(f"[{msg.message_type}/{msg.message_code}] {msg.message}")
        except Exception:
            self._errors.append("收到无法解析的 error_info")

    def _on_signal(self, signum, frame) -> None:
        self._stop = True

    # -------------------------------------------------------------- 基础调用

    def _call(self, client, request, name, timeout=None, ignore_stop=False):
        timeout = self.args.timeout if timeout is None else timeout
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout
        while not done.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(f"{name} 调用超时（{timeout:.1f}s）")
            if self._stop and not ignore_stop:
                raise RuntimeError(f"{name} 被中断")
            done.wait(min(0.05, remaining))
        return future.result()

    def _wait(self, future, name, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError(f"{name} 超时（{timeout:.1f}s）")
        return future.result()

    def wait_services(self, timeout: float = 15.0) -> None:
        checks = [
            ("/tl_driver/ik", self.ik_cli),
            ("/tl_driver/fk", self.fk_cli),
            ("/tl_driver/open_servoj", self.open_cli),
            ("/tl_driver/close_servoj", self.close_cli),
            ("/tl_driver/get_coord_num", self.coord_cli),
        ]
        if self.args.check_limits:
            checks.append(("/tl_driver/get_robot_joint_param", self.joint_param_cli))
        deadline = time.monotonic() + timeout
        for name, client in checks:
            remaining = max(0.1, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                raise RuntimeError(f"服务 {name} 不可用")

    def wait_state(self, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while self._joints is None or self._tcp is None:
            if time.monotonic() > deadline:
                raise RuntimeError("等待 /joint_states 与 /tcp_pose 超时")
            time.sleep(0.05)
        joints = np.asarray(self._joints.position, dtype=float)
        pose = (
            np.array([self._tcp.position.x, self._tcp.position.y, self._tcp.position.z]),
            np.array([self._tcp.rpy.x, self._tcp.rpy.y, self._tcp.rpy.z]),
        )
        return joints, pose

    def check_tool(self):
        response = self._call(self.coord_cli, self._GetCoordNum.Request(), "get_coord_num")
        if response is None or not response.success:
            raise RuntimeError(
                "读取当前坐标系失败: " + (response.message if response else "无响应")
            )
        if response.tool_num != 0:
            message = f"当前工具坐标系编号 {response.tool_num} ≠ 0，IK 位姿会被当成 TCP 位姿"
            if not self.args.allow_tool:
                raise RuntimeError(message + "；无工具复现请先切到工具 0，或加 --allow-tool")
            self.logger.warn(message + "（--allow-tool 继续）")
        return int(response.tool_num), int(response.user_num)

    # -------------------------------------------------------------- 求解

    def solve(self, positions, rpys, reference, dof: int) -> np.ndarray:
        joints = np.full((len(positions), dof), np.nan)
        ref = np.asarray(reference, dtype=float)
        for index, (pos, rpy) in enumerate(zip(positions, rpys)):
            if self._stop:
                raise RuntimeError("收到停止信号")
            request = self._Ik.Request()
            request.frame = 1
            request.pose.header.frame_id = "base_link"
            request.pose.position.x = float(pos[0])
            request.pose.position.y = float(pos[1])
            request.pose.position.z = float(pos[2])
            request.pose.rpy.x = float(rpy[0])
            request.pose.rpy.y = float(rpy[1])
            request.pose.rpy.z = float(rpy[2])
            request.pose.arm_angle = 0.0
            request.form = int(self.args.form)
            request.reference_pos = ref.tolist()
            request.source = int(self.args.ik_source)
            response = self._call(self.ik_cli, request, f"ik[{index}]")
            if response is None or not response.success:
                raise RuntimeError(
                    f"第 {index} 点逆解失败: " + (response.message if response else "无响应")
                )
            solution = np.asarray(response.joints[:dof], dtype=float)
            if not np.all(np.isfinite(solution)):
                raise RuntimeError(f"第 {index} 点逆解返回非有限值")
            joints[index] = solution
            ref = solution
            if (index + 1) % 50 == 0:
                print(f"\r逆解 {index + 1}/{len(positions)}", end="", flush=True)
        print()
        return joints

    # -------------------------------------------------------------- 数值微分 IK

    @staticmethod
    def _log_rot(rotation: np.ndarray) -> np.ndarray:
        """SO(3) 对数映射：旋转矩阵 → 基座系旋转向量。"""
        cos_angle = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
        angle = math.acos(cos_angle)
        if angle < 1e-9:
            return np.zeros(3)
        return (
            angle
            / (2.0 * math.sin(angle))
            * np.array(
                [
                    rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1],
                ]
            )
        )

    def _fk_service(self, joints, source: int):
        request = self._Fk.Request()
        request.frame = 1
        request.joints = [float(v) for v in joints]
        request.source = source
        response = self._call(self.fk_cli, request, "fk(dls)")
        if response is None or not response.success:
            raise RuntimeError("FK 失败: " + (response.message if response else "无响应"))
        position = np.array(
            [response.pose.position.x, response.pose.position.y, response.pose.position.z]
        )
        rotation = rpy_to_R(response.pose.rpy.x, response.pose.rpy.y, response.pose.rpy.z)
        return position, rotation

    def _fk_local(self, joints):
        return self._fk_service(joints, self._dls_source)

    def _calibrate_local_model(self, joints) -> None:
        """本地模型在驱动启动时固化当时的工具手；工具改过后会差一个常值平移。

        用控制器 FK（source=0，跟随当前工具）与本地 FK 在同一组关节下比对，
        差值转到工具系即得补偿量；姿态必须一致，否则不能只做平移补偿。
        """
        p_ctrl, r_ctrl = self._fk_service(joints, 0)
        p_loc, r_loc = self._fk_service(joints, 1)
        rot_gap = math.degrees(np.linalg.norm(self._log_rot(r_ctrl @ r_loc.T)))
        if rot_gap > 0.5:
            raise RuntimeError(f"本地模型与控制器姿态差 {rot_gap:.2f}°，无法用平移补偿")
        offset = r_ctrl.T @ (p_loc - p_ctrl)
        if np.linalg.norm(offset) > 1e-6:
            # 偏移不是常值工具平移时无法可靠补偿（实测 DLS 会跟错分支），直接拒绝
            raise RuntimeError(
                f"本地 FK 固化了 {np.linalg.norm(offset) * 1000:.2f} mm 工具偏置"
                "（驱动启动时的工具手），与当前工具不一致；"
                "请重启 tl_driver 让本地模型按当前工具重建，"
                "或改用 --record-joints / --replay-joints"
            )

    def _jacobian(self, joints):
        """数值雅可比：6×dof，列 = 各关节单位增量对 [位置; 旋转向量] 的贡献。"""
        position, rotation = self._fk_local(joints)
        jacobian = np.zeros((6, len(joints)))
        eps = self.args.dls_eps
        for axis in range(len(joints)):
            perturbed = np.asarray(joints, dtype=float).copy()
            perturbed[axis] += eps
            p2, r2 = self._fk_local(perturbed)
            jacobian[:3, axis] = (p2 - position) / eps
            jacobian[3:, axis] = self._log_rot(r2 @ rotation.T) / eps
        return position, rotation, jacobian

    def _dls_leg(self, targets, q0):
        """从 q0 沿 targets 连续跟踪，返回逐点关节角（含起点）。"""
        damping = self.args.dls_lambda**2
        q = np.asarray(q0, dtype=float).copy()
        out = [q.copy()]
        for target_pos, target_rot in targets:
            # 每轮都重算雅可比：手腕奇异区（q5 过 ±90°）附近 J 变化剧烈，
            # 复用旧 J 会发散，实测 λ=0.005 + 20 轮可把残差压到 1e-3 mm。
            for _ in range(self.args.dls_iters):
                p0, r0, jacobian = self._jacobian(q)
                error = np.concatenate([target_pos - p0, self._log_rot(target_rot @ r0.T)])
                if np.linalg.norm(error[:3]) < 1e-7 and np.linalg.norm(error[3:]) < 1e-6:
                    break
                delta = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + damping * np.eye(6), error
                )
                peak = float(np.max(np.abs(delta)))
                if peak > self.args.dls_max_step:
                    delta *= self.args.dls_max_step / peak
                q = q + delta
            out.append(q.copy())
        return np.array(out)

    def _dls_arc(self, positions, rpys, thetas, q0):
        """以 θ=0 为两支共同起点分别跟踪，再按原顺序拼回，保证 -90°→+90° 连续。"""
        thetas = np.asarray(thetas, dtype=float)
        zero = int(np.argmin(np.abs(thetas)))
        neg = sorted(
            [i for i in range(len(thetas)) if thetas[i] <= thetas[zero]],
            key=lambda i: -thetas[i],
        )
        pos = sorted(
            [i for i in range(len(thetas)) if thetas[i] >= thetas[zero]],
            key=lambda i: thetas[i],
        )

        def targets(indices):
            return [(np.asarray(positions[i], dtype=float), rpy_to_R(*rpys[i])) for i in indices]

        leg_neg = self._dls_leg(targets(neg), q0)
        leg_pos = self._dls_leg(targets(pos), q0)

        # _dls_leg 返回 [起点] + 各点解，因此按目标顺序取 k+1
        joints = np.zeros((len(thetas), len(q0)), dtype=float)
        for k, index in enumerate(neg):
            joints[index] = leg_neg[k + 1]
        for k, index in enumerate(pos):
            joints[index] = leg_pos[k + 1]
        return joints

    def fk_check(self, joints, positions, rpys):
        residual = np.full(len(positions), np.nan)
        rot_err = np.full(len(positions), np.nan)
        for index, (q, pos, rpy) in enumerate(zip(joints, positions, rpys)):
            request = self._Fk.Request()
            request.frame = 1
            request.joints = [float(v) for v in q]
            request.source = 0
            response = self._call(self.fk_cli, request, f"fk[{index}]")
            if response is None or not response.success:
                continue
            fk_pos = np.array(
                [response.pose.position.x, response.pose.position.y, response.pose.position.z]
            )
            residual[index] = float(np.linalg.norm(fk_pos - np.asarray(pos))) * 1000.0
            R_fk = rpy_to_R(response.pose.rpy.x, response.pose.rpy.y, response.pose.rpy.z)
            R_target = rpy_to_R(*rpy)
            cos_angle = (np.trace(R_fk.T @ R_target) - 1.0) / 2.0
            rot_err[index] = math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))
        return residual, rot_err

    def joint_limits(self, dof: int):
        low = np.full(dof, np.nan)
        high = np.full(dof, np.nan)
        for axis in range(1, dof + 1):
            request = self._GetRobotJointParam.Request()
            request.id = axis
            try:
                response = self._call(self.joint_param_cli, request, f"joint_param[{axis}]")
            except Exception as exc:
                self.logger.warn(f"读取 J{axis} 限位失败（{exc}），跳过限位检查")
                return None, None
            if response is None or not response.success:
                self.logger.warn(f"读取 J{axis} 限位失败，跳过限位检查")
                return None, None
            low[axis - 1] = response.param.neg_sw_limit
            high[axis - 1] = response.param.pos_sw_limit
        return low, high

    # -------------------------------------------------------------- 运动

    def move_joint(self, joints, dof: int) -> None:
        if not self.move_cli.wait_for_server(timeout_sec=PREPOSE_GOAL_TIMEOUT_S):
            raise RuntimeError("/tl_driver/move_joint action 不可用")
        goal = self._Move.Goal()
        values = [0.0] * 14
        for axis in range(min(7, dof)):
            values[axis] = float(joints[axis])
        command = goal.command
        command.target_pos_value = values
        command.coord = 0
        command.velocity = float(PREPOSE_VELOCITY)
        command.velocity_sync = 0.0
        command.acc = float(PREPOSE_ACC)
        command.dec = float(PREPOSE_ACC)
        command.pl = 0
        command.tool_num = 0
        command.user_num = 0
        command.para_sync = False
        goal.timeout_sec = float(self.args.prepose_timeout)
        handle = self._wait(
            self.move_cli.send_goal_async(goal), "move_joint goal", PREPOSE_GOAL_TIMEOUT_S
        )
        if not handle.accepted:
            raise RuntimeError("move_joint goal 被拒绝")
        result = self._wait(
            handle.get_result_async(), "move_joint", self.args.prepose_timeout + PREPOSE_MARGIN_S
        )
        response = result.result
        if response is None or not response.success:
            raise RuntimeError("move_joint 失败: " + (response.message if response else "无响应"))

    def _open_servo(self, dof: int) -> None:
        request = self._OpenServoJ.Request()
        request.vmax = _vec7(math.radians(self.args.vmax_deg), dof)
        request.amax = _vec7(math.radians(self.args.amax_deg), dof)
        request.jmax = _vec7(math.radians(self.args.jmax_deg), dof)
        response = self._call(self.open_cli, request, "open_servoj")
        if response is None or not response.success:
            raise RuntimeError("open_servoj 失败: " + (response.message if response else "无响应"))

    def _close_servo(self) -> None:
        try:
            response = self._call(
                self.close_cli, self._Trigger.Request(), "close_servoj", ignore_stop=True
            )
            if response is None or not response.success:
                self.logger.error(
                    "close_servoj 失败: " + (response.message if response else "无响应")
                )
        except Exception as exc:
            self.logger.error(f"close_servoj 异常: {exc}")

    def _stream(self, joints: np.ndarray, duration: float) -> int:
        from std_msgs.msg import Float64MultiArray

        period = 1.0 / self.args.rate
        count = len(joints)
        start = time.monotonic()
        next_tick = start
        frames = 0
        while True:
            if self._stop:
                self.logger.warn("收到停止信号，提前结束")
                break
            if self._errors:
                raise RuntimeError(f"控制器报错，停止下发：{self._errors[-1]}")
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break
            x = elapsed / duration * (count - 1)
            i0 = int(x)
            i1 = min(i0 + 1, count - 1)
            frac = x - i0
            command = joints[i0] * (1.0 - frac) + joints[i1] * frac
            message = Float64MultiArray()
            message.data = [float(v) for v in command]
            self.servoj_pub.publish(message)
            frames += 1
            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep > 0.0:
                time.sleep(sleep)
            else:
                # 落后一个以上节拍：重新对齐，避免连续补发造成抖动
                next_tick = time.monotonic() + period
        message = Float64MultiArray()
        message.data = [float(v) for v in joints[-1]]
        self.servoj_pub.publish(message)
        return frames

    def _check_common_point(self, arc, current_joints) -> None:
        """DLS 以当前关节为起点，必须先确认它就在 θ=0 的公共起点上。"""
        _, positions, rpys, thetas, _ = arc
        zero = int(np.argmin(np.abs(np.asarray(thetas, dtype=float))))
        target_pos = np.asarray(positions[zero], dtype=float)
        target_rot = rpy_to_R(*rpys[zero])
        # 用控制器 FK（跟随当前工具），避免本地模型固化工具偏置干扰判断
        p_now, r_now = self._fk_service(current_joints, 0)
        gap = float(np.linalg.norm(p_now - target_pos)) * 1000.0
        rot_gap = math.degrees(np.linalg.norm(self._log_rot(target_rot @ r_now.T)))
        print(f"公共起点校验: 位置差 {gap:.2f} mm，姿态差 {rot_gap:.2f}°")
        if gap > 5.0 or rot_gap > 5.0:
            self.logger.warn(
                "当前关节不在 θ=0 公共起点，DLS 会先收敛到该点；建议先让法兰停在球面起点"
            )

    def run(self, arcs, current_joints, dof: int, out_dir: Path, execute: bool) -> dict:
        tool_num, user_num = self.check_tool()
        report = {"tool_num": tool_num, "user_num": user_num, "arcs": []}
        low, high = self.joint_limits(dof) if self.args.check_limits else (None, None)

        solved = []
        reference = np.asarray(current_joints, dtype=float)
        if self.args.ik_mode == "dls" and dof != 6:
            self.logger.warn("DLS 只支持 6 轴，自动退回 service 模式")
        if self.args.ik_mode == "dls" and dof == 6:
            self._calibrate_local_model(current_joints)
            self._check_common_point(arcs[0], current_joints)
        for name, positions, rpys, thetas, duration in arcs:
            print(f"[{name}] 逆解 {len(positions)} 点 ...")
            if self.args.ik_mode == "dls" and dof == 6:
                joints = self._dls_arc(positions, rpys, thetas, current_joints)
            else:
                joints = self.solve(positions, rpys, reference, dof)
                reference = joints[-1].copy()
            step = float(np.max(np.abs(np.diff(joints, axis=0)))) if len(joints) > 1 else 0.0
            residual, rot_err = self.fk_check(joints, positions, rpys)
            entry = {
                "segment": name,
                "samples": int(len(joints)),
                "max_joint_step_rad": step,
                # 全 NaN 表示 FK 全部失败，必须当作失败而不是 nan 比较的 False
                "max_residual_mm": (
                    float(np.nanmax(residual)) if np.any(np.isfinite(residual)) else float("nan")
                ),
                "max_rot_err_deg": (
                    float(np.nanmax(rot_err)) if np.any(np.isfinite(rot_err)) else float("nan")
                ),
                "duration_s": float(duration),
            }
            if low is not None:
                entry["limits_ok"] = bool(
                    np.all(joints >= low - 1e-6) and np.all(joints <= high + 1e-6)
                )
            report["arcs"].append(entry)
            print(
                f"[{name}] 关节最大步长 {step:.4f} rad，FK 位置残差 "
                f"{entry['max_residual_mm']:.2f} mm，姿态误差 {entry['max_rot_err_deg']:.3f}°"
            )
            solved.append((name, joints))

        failures = []
        for entry in report["arcs"]:
            if entry["max_joint_step_rad"] > self.args.max_joint_step and not self.args.allow_jumps:
                failures.append(
                    f"{entry['segment']} 关节跳变 {entry['max_joint_step_rad']:.3f} rad 超限"
                )
            if not np.isfinite(entry["max_residual_mm"]) or not np.isfinite(
                entry["max_rot_err_deg"]
            ):
                failures.append(f"{entry['segment']} FK 校验无有效数据")
            elif entry["max_residual_mm"] > self.args.max_residual_mm:
                failures.append(
                    f"{entry['segment']} FK 位置残差 {entry['max_residual_mm']:.2f} mm 超限"
                )
            elif entry["max_rot_err_deg"] > self.args.max_rot_err_deg:
                failures.append(
                    f"{entry['segment']} FK 姿态误差 {entry['max_rot_err_deg']:.2f}° 超限"
                )
            if entry.get("limits_ok") is False:
                failures.append(f"{entry['segment']} 逆解超出关节软限位")
        report["failures"] = failures
        if failures:
            for message in failures:
                self.logger.error(message)
            raise RuntimeError("轨迹检查失败，拒绝执行：" + "；".join(failures))

        if not execute:
            print("检查通过（--solve-only，未运动）")
            return report

        if not self.args.no_prepose:
            self.logger.warn("预定位：MoveJ 到第一条弧起点，请确认路径无障碍")
            self.move_joint(solved[0][1][0], dof)
        else:
            gap = float(
                np.max(
                    np.abs(np.asarray(current_joints, dtype=float) - np.asarray(solved[0][1][0]))
                )
            )
            if gap > self.args.max_joint_step:
                raise RuntimeError(
                    f"--no-prepose 但当前关节距首点 {gap:.3f} rad，ServoJ 会突跳；请去掉 --no-prepose"
                )

        self._recording = True
        try:
            for index, (name, joints) in enumerate(solved):
                if index > 0 and not self.args.no_prepose:
                    self.logger.warn(f"MoveJ 到 {name} 起点")
                    self.move_joint(joints[0], dof)
                duration = arcs[index][3]
                opened = False
                try:
                    self._open_servo(dof)
                    opened = True
                    print(f"[{name}] ServoJ {duration:.2f}s @ {self.args.rate:.0f} Hz ...")
                    frames = self._stream(joints, duration)
                    print(f"[{name}] 下发 {frames} 帧，保持 {self.args.hold_time:.2f}s")
                    time.sleep(self.args.hold_time)
                finally:
                    if opened:
                        self._close_servo()
            time.sleep(SERVOJ_SETTLE_S)
        finally:
            self._recording = False
            self.write_recordings(out_dir)
        return report

    def write_recordings(self, out_dir: Path) -> None:
        if self._tcp_rec:
            with open(out_dir / "actual_tcp.csv", "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["t_s", "x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad"])
                writer.writerows(self._tcp_rec)
        if self._joint_rec:
            header = ["t_s"] + [f"q{index + 1}" for index in range(len(self._joint_rec[0]) - 1)]
            with open(out_dir / "actual_joints.csv", "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(self._joint_rec)
        print(f"实采数据: {out_dir}/actual_tcp.csv, actual_joints.csv")

    # -------------------------------------------------------------- 录 / 放关节

    def record_joints(self, path: Path) -> None:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        print("开始记录 /joint_states：现在用点动走要复现的轨迹，Ctrl-C 结束")
        self._recording = True
        try:
            last = time.monotonic()
            while not self._stop:
                time.sleep(0.05)
                now = time.monotonic()
                if now - last >= 1.0:
                    print(f"\r已记录 {len(self._joint_rec)} 帧", end="", flush=True)
                    last = now
        finally:
            self._recording = False
        print()
        if len(self._joint_rec) < 2:
            raise RuntimeError("没有录到足够的 /joint_states")
        header = ["t_s"] + [f"q{index + 1}" for index in range(len(self._joint_rec[0]) - 1)]
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self._joint_rec)
        duration = self._joint_rec[-1][0] - self._joint_rec[0][0]
        print(f"已保存 {len(self._joint_rec)} 帧 / {duration:.2f}s → {path}")

    def replay_joints(self, path: Path) -> None:
        path = Path(path).expanduser()
        with open(path, encoding="utf-8") as f:
            rows = list(csv.reader(f))
        if len(rows) < 3:
            raise RuntimeError(f"{path} 数据不足")
        data = np.asarray(rows[1:], dtype=float)
        times = data[:, 0] - data[0, 0]
        joints = data[:, 1:]
        current, _ = self.wait_state()
        dof = min(joints.shape[1], len(current))
        if dof < 6:
            raise RuntimeError(f"录制数据只有 {dof} 轴，无法回放")
        duration = float(times[-1])
        count = max(int(duration * self.args.rate) + 1, 2)
        grid = np.linspace(0.0, duration, count)
        resampled = np.column_stack([np.interp(grid, times, joints[:, i]) for i in range(dof)])
        print(
            f"回放 {path.name}: {len(rows) - 1} 帧 / {duration:.2f}s，"
            f"重采样 {count} 点 @ {self.args.rate:.0f} Hz"
        )
        gap = float(np.max(np.abs(resampled[0] - current[:dof])))
        if gap > 0.05:
            if self.args.no_prepose:
                raise RuntimeError(
                    f"录制首点与当前关节差 {gap:.3f} rad，--no-prepose 会突跳；请去掉该参数"
                )
            self.logger.warn(f"预定位：MoveJ 到录制首点（关节差 {gap:.3f} rad）")
            self.move_joint(resampled[0], dof)
        self._recording = True
        try:
            opened = False
            try:
                self._open_servo(dof)
                opened = True
                frames = self._stream(resampled, duration)
                print(f"下发 {frames} 帧，保持 {self.args.hold_time:.2f}s")
                time.sleep(self.args.hold_time)
            finally:
                if opened:
                    self._close_servo()
            time.sleep(SERVOJ_SETTLE_S)
        finally:
            self._recording = False
            self.write_recordings(path.parent)

    def close(self) -> None:
        self._stop = True
        for signum, handler in self._old_handlers.items():
            try:
                signal.signal(signum, handler)
            except (ValueError, OSError):
                pass
        self._old_handlers.clear()
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self._spin_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self._rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- main


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="生成工具手偏置下法兰绕 TCP 点动 X/Y 旋转的轨迹（两个半圆）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--center",
        nargs=3,
        type=float,
        default=DEFAULT_CENTER_MM,
        metavar=("X", "Y", "Z"),
        help="TCP 球心位置，单位 mm",
    )
    parser.add_argument(
        "--radius", type=float, default=DEFAULT_RADIUS_MM, help="工具偏置 r，单位 mm"
    )
    parser.add_argument(
        "--sweep-deg",
        type=float,
        default=180.0,
        help="每段弧的总扫掠角（180° = 半圆，90° = 1/4 圆）",
    )
    parser.add_argument("--samples", type=int, default=181, help="每段弧采样点数")
    parser.add_argument(
        "--rpy-deg", nargs=3, type=float, default=None, help="该位姿 TCP 姿态（度，XYZ 外旋）"
    )
    parser.add_argument("--from-live", action="store_true", help="从 /tcp_pose 读取当前 TCP 位姿")
    parser.add_argument("--out", type=Path, default=None, help="输出目录")
    parser.add_argument("--no-plot", action="store_true", help="不生成 preview.png")
    parser.add_argument(
        "--solve-only", action="store_true", help="连接实机做 IK/FK 校验与可达性检查，不运动"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="求解并实际运动（工具必须切到 0；无工具时法兰位姿即 TCP 位姿）",
    )
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPS, help="沿弧线速度 m/s")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="ServoJ 下发频率 Hz")
    parser.add_argument(
        "--ik-mode",
        choices=("dls", "service"),
        default="dls",
        help="dls=数值微分 IK 连续跟踪（默认，过奇异/分支边界稳）；service=调用 IK 服务",
    )
    parser.add_argument(
        "--ik-source",
        type=int,
        choices=(0, 1),
        default=0,
        help="service 模式：0=控制器，1=本地模型",
    )
    parser.add_argument(
        "--form", type=int, default=0, help="service 模式逆解形态（0=自动，1~8 指定分支）"
    )
    parser.add_argument("--dls-lambda", type=float, default=0.005, help="DLS 阻尼系数")
    parser.add_argument("--dls-eps", type=float, default=1e-5, help="数值雅可比扰动 rad")
    parser.add_argument("--dls-iters", type=int, default=20, help="单点 DLS 迭代次数")
    parser.add_argument("--dls-max-step", type=float, default=0.2, help="单次 DLS 关节增量上限 rad")
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=DEFAULT_MAX_JOINT_STEP,
        help="相邻采样点关节最大变化 rad，超限视为分支跳变",
    )
    parser.add_argument(
        "--max-residual-mm",
        type=float,
        default=DEFAULT_MAX_RESIDUAL_MM,
        help="IK 回代 FK 位置残差上限",
    )
    parser.add_argument(
        "--max-rot-err-deg",
        type=float,
        default=DEFAULT_MAX_ROT_ERR_DEG,
        help="IK 回代 FK 姿态误差上限",
    )
    parser.add_argument("--allow-jumps", action="store_true", help="允许关节跳变（不推荐）")
    parser.add_argument(
        "--allow-tool", action="store_true", help="允许 tool_num≠0 继续（位姿会整体偏 r）"
    )
    parser.add_argument(
        "--check-limits",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="检查关节软限位",
    )
    parser.add_argument("--no-prepose", action="store_true", help="不做 MoveJ 预定位到弧起点")
    parser.add_argument("--prepose-timeout", type=float, default=60.0, help="MoveJ 超时 s")
    parser.add_argument("--hold-time", type=float, default=0.5, help="每段弧结束后的保持时间 s")
    parser.add_argument("--vmax-deg", type=float, default=200.0, help="ServoJ 最大关节速度 °/s")
    parser.add_argument("--amax-deg", type=float, default=800.0, help="ServoJ 最大关节加速度 °/s²")
    parser.add_argument(
        "--jmax-deg", type=float, default=4800.0, help="ServoJ 最大关节加加速度 °/s³"
    )
    parser.add_argument("--timeout", type=float, default=10.0, help="单次服务调用超时 s")
    parser.add_argument(
        "--record-joints",
        type=Path,
        default=None,
        help="记录 /joint_states 到 CSV（Ctrl-C 结束）；带工具点动时录，切工具 0 后回放即可复现",
    )
    parser.add_argument(
        "--replay-joints",
        type=Path,
        default=None,
        help="回放 --record-joints 录下的关节序列（ServoJ）",
    )
    args = parser.parse_args(argv)

    if args.samples < 3:
        parser.error("--samples 至少为 3")
    if args.radius <= 0.0:
        parser.error("--radius 必须为正")
    if args.solve_only and args.execute:
        parser.error("--solve-only 与 --execute 只能二选一")
    if (args.solve_only or args.execute) and args.rpy_deg is None and not args.from_live:
        parser.error("--solve-only/--execute 需要 --from-live 或 --rpy-deg 提供基座姿态")
    if args.rate <= 0.0:
        parser.error("--rate 必须为正")
    if args.speed <= 0.0:
        parser.error("--speed 必须为正")

    # 录/放关节：不依赖 IK 与工具偏置，带工具点动录、切工具 0 回放即可复现
    if args.record_joints is not None or args.replay_joints is not None:
        if args.execute or args.solve_only:
            parser.error("--record-joints / --replay-joints 不能与 --execute / --solve-only 同用")
        runner = ArmRunner(args)
        try:
            runner.wait_services()
            if args.record_joints is not None:
                runner.record_joints(args.record_joints)
            if args.replay_joints is not None:
                runner.replay_joints(args.replay_joints)
        finally:
            runner.close()
        return 0

    center = np.asarray(args.center, dtype=float)

    runner = None
    current_joints = None
    R_base = None
    frame = "tool"
    if args.rpy_deg is not None and args.from_live:
        parser.error("--rpy-deg 与 --from-live 只能二选一")

    if args.solve_only or args.execute:
        # 执行链路自己持有节点，顺便读当前位姿，避免 rclpy 反复 init/shutdown
        runner = ArmRunner(args)
        try:
            runner.wait_services()
            current_joints, (live_pos, live_rpy) = runner.wait_state()
            if args.from_live:
                print(
                    "live TCP: pos=(%.4f, %.4f, %.4f) m rpy=(%.4f, %.4f, %.4f) rad"
                    % (*live_pos, *live_rpy)
                )
                R_base = rpy_to_R(*live_rpy)
                frame = "base"
                distance = float(np.linalg.norm(live_pos * 1000.0 - center))
                if abs(distance - args.radius) > 1.0:
                    print(
                        f"[warn] 当前 TCP 距球心 {distance:.2f} mm，不等于 r={args.radius:.2f} mm；"
                        "无工具复现时应先把工具切到 0，并让法兰停在球面上"
                    )
            else:
                R_base = rpy_to_R(*(math.radians(a) for a in args.rpy_deg))
                frame = "base"
        except Exception:
            runner.close()
            raise
    elif args.from_live:
        live = pose_from_live()
        if live is None:
            return 1
        pos, rpy = live
        print("live TCP: pos=(%.4f, %.4f, %.4f) m rpy=(%.4f, %.4f, %.4f) rad" % (*pos, *rpy))
        R_base = rpy_to_R(*rpy)
        frame = "base"
        distance = float(np.linalg.norm(np.asarray(pos) * 1000.0 - center))
        if distance > 1.0 and abs(distance - args.radius) > 1.0:
            print(
                f"[warn] 当前 TCP 距球心 {distance:.2f} mm：既不在球心（工具开）也不在球面"
                f"（工具关，r={args.radius:.2f} mm）上，请确认 --center"
            )
    elif args.rpy_deg is not None:
        R_base = rpy_to_R(*(math.radians(a) for a in args.rpy_deg))
        frame = "base"

    theta, arcs = build_arcs(args.radius, args.sweep_deg, args.samples)
    rows = build_rows(theta, arcs, center, R_base)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out_dir = args.out if args.out is not None else DEFAULT_OUT_DIR / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    write_csv(rows, out_dir / "flange_path.csv")
    write_ply(rows, out_dir / "flange_path.ply", args.radius)
    plotted = (
        False if args.no_plot else write_preview(rows, out_dir / "preview.png", args.radius, frame)
    )

    arc_len = math.pi * args.radius * (args.sweep_deg / 180.0)
    summary = {
        "center_mm": list(map(float, args.center)),
        "radius_mm": float(args.radius),
        "sweep_deg": float(args.sweep_deg),
        "samples_per_arc": int(args.samples),
        "frame": frame,
        "arc_length_mm": arc_len,
        "total_length_mm": 2.0 * arc_len,
        "R_base": None if R_base is None else R_base.tolist(),
        "segments": {
            "arc_x": "y-z 平面半圆，绕工具 X",
            "arc_y": "x-z 平面半圆，绕工具 Y",
        },
    }
    with open(out_dir / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    first_x = next(
        r
        for r in rows
        if r["segment"] == "arc_x" and abs(r["theta_deg"] + args.sweep_deg / 2) < 1e-9
    )
    last_x = next(
        r
        for r in rows
        if r["segment"] == "arc_x" and abs(r["theta_deg"] - args.sweep_deg / 2) < 1e-9
    )
    first_y = next(
        r
        for r in rows
        if r["segment"] == "arc_y" and abs(r["theta_deg"] + args.sweep_deg / 2) < 1e-9
    )
    last_y = next(
        r
        for r in rows
        if r["segment"] == "arc_y" and abs(r["theta_deg"] - args.sweep_deg / 2) < 1e-9
    )

    def fmt(row):
        return "(%.3f, %.3f, %.3f) mm" % (row["x_mm"], row["y_mm"], row["z_mm"])

    print(f"frame        : {frame}")
    print(
        f"center       : {list(map(float, args.center))} mm, r={args.radius} mm, sweep={args.sweep_deg}°"
    )
    print(f"arc_x        : {fmt(first_x)} -> {fmt(last_x)}  (len {arc_len:.2f} mm)")
    print(f"arc_y        : {fmt(first_y)} -> {fmt(last_y)}  (len {arc_len:.2f} mm)")
    mid = next(r for r in rows if r["segment"] == "arc_x" and abs(r["theta_deg"]) < 1e-9)
    print(f"common point : {fmt(mid)}  (theta=0, 两弧共用起点)")
    print(f"output       : {out_dir}")
    print(f"  csv={'yes'} ply={'yes'} preview={'yes' if plotted else 'no'}")

    if runner is not None:
        try:
            dof = int(len(current_joints))
            arc_len = math.pi * args.radius * (args.sweep_deg / 180.0)
            duration = (arc_len / 1000.0) / args.speed
            exec_arcs = []
            for seg_index, seg_name in ((0, "arc_x"), (1, "arc_y")):
                subset = [row for row in rows if row["seg_index"] == seg_index]
                positions = np.array([[r["x_m"], r["y_m"], r["z_m"]] for r in subset])
                rpys = np.array([[r["rx_rad"], r["ry_rad"], r["rz_rad"]] for r in subset])
                thetas = np.array([r["theta_deg"] for r in subset])
                exec_arcs.append((seg_name, positions, rpys, thetas, duration))
            summary["execution"] = runner.run(
                exec_arcs, current_joints, dof, out_dir, execute=args.execute
            )
            with open(out_dir / "summary.json", "w", encoding="utf-8") as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
        finally:
            runner.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
