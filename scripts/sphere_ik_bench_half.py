#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sphere_ik_bench_half.py — 上半球线框轨迹的控制器逆解性能评估 + 点云导出

用两个互相垂直的大圆（基座坐标系 xz 面与 yz 面，极轴固定为基座 z）的
**上半段**组成半球线框（z ≥ 球心 z），逐点调用 `/tl_driver/ik`（source=0，
控制器逆解），统计延迟、成功率、关节连续性、FK 残差与关节限位；可选通过
ServoJ 连续画出上半球，并采集实际 TCP 轨迹。末端姿态默认 `--orient fixed`
（全程等于当前 TCP 姿态），不做任何朝向球心的计算；`inward` / `radial` 是
显式开启的额外模式。

两段半圆之间用赤道 90° 弧（equator_bridge）连接，整条轨迹连续且全部落在
球面上。球心默认从灵巧空间体素数据（`scripts/workspace_measure/results/`
下 `dexterous_workspace.py --save-data` 生成的 `.npz`）自动选：只保留上半球
路径采样点都落在可达体积内部、首点距当前 TCP 不超过 `--auto-start-gap` 的
球心；按灵巧度排序后先用粗采样排序，再对候选做**全量细采样**逆解校验
（100% 成功、相邻解最大关节变化、关节软限位），通过即采用——粗采样会漏掉
1 rad 级换支，必须用全量路径确认；`--center` 显式指定球心，`--no-auto-center` 回退到
当前 TCP + R·x（轨迹首点落在当前 TCP）。tool Z 指向球心/背离球心时 roll 是自由量：
tool X 由法向序列平行移动
（接缝处姿态连续），脚本再按整条轨迹抽样预选一个绕 tool Z 的 roll 偏移、全程不变
（可用 `--roll-offset-deg` 强制指定并跳过预选），
用控制器 FK 回代、关节软限位与 |pitch| 上限筛掉「报 success 但姿态不对」「超出限位」
的解；换 roll 会让控制器逆解换分支（实测 45° 的 roll 改动换来 171° 关节重构、
TCP 离球面 9 mm）。`--no-roll-search` 关闭该搜索，退回固定参考 roll。弧的两端各让开
一个采样步长，避免赤道处切向竖直使目标 rpy 的 pitch 恰好落在 ±90°（万向锁，
该点上控制器会静默返回错姿态却报 success）。

时间轴与下发：roll 搜索优先选距关节软限位 ≥ `--min-joint-margin` 的解（贴限位的解
会把控制器 IK 逼到换分支，实测 J6 64.7°/65° 后整段翻转、TCP 甩出球面）；只有"按
--speed 节奏就会超过 --vmax-deg"的关节步长才当成跳变并拉伸（smoothstep 按 1.5 倍
峰值计入），每毫米都在动的冗余轴不再被误判成跳变而逐点停顿；blend 段不足时按 vmax
自动延长；下发线程被调度拖后时轨迹时钟一起顺延，不跳过轨迹点。

输出（默认 scripts/sphere_ik_bench_results/<时间戳>/）：
    target_path.pcd / .ply   理论球面轨迹点云（intensity = 弧段编号）
    ik_fk.pcd / .ply         控制器逆解回代 FK 的点云（intensity = 位置残差 mm）
    actual_tcp.pcd / .ply    实际 TCP 点云（intensity = 运行时间 s，仅 --execute）
    samples.csv              每个采样点的目标位姿、逆解、FK、残差、延迟
    commands.csv             100 Hz 下发关节序列（仅 --execute）
    actual_tcp.csv           实际 TCP 位姿序列（仅 --execute）
    actual_joints.csv        实际关节角序列（仅 --execute）
    summary.json / summary.txt
    preview.png              3D 预览图（matplotlib 可用时）

用法（需先 source install/setup.bash，tl_driver 已连接机械臂）：
    # 只解算、不动臂；默认从灵巧空间自动选球心（--workspace-data 指定 .npz）
    python3 scripts/sphere_ik_bench_half.py

    # 实画：默认已固化实测参数（--preset tcb705f-inward），只需 --execute
    python3 scripts/sphere_ik_bench_half.py --execute

    # 回到脚本原来的通用默认参数（orient=fixed、不自动放宽跳变/限位等）
    python3 scripts/sphere_ik_bench_half.py --preset none

    # 第二遍：先 MoveJ 回到上次成功运行记录的起始位姿，再画
    # （成功执行后默认会 MoveJ 回起始位姿并写入 --start-pose-file）
    python3 scripts/sphere_ik_bench_half.py --execute --restore-start

    # 回到旧行为：球心取当前 TCP + R·x，轨迹首点落在当前 TCP
    python3 scripts/sphere_ik_bench_half.py --no-auto-center

    # 改球心/半径/朝向，并用本地逆解做对比
    python3 scripts/sphere_ik_bench_half.py --center 0.30 0.00 0.40 --radius 0.04 \\
        --orient radial --ik-source 1 --allow-jumps

    # 工具 Z 始终指向球心（姿态随球面变化）
    python3 scripts/sphere_ik_bench_half.py --execute --orient inward

    # 预检不过也强行跑，主要看姿态是否正确
    python3 scripts/sphere_ik_bench_half.py --execute --force --orient inward

自检（不需要 ROS/机械臂）：
    python3 scripts/sphere_ik_bench_half.py --self-test
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import signal
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = SCRIPT_DIR / "sphere_ik_bench_results"

# 关节跳变判据：超过 0.05 rad 视为一次分支/姿态切换候选
DEFAULT_JUMP_THRESHOLD = 0.05
# 单个采样间隔允许的最大关节变化，超过则拒绝执行
DEFAULT_MAX_JOINT_STEP = 1.0
# IK 回代 FK 的位置残差上限（mm），超过则拒绝执行
DEFAULT_MAX_RESIDUAL_MM = 5.0
# roll 搜索步长（°）：tool Z 指向/背离球心时 roll 自由，按此粒度绕 tool Z 试解；
# 候选顺序按与上一帧姿态的角距离排序，优先保持 roll 连续
DEFAULT_ROLL_STEP_DEG = 45.0
# roll 搜索优先保留的关节软限位余量（°）：贴限位的解会在后续点把控制器 IK
# 逼到换分支（实测 J6 64.7°/65° 后整段翻转），有别的 roll 可选时避开
DEFAULT_ROLL_MARGIN_DEG = 5.0
# 全程预选 roll 时的抽样点数：候选各试解一次，取代逐点/逐段换 roll
ROLL_PRESCAN_POINTS = 16
# 目标 rpy 的俯仰角上限（°）：控制器在 ±90° 万向锁附近会「报 success 但姿态错」，
# 平行移动的 tool X 可能把 pitch 推近 90°，选 roll 时先在线挡掉
PITCH_LIMIT_DEG = 88.0
# 轨迹首点与当前 TCP 的允许位置偏差（m）：blend 只在关节空间插值，
# 首点偏离越远，过渡段 TCP 的实际走线越不受控
DEFAULT_START_GAP_TOL_M = 0.02
# 轨迹首点与当前 TCP 的允许姿态偏差（°）：radial/inward 会按球面法向重定向
DEFAULT_START_ROT_TOL_DEG = 10.0
# 球面采样步长（m）
DEFAULT_SAMPLE_STEP = 0.001
# 沿弧线速度（m/s）
DEFAULT_SPEED = 0.02
# 起步/收尾加减速时间（s）：路径段按梯形速度剖面推进
DEFAULT_RAMP_TIME = 0.5
# 关节距软限位的最小允许余量（rad）：低于此值时控制器逆解会开始失败
DEFAULT_MIN_JOINT_MARGIN = 0.02
# ServoJ 下发频率（Hz）
DEFAULT_RATE = 100.0
# 最后一帧下发后的等待时间（s）：让控制器把 ServoJ 目标跟随到位，
# 期间保持录制，使 actual_tcp 点云包含收尾段
DEFAULT_SERVOJ_SETTLE_S = 5.0
# 灵巧工作空间体素数据（dexterous_workspace.py --save-data 生成）
DEFAULT_WORKSPACE_DATA = (
    SCRIPT_DIR / "workspace_measure" / "results" / "tcb705f_dexterous_workspace.npz"
)
# 自动选球：粗校验的候选数、每条候选轨迹的抽样点数、首点允许偏差（m）
DEFAULT_AUTO_CANDIDATES = 8
DEFAULT_AUTO_VERIFY_SAMPLES = 64
DEFAULT_AUTO_START_GAP = 0.15
DEFAULT_AUTO_FULL_CANDIDATES = 4
DEFAULT_AUTO_MAX_STEP = 0.2
# 回起始位姿的 MoveJ 参数：速度百分比、action 超时（s）、acc/dec 百分比、pl
DEFAULT_START_POSE_FILE = SCRIPT_DIR / "sphere_ik_bench_results" / "start_pose.json"
DEFAULT_MOVE_VELOCITY = 20.0
DEFAULT_MOVE_TIMEOUT = 60.0
PREPOSE_ACC_PERCENT = 20.0
PREPOSE_PL = 0
PREPOSE_GOAL_TIMEOUT_S = 5.0
PREPOSE_RESULT_MARGIN_S = 10.0
# 已实测能连续画上半球（inward + 基座 z 上半球 + 5 cm）的参数组合；
# 命令行显式传参会覆盖，--preset none 回到脚本原来的通用默认值。
PRESETS = {
    "tcb705f-inward": {
        "orient": "inward",
        "roll_offset_deg": 45.0,
        "allow_jumps": True,
        "max_start_gap": 0.13,
        "max_start_rot": 120.0,
        "max_joint_step": 1.5,
        "speed": 0.01,
        "ramp_time": 4.0,
        "amax_deg": 1600.0,
    },
    "none": {},
}
DEFAULT_PRESET = "tcb705f-inward"
# 粗校验判据：相邻解最大关节变化（rad）与关节软限位容差（rad）
PROBE_MAX_STEP_RAD = 0.8
JOINT_LIMIT_MARGIN_RAD = 0.02


# ---------------------------------------------------------------------------
# 数学工具：欧拉角采用 XYZ 外旋（scipy 'xyz'），与控制器约定一致
# ---------------------------------------------------------------------------


def rpy_to_matrix(rpy) -> np.ndarray:
    """RPY（XYZ 外旋，scipy 'xyz'）→ 旋转矩阵：R = Rz(rz) @ Ry(ry) @ Rx(rx)。"""
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=float,
    )


def matrix_to_rpy(matrix: np.ndarray) -> np.ndarray:
    """旋转矩阵 → RPY（XYZ 外旋）；处理 ±90° 俯仰的万向锁。"""
    sy = -float(matrix[2, 0])
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(math.cos(ry)) > 1e-8:
        rx = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        rz = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    elif ry > 0.0:
        # ry = +π/2：R[0,1]=sin(rx-rz)、R[0,2]=cos(rx-rz)，约定 rz=0
        rx = math.atan2(float(matrix[0, 1]), float(matrix[0, 2]))
        rz = 0.0
    else:
        # ry = -π/2：R[0,1]=-sin(rx+rz)、R[0,2]=-cos(rx+rz)，约定 rz=0
        rx = math.atan2(-float(matrix[0, 1]), -float(matrix[0, 2]))
        rz = 0.0
    return np.array([rx, ry, rz], dtype=float)


def rotation_error_deg(rpy_a, rpy_b) -> float:
    """两个 RPY 表示的姿态之间的测地角（度）。"""
    ra = rpy_to_matrix(rpy_a)
    rb = rpy_to_matrix(rpy_b)
    cosine = (float(np.trace(ra.T @ rb)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def tool_z_error_deg(rpy_a, rpy_b) -> float:
    """两个姿态 tool Z 轴的夹角（°）：roll 自由时用它代替整姿态误差。"""
    za = rpy_to_matrix(rpy_a)[:, 2]
    zb = rpy_to_matrix(rpy_b)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(za, zb))))))


def unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("无法归一化零向量")
    return vector / norm


# ---------------------------------------------------------------------------
# 上半球轨迹：两个互相垂直的大圆（基座 xz 面 + yz 面）的 z ≥ 球心 z 半段
# ---------------------------------------------------------------------------

# 每段圆弧从自家 -e_i 赤道点起步（θ=-π/2），经北极点（θ=0）到 +e_i 赤道点
ARC_START = -0.5 * math.pi
ARC_SWEEP = math.pi
# 两段半圆之间的赤道连接弧：+e1 → -e2（绕基座 z 轴 90°）
BRIDGE_START = 0.0
BRIDGE_SWEEP = 0.5 * math.pi


@dataclass
class SpherePath:
    """一条按弧长采样的球面线框轨迹。"""

    center: np.ndarray
    radius: float
    positions: np.ndarray  # (N, 3) m
    rpys: np.ndarray  # (N, 3) rad
    s: np.ndarray  # (N,) 累计弧长 m
    segments: list[str]  # (N,) 所在弧段名称

    @property
    def length(self) -> float:
        return float(self.s[-1]) if len(self.s) else 0.0

    def segment_ids(self) -> np.ndarray:
        """弧段名称 → 连续整数，用作点云 intensity。"""
        mapping: dict[str, int] = {}
        ids = np.zeros(len(self.segments), dtype=float)
        for i, name in enumerate(self.segments):
            if name not in mapping:
                mapping[name] = len(mapping)
            ids[i] = mapping[name]
        return ids

    def truncate(self, count: int) -> "SpherePath":
        """截取前 count 个采样点（末尾逆解不可达时只执行可达部分）。"""
        count = max(1, min(int(count), len(self.positions)))
        return SpherePath(
            center=self.center,
            radius=self.radius,
            positions=self.positions[:count].copy(),
            rpys=self.rpys[:count].copy(),
            s=self.s[:count].copy(),
            segments=self.segments[:count],
        )

    def with_rpys(self, rpys) -> "SpherePath":
        """换一份逐点目标姿态（roll 搜索选出的姿态），几何保持不变。"""
        return SpherePath(
            center=self.center,
            radius=self.radius,
            positions=self.positions,
            rpys=np.array(rpys, dtype=float).copy(),
            s=self.s,
            segments=self.segments,
        )


def _append_arc(
    positions: list[np.ndarray],
    segment_names: list[str],
    center: np.ndarray,
    radius: float,
    u: np.ndarray,
    v: np.ndarray,
    start: float,
    sweep: float,
    step: float,
    name: str,
    include_end: bool = False,
) -> None:
    """把 p(θ)=C+R(cosθ·u+sinθ·v) 在 θ∈[start, start+sweep] 按弧长等步长采样追加。

    默认半开（不含终点），相邻弧段的接缝由下一段首点负责；include_end 时把
    θ=start+sweep 也发出（用于让弧末端停在赤道前一个步长处）。
    """
    arc_length = abs(sweep) * radius
    count = max(2, int(math.ceil(arc_length / max(step, 1e-6))))
    for i in range(count + 1 if include_end else count):
        theta = start + sweep * i / count
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        positions.append(center + radius * (cos_t * u + sin_t * v))
        segment_names.append(name)


def roll_candidates(step_deg: float = DEFAULT_ROLL_STEP_DEG) -> tuple[float, ...]:
    """绕 tool Z 的 roll 候选偏移（°）：0 优先，其余按绝对值递增，最后 180。"""
    step = max(abs(step_deg), 1.0)
    values = [0.0]
    value = step
    while value < 180.0 - 1e-9:
        values.extend([value, -value])
        value += step
    values.append(180.0)
    return tuple(values)


def rotate_rpy_about_z(rpy, offset_deg: float) -> np.ndarray:
    """绕自身 tool Z 旋转 offset_deg：tool Z 指向不变，只改 roll。"""
    matrix = rpy_to_matrix(rpy) @ rpy_to_matrix([0.0, 0.0, math.radians(offset_deg)])
    return matrix_to_rpy(matrix)


def order_roll_candidates(base_rpy, previous_rpy, step_deg: float) -> list[np.ndarray]:
    """按与上一帧姿态的角距离排序 roll 候选：优先让 roll 逐点连续。"""
    candidates = [rotate_rpy_about_z(base_rpy, offset) for offset in roll_candidates(step_deg)]
    if previous_rpy is None:
        return candidates
    previous = np.asarray(previous_rpy, dtype=float)
    return sorted(candidates, key=lambda rpy: rotation_error_deg(rpy, previous))


def perpendicular_component(vector, axis) -> np.ndarray | None:
    """向量在垂直于 axis 的平面上的分量；退化时返回 None。"""
    projected = np.asarray(vector, dtype=float) - float(np.dot(vector, axis)) * axis
    norm = float(np.linalg.norm(projected))
    if norm < 1e-9:
        return None
    return projected / norm


def rotate_about(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues 旋转：绕单位轴 axis 转 angle（rad）。"""
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        vector * cosine
        + np.cross(axis, vector) * sine
        + axis * float(np.dot(axis, vector)) * (1.0 - cosine)
    )


def pitch_deg(rpy) -> float:
    """rpy 的俯仰角（°）：R[2,0] = -sin(pitch)，±90° 是万向锁。"""
    return math.degrees(math.asin(max(-1.0, min(1.0, -float(rpy_to_matrix(rpy)[2, 0])))))


def parallel_frames(points: np.ndarray, normals: np.ndarray, start_x) -> np.ndarray:
    """tool 坐标系：tool Z = 法向，tool X 沿法向序列平行移动（rotation-minimizing）。

    接缝两侧法向相同（都是 ±e1），平行移动因此不产生额外 roll；若改用段内切向
    当 tool X，接缝处切向本身转 90°，请求姿态就带 90° 级 roll 台阶，控制器只能
    用几秒的关节重构去实现（实测一次 135° 台阶＝3.61 rad 重构、TCP 离球面 6 cm）。
    仅当法向反向（轴退化）时回退到固定参考轴。
    """
    frames = np.empty((len(points), 3, 3), dtype=float)
    x_axis = None
    previous_z = None
    for index, normal in enumerate(normals):
        z_axis = unit(np.asarray(normal, dtype=float))
        if x_axis is None:
            x_axis = perpendicular_component(start_x, z_axis)
        else:
            axis = np.cross(previous_z, z_axis)
            norm = float(np.linalg.norm(axis))
            if norm > 1e-9:
                angle = math.atan2(norm, float(np.dot(previous_z, z_axis)))
                x_axis = rotate_about(x_axis, axis / norm, angle)
        x_axis = perpendicular_component(x_axis, z_axis)
        if x_axis is None:
            for candidate in (np.eye(3)[0], np.eye(3)[1], np.eye(3)[2]):
                x_axis = perpendicular_component(candidate, z_axis)
                if x_axis is not None:
                    break
        if x_axis is None:
            raise ValueError("无法构造工具坐标系")
        frames[index] = np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])
        previous_z = z_axis
    return frames


def build_sphere_path(
    center: np.ndarray,
    radius: float,
    e1: np.ndarray,
    e2: np.ndarray,
    e3: np.ndarray,
    step: float,
    orient: str,
    base_rpy: np.ndarray,
) -> SpherePath:
    """生成上半球线框轨迹：两段大圆上半段 + 赤道连接弧。

    xz 面弧：p(θ)=C+R(cosθ·e3+sinθ·e1)，θ∈[-π/2, π/2)，从 C-R·e1 经北极点
    走到 +e1 附近；随后沿赤道走 90° 到 -e2 附近（equator_bridge）；yz 面弧
    同理从 C-R·e2 经北极点走到 +e2 附近。三段接缝只差一个采样步长，整条轨迹
    连续、全部落在球面上，z ≥ C_z。_build_path 未指定球心时取
    center = TCP + R·e1，使首点逼近当前 TCP。

    弧的两端各让开一个采样步长：赤道处切向竖直（±e3），若目标 rpy 的 pitch
    恰好是 ±90°，控制器逆解会静默返回错姿态。非 fixed 模式 tool Z 取 ±法向、
    tool X 沿法向平行移动（见 parallel_frames），接缝姿态连续；具体 roll 由
    _solve_path 按整条轨迹抽样预选一次。首点姿态仍由几何决定，不保证等于当前 TCP
    姿态，_precheck 会按首点位置/姿态跨度给阻断项。
    """
    if radius <= 0.0 or step <= 0.0:
        raise ValueError("radius/step 必须大于 0")
    guard = min(step / radius, 0.25 * ARC_SWEEP)  # 两端各让开的角度（rad）
    positions: list[np.ndarray] = []
    segment_names: list[str] = []

    _append_arc(
        positions,
        segment_names,
        center,
        radius,
        e3,
        e1,
        ARC_START + guard,
        ARC_SWEEP - 2.0 * guard,
        step,
        "circle_xz",
        include_end=True,
    )
    # 赤道连接弧：+e1 → -e2，替代原先 70.7 mm 的离球面直线跃迁。两端同样让开
    # 一个步长：赤道点 tool Z 水平（pitch 恰为 ±90°），rpy 无法表达任意 roll
    _append_arc(
        positions,
        segment_names,
        center,
        radius,
        e1,
        -e2,
        BRIDGE_START + guard,
        BRIDGE_SWEEP - 2.0 * guard,
        step,
        "equator_bridge",
        include_end=True,
    )
    _append_arc(
        positions,
        segment_names,
        center,
        radius,
        e3,
        e2,
        ARC_START + guard,
        ARC_SWEEP - 2.0 * guard,
        step,
        "circle_yz",
        include_end=True,
    )

    points = np.asarray(positions, dtype=float)
    if orient == "fixed":
        rpys = np.tile(np.asarray(base_rpy, dtype=float), (len(points), 1))
    elif orient in ("radial", "inward"):
        # radial: tool Z 背离球心；inward: tool Z 指向球心
        sign = 1.0 if orient == "radial" else -1.0
        normals = sign * (points - np.asarray(center, dtype=float)) / float(radius)
        frames = parallel_frames(points, normals, rpy_to_matrix(base_rpy)[:, 0])
        rpys = np.array([matrix_to_rpy(frame) for frame in frames])
    else:
        raise ValueError(f"未知姿态模式: {orient}")

    deltas = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(deltas)])
    return SpherePath(
        center=np.asarray(center, dtype=float),
        radius=float(radius),
        positions=points,
        rpys=rpys,
        s=s,
        segments=segment_names,
    )


# ---------------------------------------------------------------------------
# 灵巧工作空间数据与上半球自动选球
# ---------------------------------------------------------------------------


@dataclass
class WorkspaceData:
    """灵巧工作空间体素：规则网格上的灵巧度，NaN 表示不可达。"""

    origin: np.ndarray
    step: float
    dexterity: np.ndarray
    interior: np.ndarray
    arm_type: str
    tau_rad: float


@dataclass
class SphereCandidate:
    """候选球：球心、半径与离线几何评分。"""

    center: np.ndarray
    radius: float
    min_dexterity: float
    mean_dexterity: float
    start_gap: float


def _erode(mask: np.ndarray) -> np.ndarray:
    """3×3×3 腐蚀：仅当周围 26 个体素都可达时才算「内部」。"""
    padded = np.pad(mask, 1, constant_values=False)
    nx, ny, nz = mask.shape
    interior = np.ones_like(mask)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                interior &= padded[1 + dx : 1 + dx + nx, 1 + dy : 1 + dy + ny, 1 + dz : 1 + dz + nz]
    return interior


def load_workspace_data(path: Path) -> WorkspaceData:
    """读取 `dexterous_workspace.py --save-data` 生成的 .npz。"""
    with np.load(path) as data:
        centers = np.asarray(data["centers"], dtype=float)
        dexterity = np.asarray(data["dexterity"], dtype=float)
        step = float(data["step"])
        arm_type = str(data["arm_type"]) if "arm_type" in data else ""
        tau_rad = float(data["tau"]) if "tau" in data else float("nan")
    if centers.ndim != 2 or centers.shape[1] != 3 or centers.shape[0] == 0:
        raise ValueError("工作空间数据没有有效的体素中心")
    if dexterity.shape[0] != centers.shape[0]:
        raise ValueError("dexterity 长度与 centers 不一致")
    if step <= 0.0:
        raise ValueError("工作空间体素 step 必须大于 0")
    origin = centers.min(axis=0)
    shape = np.round((centers.max(axis=0) - origin) / step).astype(int) + 1
    grid = np.full(tuple(int(value) for value in shape), np.nan)
    indices = np.round((centers - origin) / step).astype(int)
    grid[indices[:, 0], indices[:, 1], indices[:, 2]] = dexterity
    reachable = np.isfinite(grid)
    return WorkspaceData(
        origin=origin,
        step=step,
        dexterity=grid,
        interior=_erode(reachable),
        arm_type=arm_type,
        tau_rad=tau_rad,
    )


def half_sphere_candidates(
    workspace: WorkspaceData,
    radius: float,
    tcp_position: np.ndarray,
    max_start_gap: float,
    limit: int,
    verify_points: int,
    orient: str,
    base_rpy: np.ndarray,
) -> list[SphereCandidate]:
    """在灵巧空间内搜索上半球线框可用的球心，按灵巧度排序。

    上半球路径固定为基座坐标系 xz/yz 两个大圆的上半段 + 赤道连接弧，极轴
    为基座 z。对每个内部体素中心生成一条粗采样路径（约 verify_points 点），
    只保留「路径首点距当前 TCP ≤ max_start_gap」且「每个采样点所在体素都是
    内部体素」的球心；评分取路径采样点的最小/平均灵巧度。
    """
    if radius <= 0.0:
        raise ValueError("radius 必须大于 0")
    if limit <= 0:
        return []
    e1 = np.array([1.0, 0.0, 0.0])
    e2 = np.array([0.0, 1.0, 0.0])
    e3 = np.array([0.0, 0.0, 1.0])
    # 上半球线框总长约 2.5πR；步长按抽样点数展开，并保证不细于体素 1/4
    coarse_step = max(2.5 * math.pi * radius / max(int(verify_points), 8), workspace.step * 0.25)
    cells = np.argwhere(np.isfinite(workspace.dexterity) & workspace.interior)
    if cells.shape[0] == 0:
        return []
    centers = workspace.origin + cells * workspace.step
    tcp_position = np.asarray(tcp_position, dtype=float)
    shape = np.asarray(workspace.dexterity.shape, dtype=int)
    candidates: list[SphereCandidate] = []
    for center in centers:
        path = build_sphere_path(
            center=center,
            radius=radius,
            e1=e1,
            e2=e2,
            e3=e3,
            step=coarse_step,
            orient=orient,
            base_rpy=base_rpy,
        )
        start_gap = float(np.linalg.norm(path.positions[0] - tcp_position))
        if start_gap > max_start_gap:
            continue
        ijk = np.round((path.positions - workspace.origin) / workspace.step).astype(int)
        if np.any(ijk < 0) or np.any(ijk >= shape):
            continue
        inside = workspace.interior[ijk[:, 0], ijk[:, 1], ijk[:, 2]]
        if not bool(inside.all()):
            continue
        sampled = workspace.dexterity[ijk[:, 0], ijk[:, 1], ijk[:, 2]]
        candidates.append(
            SphereCandidate(
                center=center,
                radius=float(radius),
                min_dexterity=float(sampled.min()),
                mean_dexterity=float(sampled.mean()),
                start_gap=start_gap,
            )
        )
    candidates.sort(
        key=lambda item: (item.min_dexterity, item.mean_dexterity, -item.start_gap),
        reverse=True,
    )
    return candidates[:limit]


# ---------------------------------------------------------------------------
# 时间轴：把采样点按速度展开成可 100 Hz 跟随的关节序列
# ---------------------------------------------------------------------------


@dataclass
class Timeline:
    """分段线性指令时间轴：路径段匀速推进，blend/jump 段用 smoothstep。"""

    times: np.ndarray  # 每个断点的累计时刻 s
    joints: np.ndarray  # (M, dof) 断点关节角
    positions: np.ndarray  # (M, 3) 断点目标位置
    rpys: np.ndarray  # (M, 3) 断点目标姿态（已逐列展开，线性插值不会跨 ±π 折返）
    kinds: list[str]  # 每段类型，长度 = len(times)-1；kinds[k] 属于 [times[k], times[k+1]]

    @property
    def total(self) -> float:
        return float(self.times[-1])

    def sample(self, t: float):
        """返回 (关节角, 目标位置, 目标姿态, 段类型)。

        路径段按速度剖面线性推进（段内匀速，峰值不再有 1.5 倍纹波）；
        blend/jump 段需要零速度端点，仍用 smoothstep。
        """
        if t <= 0.0:
            return self.joints[0], self.positions[0], self.rpys[0], self._kind_at(0)
        if t >= self.total:
            return (
                self.joints[-1],
                self.positions[-1],
                self.rpys[-1],
                self._kind_at(len(self.times) - 1),
            )
        index = int(np.searchsorted(self.times, t, side="right"))
        index = min(max(index, 1), len(self.times) - 1)
        t0, t1 = float(self.times[index - 1]), float(self.times[index])
        alpha = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        kind = self._kind_at(index - 1)
        if kind != "path":
            alpha = alpha * alpha * (3.0 - 2.0 * alpha)  # smoothstep
        joints = self.joints[index - 1] + alpha * (self.joints[index] - self.joints[index - 1])
        position = self.positions[index - 1] + alpha * (
            self.positions[index] - self.positions[index - 1]
        )
        rpy = self.rpys[index - 1] + alpha * (self.rpys[index] - self.rpys[index - 1])
        return joints, position, rpy, kind

    def _kind_at(self, segment: int) -> str:
        if not self.kinds:
            return "path"
        return self.kinds[min(max(segment, 0), len(self.kinds) - 1)]


def _trapezoid_times(s: np.ndarray, speed: float, accel: float) -> np.ndarray:
    """沿弧长 s 的梯形速度剖面时刻表：两端按 accel 加减速，中间匀速。

    逐点独立 smoothstep 会让指令在每个采样点归零（峰值 1.5× 平均、与 100 Hz
    下发网格混叠）；换成沿弧长一次积分出的速度剖面，段内匀速、只在首尾加减速。
    """
    distances = np.asarray(s, dtype=float)
    length = float(distances[-1]) if distances.size else 0.0
    if length <= 0.0 or speed <= 0.0:
        return np.zeros_like(distances)
    if not math.isfinite(accel) or accel <= 0.0:
        return distances / speed
    peak = min(speed, math.sqrt(accel * length))
    ramp = peak * peak / (2.0 * accel)
    cruise = max(length - 2.0 * ramp, 0.0)
    total = 2.0 * peak / accel + cruise / peak
    times = np.empty_like(distances)
    for index, distance in enumerate(distances):
        if distance <= ramp:
            times[index] = math.sqrt(max(2.0 * distance / accel, 0.0))
        elif distance >= length - ramp:
            times[index] = total - math.sqrt(max(2.0 * (length - distance) / accel, 0.0))
        else:
            times[index] = peak / accel + (distance - ramp) / peak
    return times


def build_timeline(
    path: SpherePath,
    solutions: np.ndarray,
    speed: float,
    current_joints: np.ndarray,
    blend_time: float,
    jump_threshold: float,
    allow_jumps: bool,
    vmax: float,
    ramp_time: float,
) -> Timeline:
    """把逆解序列展开成时间轴。

    blend_time>0 时先用 blend_time 从当前关节平滑过渡到首点（不足时按 vmax 自动
    延长）；路径段按弧长梯形速度剖面（--speed 匀速 + --ramp-time 加减速）定时。
    相邻解关节步长超过 jump_threshold **且** 按路径节奏就会超过 vmax 时才算跳变：
    allow_jumps=True 则把该段拉长到 smoothstep 峰值也不超过 vmax，否则保持原节奏
    （执行前拦截）。smoothstep 峰值是平均速度的 1.5 倍，拉伸按 1.5 倍计入，否则
    控制器仍会被限速拖住而跟不上。
    kinds 与段一一对应（长度 len(times)-1），sample() 返回的就是该段类型。
    """
    if speed <= 0.0:
        raise ValueError("speed 必须大于 0")
    accel = speed / ramp_time if ramp_time > 0.0 else float("inf")
    path_times = _trapezoid_times(np.asarray(path.s, dtype=float), speed, accel)
    # 断点 rpy 逐列展开，避免线性插值在 ±π 处绕远路（只影响指标/预览）
    unwrapped_rpys = np.unwrap(np.asarray(path.rpys, dtype=float), axis=0)
    peak_factor = 1.5  # smoothstep 峰值 / 平均
    speed_limit = max(vmax, 1e-6)

    times = [0.0]
    if blend_time > 0.0:
        joints = [np.asarray(current_joints, dtype=float)]
        kinds: list[str] = []
    else:
        joints = [np.asarray(solutions[0], dtype=float).copy()]
        kinds = []
    positions = [path.positions[0].copy()]
    rpys = [unwrapped_rpys[0].copy()]

    if blend_time > 0.0:
        first = np.asarray(solutions[0], dtype=float)
        current = np.asarray(current_joints, dtype=float)
        shared = min(first.size, current.size)
        first_step = float(np.max(np.abs(first[:shared] - current[:shared])))
        blend_time = max(blend_time, peak_factor * first_step / speed_limit)
        times.append(blend_time)
        joints.append(np.asarray(solutions[0], dtype=float).copy())
        positions.append(path.positions[0].copy())
        rpys.append(unwrapped_rpys[0].copy())
        kinds.append("blend_in")

    for i in range(1, len(solutions)):
        duration = float(path_times[i] - path_times[i - 1])
        step = float(np.max(np.abs(solutions[i] - solutions[i - 1])))
        kind = "path"
        # 只有"按路径节奏就会超过 vmax"的步长才需要单独拉伸；每毫米都动一点的
        # 冗余轴（如 7 轴臂角）不算跳变，否则会被固定下限拖成逐点停顿
        if step > jump_threshold and step > 0.9 * speed_limit * duration:
            kind = "jump"
            if allow_jumps:
                duration = max(duration, peak_factor * step / speed_limit)
        times.append(times[-1] + max(duration, 1e-4))
        joints.append(solutions[i].copy())
        positions.append(path.positions[i].copy())
        rpys.append(unwrapped_rpys[i].copy())
        kinds.append(kind)

    return Timeline(
        times=np.asarray(times, dtype=float),
        joints=np.asarray(joints, dtype=float),
        positions=np.asarray(positions, dtype=float),
        rpys=np.asarray(rpys, dtype=float),
        kinds=kinds,
    )


def start_gap_metrics(
    path: SpherePath,
    current_pose: np.ndarray,
    current_joints: np.ndarray,
    solutions: np.ndarray,
    ok: np.ndarray,
) -> dict:
    """首点目标相对当前 TCP 的跨度：位置（m）、姿态（°）、关节（rad）。

    blend 段在关节空间插值到首点解，位置固定为首点、姿态沿首点解，
    跨度越大过渡段 TCP 走线越不受控，也是 ServoJ 限速的输入。
    """
    gap = {
        "position_m": float(np.linalg.norm(path.positions[0] - current_pose[:3])),
        "rotation_deg": rotation_error_deg(path.rpys[0], current_pose[3:]),
    }
    if bool(ok[0]):
        gap["joint_rad"] = float(np.max(np.abs(solutions[0] - current_joints)))
    return gap


# ---------------------------------------------------------------------------
# 点云 / CSV 输出
# ---------------------------------------------------------------------------


def _fill_missing_solutions(
    solutions: np.ndarray, ok: np.ndarray, fallback: np.ndarray | None = None
) -> np.ndarray:
    """把失败采样点用前一个有效解填充；开头失败时用 fallback（当前关节）。"""
    filled = np.asarray(solutions, dtype=float).copy()
    if fallback is None:
        fallback = np.zeros(filled.shape[1], dtype=float)
    base = np.asarray(fallback, dtype=float).copy()
    last_valid = None
    for index in range(len(filled)):
        if bool(ok[index]):
            last_valid = filled[index].copy()
        elif last_valid is not None:
            filled[index] = last_valid
        else:
            filled[index] = base
    return filled


def last_valid_index(ok: np.ndarray) -> int:
    """最后一个逆解成功点的下标；全部失败返回 -1。"""
    valid = np.flatnonzero(np.asarray(ok, dtype=bool))
    return int(valid[-1]) if valid.size else -1


def write_pcd(path: Path, points: np.ndarray, intensity=None, comment: str = "") -> None:
    """写 PCD 0.7 ASCII 点云（FIELDS x y z intensity）。"""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points 形状必须为 (N, 3)")
    count = points.shape[0]
    if count == 0:
        print(f"跳过空点云: {path}")
        return
    intensity = np.zeros(count) if intensity is None else np.asarray(intensity, dtype=float)
    if intensity.shape[0] != count:
        raise ValueError("intensity 长度与点数不一致")
    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        f"# {comment}" if comment else "# sphere_ik_bench",
        "VERSION 0.7",
        "FIELDS x y z intensity",
        "SIZE 4 4 4 4",
        "TYPE F F F F",
        "COUNT 1 1 1 1",
        f"WIDTH {count}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {count}",
        "DATA ascii",
    ]
    lines = header + [
        f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {float(i):.6f}" for p, i in zip(points, intensity)
    ]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_ply(path: Path, points: np.ndarray, intensity=None, comment: str = "") -> None:
    """写 PLY 1.0 ASCII 点云，便于 CloudCompare/MeshLab 直接打开。"""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points 形状必须为 (N, 3)")
    count = points.shape[0]
    if count == 0:
        print(f"跳过空点云: {path}")
        return
    intensity = np.zeros(count) if intensity is None else np.asarray(intensity, dtype=float)
    if intensity.shape[0] != count:
        raise ValueError("intensity 长度与点数不一致")
    header = [
        "ply",
        "format ascii 1.0",
        f"comment {comment}" if comment else "comment sphere_ik_bench",
        f"element vertex {count}",
        "property float x",
        "property float y",
        "property float z",
        "property float intensity",
        "end_header",
    ]
    lines = header + [
        f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {float(i):.6f}" for p, i in zip(points, intensity)
    ]
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_csv(path: Path, header: list[str], rows: list[list]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# ROS 节点
# ---------------------------------------------------------------------------


class SphereIKBench:
    """rclpy 封装：逆解扫描、ServoJ 执行、数据采集与结果落盘。"""

    def __init__(self, args: argparse.Namespace):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from tl_ros2_interface.action import Move
        from tl_ros2_interface.msg import CartesianPose, ErrorInfo

        self.args = args
        self.node = rclpy.create_node("sphere_ik_bench")
        self.logger = self.node.get_logger()
        self.cb_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        # 状态与录制缓冲
        self._tcp_msg = None  # 最新 /tcp_pose
        self._joint_msg = None  # 最新 /joint_states
        self._recording = False
        self._cmd_rec: list[tuple[float, np.ndarray]] = []
        self._tcp_rec: list[tuple[float, np.ndarray, np.ndarray]] = []
        self._joint_rec: list[tuple[float, np.ndarray]] = []
        self._errors: list[tuple[float, int, int, str]] = []
        self._stop = False
        self._fk_preview = np.zeros((0, 6))
        # 自己接管 SIGINT/SIGTERM：只置停止标志，保证 finally 里的 close_servoj 还能执行
        self._signal_handlers: dict[int, object] = {}
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                self._signal_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, self._handle_signal)
            except (ValueError, OSError):
                self._signal_handlers.pop(signum, None)

        # 订阅 / 发布
        self.node.create_subscription(
            CartesianPose, "/tcp_pose", self._on_tcp_pose, 20, callback_group=self.cb_group
        )
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 20, callback_group=self.cb_group
        )
        self.node.create_subscription(
            ErrorInfo,
            "/tl_driver/error_info",
            self._on_error_info,
            100,
            callback_group=self.cb_group,
        )
        self.servoj_pub = self.node.create_publisher(
            Float64MultiArray, "/tl_driver/set_servoj_pos", 10
        )

        # 服务客户端
        from rcl_interfaces.srv import GetParameters
        from std_srvs.srv import Trigger
        from tl_ros2_interface.srv import (
            Fk,
            GetArmStatus,
            GetRobotJointParam,
            GetSpeed,
            Ik,
            OpenServoJ,
            SetSpeed,
        )

        self._Ik = Ik
        self._Fk = Fk
        self._GetArmStatus = GetArmStatus
        self._GetRobotJointParam = GetRobotJointParam
        self._GetSpeed = GetSpeed
        self._OpenServoJ = OpenServoJ
        self._SetSpeed = SetSpeed
        self._Trigger = Trigger
        self._GetParameters = GetParameters

        def make_client(srv_type, name):
            return self.node.create_client(srv_type, name, callback_group=self.cb_group)

        self.ik_cli = make_client(Ik, "/tl_driver/ik")
        self.fk_cli = make_client(Fk, "/tl_driver/fk")
        self.open_cli = make_client(OpenServoJ, "/tl_driver/open_servoj")
        self.close_cli = make_client(Trigger, "/tl_driver/close_servoj")
        self.status_cli = make_client(GetArmStatus, "/tl_driver/get_arm_status")
        self.joint_param_cli = make_client(GetRobotJointParam, "/tl_driver/get_robot_joint_param")
        self.set_speed_cli = make_client(SetSpeed, "/tl_driver/set_speed")
        self.get_speed_cli = make_client(GetSpeed, "/tl_driver/get_speed")
        self.param_cli = make_client(GetParameters, "/tl_driver/get_parameters")
        self._Move = Move
        self.move_cli = ActionClient(self.node, Move, "/tl_driver/move_joint")

    # ------------------------------------------------------------------
    # 基础调用
    # ------------------------------------------------------------------

    def _call(
        self, client, request, name: str, timeout: float | None = None, ignore_stop: bool = False
    ):
        timeout = self.args.timeout if timeout is None else timeout
        future = client.call_async(request)
        done = threading.Event()
        # 响应到达时由执行器线程唤醒；等待期间释放 GIL，不阻塞 spin 线程
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

    def _handle_signal(self, signum, frame) -> None:
        """信号处理只置位；真正的收尾在 _execute 的 finally 与 close()。"""
        self._stop = True

    def _wait_services(self) -> None:
        checks = [
            (self.ik_cli, "/tl_driver/ik"),
            (self.fk_cli, "/tl_driver/fk"),
            (self.open_cli, "/tl_driver/open_servoj"),
            (self.close_cli, "/tl_driver/close_servoj"),
            (self.status_cli, "/tl_driver/get_arm_status"),
            (self.joint_param_cli, "/tl_driver/get_robot_joint_param"),
        ]
        for client, name in checks:
            if self._stop:
                raise RuntimeError("已中断")
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"服务 {name} 不可用，请确认 tl_driver 已启动并连接机械臂")
        self.logger.info("所需服务已就绪")

    def _wait_state(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop:
                raise RuntimeError("已中断")
            if self._tcp_msg is not None and self._joint_msg is not None:
                return
            time.sleep(0.02)
        raise RuntimeError("等待 /tcp_pose 与 /joint_states 超时")

    def _driver_arm_type(self) -> str | None:
        """读取 tl_driver 节点的只读参数 arm_type；失败返回 None。"""
        if not self.param_cli.service_is_ready():
            if not self.param_cli.wait_for_service(timeout_sec=2.0):
                return None
        try:
            request = self._GetParameters.Request()
            request.names = ["arm_type"]
            response = self._call(self.param_cli, request, "get_parameters", timeout=2.0)
        except Exception as exc:
            self.logger.warn(f"读取驱动 arm_type 失败: {exc}")
            return None
        if response is None or len(response.values) != 1:
            return None
        value = str(response.values[0].string_value)
        return value or None

    # ------------------------------------------------------------------
    # 起始位姿记录 / MoveJ
    # ------------------------------------------------------------------

    def _state_stamp(self):
        """当前 /tcp_pose 与 /joint_states 的时间戳，用于判断 MoveJ 后状态已刷新。"""
        tcp = self._stamp_seconds(self._tcp_msg.header.stamp) if self._tcp_msg else None
        joint = self._stamp_seconds(self._joint_msg.header.stamp) if self._joint_msg else None
        return (tcp, joint)

    def _wait_state_change(self, before, timeout: float = 5.0) -> None:
        """等待两路状态都收到新帧（MoveJ 结果返回后状态流可能还停在旧帧）。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop:
                raise RuntimeError("已中断")
            after = self._state_stamp()
            if all(value is not None for value in after) and after != before:
                return
            time.sleep(0.02)
        raise RuntimeError("等待 MoveJ 后状态刷新超时")

    def _read_start_pose(self, path: Path):
        """读取记录的起始位姿；格式错误/文件不存在返回 None。"""
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            joints = np.asarray(data["joints"], dtype=float)
            pose = np.asarray(data["pose"], dtype=float)
            if joints.size not in (6, 7) or pose.size != 6:
                raise ValueError("joints 需 6/7 个，pose 需 6 个")
            return {
                "joints": joints,
                "pose": pose,
                "arm_angle": float(data.get("arm_angle", 0.0)),
            }
        except Exception as exc:
            self.logger.warn(f"读取起始位姿 {path} 失败: {exc}")
            return None

    def _write_start_pose(self, path: Path, joints, pose, arm_angle: float) -> None:
        """把成功运行前的起始关节/位姿写入固定文件，供下次 --restore-start。"""
        payload = {
            "joints": [float(v) for v in np.asarray(joints, dtype=float)],
            "pose": [float(v) for v in np.asarray(pose, dtype=float)],
            "arm_angle": float(arm_angle),
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "source": "successful sphere_ik_bench_half run",
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _wait_action_future(self, future, name: str, timeout: float, ignore_stop: bool = False):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout
        while not done.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(f"{name} 超时（{timeout:.1f}s）")
            if self._stop and not ignore_stop:
                raise RuntimeError(f"{name} 被中断")
            done.wait(min(0.05, remaining))
        return future.result()

    def _move_to_joints(self, joints, dof: int) -> None:
        """move_joint（MoveJ）到指定关节角；用于回起始位姿。"""
        if not self.move_cli.wait_for_server(timeout_sec=PREPOSE_GOAL_TIMEOUT_S):
            raise RuntimeError("/tl_driver/move_joint action 不可用")
        goal = self._Move.Goal()
        command = goal.command
        values = [0.0] * 14
        for axis in range(min(7, dof, len(joints))):
            values[axis] = float(joints[axis])
        command.target_pos_value = values
        command.coord = 0
        command.velocity = float(self.args.move_velocity)
        command.velocity_sync = 0.0
        command.acc = PREPOSE_ACC_PERCENT
        command.dec = PREPOSE_ACC_PERCENT
        command.pl = PREPOSE_PL
        command.tool_num = 0
        command.user_num = 0
        command.para_sync = False
        goal.timeout_sec = float(self.args.move_timeout)
        self.logger.warn(
            f"MoveJ 到起点（{self.args.move_velocity:.0f}%，超时 "
            f"{self.args.move_timeout:.0f}s）"
        )
        send_future = self.move_cli.send_goal_async(goal)
        handle = self._wait_action_future(send_future, "move_joint goal", PREPOSE_GOAL_TIMEOUT_S)
        if not handle.accepted:
            raise RuntimeError("move_joint goal 被拒绝")
        result = self._wait_action_future(
            handle.get_result_async(),
            "move_joint",
            self.args.move_timeout + PREPOSE_RESULT_MARGIN_S,
            ignore_stop=True,
        )
        response = result.result
        if response is None or not response.success:
            message = response.message if response is not None else "无响应"
            raise RuntimeError(f"move_joint 失败: {message}")
        self.logger.info("MoveJ 已完成")

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else time.time()

    def _on_tcp_pose(self, msg) -> None:
        self._tcp_msg = msg
        if not self._recording:
            return
        pose = np.array(
            [
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.rpy.x,
                msg.rpy.y,
                msg.rpy.z,
            ],
            dtype=float,
        )
        self._tcp_rec.append((self._stamp_seconds(msg.header.stamp), pose))

    def _on_joint_state(self, msg) -> None:
        self._joint_msg = msg
        if not self._recording:
            return
        joints = np.asarray(msg.position, dtype=float)
        self._joint_rec.append((self._stamp_seconds(msg.header.stamp), joints))

    def _on_error_info(self, msg) -> None:
        self._errors.append(
            (time.time(), int(msg.message_type), int(msg.message_code), msg.message)
        )
        self.logger.warn(
            f"控制器消息 type={msg.message_type} code={msg.message_code}: {msg.message}"
        )

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------

    def close(self) -> None:
        """停止执行器、等待 spin 线程退出并销毁节点。"""
        self._stop = True
        for signum, handler in self._signal_handlers.items():
            try:
                signal.signal(signum, handler)
            except (ValueError, OSError):
                pass
        self._signal_handlers.clear()
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

    def run(self) -> bool:
        try:
            if self.args.rate <= 0.0:
                raise RuntimeError(f"rate 必须大于 0，当前 {self.args.rate}")
            if self.args.speed <= 0.0:
                raise RuntimeError(f"speed 必须大于 0，当前 {self.args.speed}")
            if self.args.sample_step <= 1e-6:
                raise RuntimeError(f"sample-step 必须大于 1e-6 m，当前 {self.args.sample_step}")
            if self.args.hold_time < 0.0 or self.args.blend_time < 0.0:
                raise RuntimeError("hold-time / blend-time 不能为负")
            if self.args.ramp_time < 0.0:
                raise RuntimeError("ramp-time 不能为负")
            # 每帧最多跨一个采样点，否则轨迹被跳点、局部速度会偏离 --speed
            max_speed = self.args.sample_step * self.args.rate
            if self.args.speed > max_speed:
                raise RuntimeError(
                    f"speed={self.args.speed:.3f} m/s 超过 sample_step×rate={max_speed:.3f} m/s，"
                    "每帧会跨过多个采样点；请提高 --rate 或减小 --sample-step/--speed"
                )
            self._wait_services()
            self._wait_state()
            status = self._call(self.status_cli, self._GetArmStatus.Request(), "get_arm_status")
            if status is None or not status.success:
                raise RuntimeError("get_arm_status 调用失败")
            if status.run_state != "STOP":
                text = f"机械臂当前 run_state={status.run_state}，应为 STOP"
                if self.args.execute:
                    raise RuntimeError(text)
                self.logger.warn(text)

            if self.args.restore_start:
                if not self.args.execute:
                    self.logger.warn(
                        "--restore-start 只在 --execute 时移动机械臂，当前按现位姿解算"
                    )
                else:
                    recorded = self._read_start_pose(Path(self.args.start_pose_file))
                    if recorded is not None:
                        before = self._state_stamp()
                        self.logger.warn(f"回到记录的起始位姿: {self.args.start_pose_file}")
                        self._move_to_joints(recorded["joints"], len(recorded["joints"]))
                        self._wait_state_change(before)
                    else:
                        self.logger.warn(
                            f"--restore-start 找不到可用起始位姿，按当前位姿继续: "
                            f"{self.args.start_pose_file}"
                        )

            dof = len(self._joint_msg.position)
            if dof not in (6, 7):
                raise RuntimeError(f"/joint_states 关节数为 {dof}，期望 6 或 7")
            current_joints = np.asarray(self._joint_msg.position[:dof], dtype=float)
            tcp = self._tcp_msg
            current_pose = np.array(
                [
                    tcp.position.x,
                    tcp.position.y,
                    tcp.position.z,
                    tcp.rpy.x,
                    tcp.rpy.y,
                    tcp.rpy.z,
                ],
                dtype=float,
            )
            arm_angle = float(tcp.arm_angle) if math.isfinite(float(tcp.arm_angle)) else 0.0
            initial_joints = current_joints.copy()
            initial_pose = current_pose.copy()
            initial_arm_angle = arm_angle

            path, _ = self._build_path(current_pose, current_joints, arm_angle, dof)
            self.logger.info(
                f"球心={np.round(path.center, 4).tolist()} m, R={path.radius:.3f} m, "
                f"采样={len(path.positions)}, 弧长={path.length:.3f} m, 姿态={self.args.orient}"
            )

            limits = self._joint_limits(dof) if self.args.check_limits else None
            cache = getattr(self, "_validated", None)
            if (
                cache is not None
                and np.allclose(cache["center"], path.center)
                and abs(cache["radius"] - path.radius) < 1e-9
            ):
                self.logger.info("复用自动选球阶段的全量校验逆解，跳过重复扫描")
                solutions = cache["solutions"]
                ok = cache["ok"]
                latency_ms = cache["latency_ms"]
                messages = cache["messages"]
                fk_poses = cache["fk_poses"]
                chosen_rpys = cache["chosen_rpys"]
                calls = cache["calls"]
                self._validated = None
            else:
                solutions, ok, latency_ms, messages, fk_poses, chosen_rpys, calls = (
                    self._solve_path(path, current_joints, arm_angle, dof, limits, current_pose[3:])
                )
            if self._stop:
                self.logger.warn("收到中断，停止扫描，不执行运动")
                return False
            # 后续预检/时间轴/输出都以实际请求并通过验证的姿态为准
            path = path.with_rpys(chosen_rpys)
            exec_solutions = _fill_missing_solutions(solutions, ok, current_joints)
            last_ok = last_valid_index(ok)
            if last_ok < 0:
                exec_end = len(ok)
                exec_path = path
            else:
                exec_end = last_ok + 1
                exec_path = path.truncate(exec_end) if exec_end < len(ok) else path
                if exec_end < len(ok):
                    self.logger.warn(
                        f"末尾 {len(ok) - exec_end} 个采样点逆解失败，执行轨迹截断到 "
                        f"{exec_end}/{len(ok)} 点；请检查关节限位与可达性"
                    )
            timeline = build_timeline(
                exec_path,
                exec_solutions[:exec_end],
                self.args.speed,
                current_joints,
                self.args.blend_time,
                self.args.jump_threshold,
                self.args.allow_jumps or self.args.force,
                math.radians(self.args.vmax_deg),
                self.args.ramp_time,
            )
            metrics = self._precheck(
                path,
                solutions,
                ok,
                latency_ms,
                messages,
                fk_poses,
                dof,
                current_pose,
                current_joints,
                timeline,
                exec_end,
                limits,
                calls,
            )
            self._print_precheck(metrics)

            metrics["selection"] = getattr(self, "_selection", {"source": "fixed-relative"})
            metrics["roll_offset_deg"] = getattr(self, "_roll_offset_deg", None)
            executed = False
            if self.args.execute:
                if metrics["blocking"]:
                    if not self.args.force:
                        self.logger.error("预检未通过，拒绝执行运动。请先处理上面列出的问题。")
                        self._write_outputs(
                            path, solutions, ok, latency_ms, messages, fk_poses, metrics
                        )
                        return False
                    self.logger.warn(
                        "--force：忽略预检阻断项，仍然执行；请确认工作区安全并随时准备急停"
                    )
                if not ok.any():
                    self.logger.error("全部采样逆解失败，没有可执行的关节序列")
                    self._write_outputs(
                        path, solutions, ok, latency_ms, messages, fk_poses, metrics
                    )
                    return False
                try:
                    self._execute(path, exec_solutions[:exec_end], timeline, current_joints, dof)
                    executed = True
                except Exception as exc:
                    self.logger.error(f"执行失败: {exc}")
                    metrics["execution_error"] = str(exc)
                metrics["actual"] = self._tracking_metrics(path, timeline, dof)
                self._print_tracking(metrics["actual"])

            if executed and self.args.return_start:
                try:
                    before = self._state_stamp()
                    self.logger.warn("画球结束，MoveJ 回到本次起始关节位姿")
                    self._move_to_joints(initial_joints, dof)
                    self._wait_state_change(before)
                    metrics["return_start"] = True
                except Exception as exc:
                    self.logger.error(f"回到起始位姿失败: {exc}")
                    metrics["return_start_error"] = str(exc)
            if executed and self.args.record_start:
                try:
                    self._write_start_pose(
                        Path(self.args.start_pose_file),
                        initial_joints,
                        initial_pose,
                        initial_arm_angle,
                    )
                    self.logger.info(f"起始位姿已记录: {self.args.start_pose_file}")
                    metrics["start_pose_file"] = self.args.start_pose_file
                except Exception as exc:
                    self.logger.warn(f"记录起始位姿失败: {exc}")

            metrics["executed"] = executed
            self._write_outputs(path, solutions, ok, latency_ms, messages, fk_poses, metrics)
            return True
        except KeyboardInterrupt:
            self.logger.warn("收到 Ctrl-C，正在停止")
            return False
        finally:
            self._stop = True

    def _auto_select_half(
        self,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        arm_angle: float,
        dof: int,
    ):
        """从灵巧空间体素数据自动选上半球球心。

        只在「路径采样点都落在可达体积内部」的体素中心里选，按灵巧度排序，
        再用控制器 IK 粗校验成功率、相邻解跳变与关节软限位；第一个完全通过
        的候选直接采用，否则取评分最优者并告警。返回 None 表示数据不可用或
        没有候选，调用方回退到当前 TCP + R·x。
        """
        args = self.args
        data_path = Path(args.workspace_data)
        if not data_path.is_file():
            self.logger.warn(f"工作空间数据不存在: {data_path}；回退到当前 TCP + R·x")
            return None
        try:
            workspace = load_workspace_data(data_path)
        except Exception as exc:
            self.logger.warn(f"读取工作空间数据失败: {exc}；回退到当前 TCP + R·x")
            return None

        driver_arm = self._driver_arm_type()
        if workspace.arm_type and driver_arm and workspace.arm_type.lower() != driver_arm.lower():
            message = (
                f"工作空间数据臂型 {workspace.arm_type} 与 tl_driver arm_type "
                f"{driver_arm} 不一致"
            )
            if not args.allow_arm_mismatch:
                raise RuntimeError(
                    message
                    + "；请用 --workspace-data 指向对应臂型的数据，或加 --allow-arm-mismatch"
                )
            self.logger.warn(message + "（--allow-arm-mismatch 已启用）")

        candidates = half_sphere_candidates(
            workspace,
            radius=args.radius,
            tcp_position=current_pose[:3],
            max_start_gap=args.auto_start_gap,
            limit=args.auto_candidates,
            verify_points=args.auto_verify_samples,
            orient=args.orient,
            base_rpy=current_pose[3:],
        )
        if not candidates:
            self.logger.warn(
                f"灵巧空间内没有满足 R={args.radius:.3f} m、首点偏差 ≤ "
                f"{args.auto_start_gap:.2f} m 的上半球球心；回退到当前 TCP + R·x"
            )
            return None
        self.logger.info(
            f"灵巧空间候选 {len(candidates)} 个（数据: {data_path.name}, "
            f"臂型 {workspace.arm_type or '未知'}, 体素 {workspace.step:.3f} m），"
            "开始控制器 IK 粗校验"
        )

        probe_limits = self._joint_limits(dof) if args.check_limits else None
        e1 = np.array([1.0, 0.0, 0.0])
        e2 = np.array([0.0, 1.0, 0.0])
        e3 = np.array([0.0, 0.0, 1.0])
        probe_step = max(
            2.5 * math.pi * args.radius / max(int(args.auto_verify_samples), 8),
            1e-5,
        )
        best = None
        best_score = None
        chosen_index = 0
        probes = []
        for index, candidate in enumerate(candidates):
            probe_path = build_sphere_path(
                center=candidate.center,
                radius=candidate.radius,
                e1=e1,
                e2=e2,
                e3=e3,
                step=probe_step,
                orient=args.orient,
                base_rpy=current_pose[3:],
            )
            if args.roll_offset_deg is not None and args.orient in ("radial", "inward"):
                probe_path = probe_path.with_rpys(
                    [rotate_rpy_about_z(rpy, args.roll_offset_deg) for rpy in probe_path.rpys]
                )
            probe_solutions, probe_ok, _, _, _, _, _ = self._solve_path(
                probe_path,
                current_joints,
                arm_angle,
                dof,
                probe_limits,
                current_pose[3:],
                check_fk=False,
                search_roll=False,
                verbose=False,
            )
            if self._stop:
                return None
            success = float(probe_ok.mean()) if len(probe_ok) else 0.0
            steps = [
                float(np.max(np.abs(probe_solutions[i] - probe_solutions[i - 1])))
                for i in range(1, len(probe_ok))
                if probe_ok[i] and probe_ok[i - 1]
            ]
            max_step = max(steps) if steps else 0.0
            violations = 0
            if probe_limits is not None:
                low, high = probe_limits
                valid = probe_ok & np.isfinite(probe_solutions).all(axis=1)
                if valid.any():
                    below = probe_solutions[valid] < (low - JOINT_LIMIT_MARGIN_RAD)
                    above = probe_solutions[valid] > (high + JOINT_LIMIT_MARGIN_RAD)
                    violations = int(below.any(axis=1).sum() + above.any(axis=1).sum())
            clean = success >= 1.0 and max_step <= PROBE_MAX_STEP_RAD and violations == 0
            probes.append(
                {
                    "index": index,
                    "center_m": candidate.center.tolist(),
                    "start_gap_m": candidate.start_gap,
                    "min_dexterity": candidate.min_dexterity,
                    "mean_dexterity": candidate.mean_dexterity,
                    "probe_success": success,
                    "probe_max_step_rad": max_step,
                    "probe_limit_violations": violations,
                    "clean": clean,
                }
            )
            score = (
                1.0 if clean else 0.0,
                success,
                -violations,
                -max_step,
                candidate.min_dexterity,
            )
            if best_score is None or score > best_score:
                best = candidate
                best_score = score
                chosen_index = index

        if best is None:
            return None
        # 粗校验只做候选排序；最终必须用**全量细采样**逆解确认，粗采样会漏掉
        # 1 rad 级换支（实测 64 点粗校验通过、392 点全量出现 1.3 rad 跳变）。
        order = sorted(
            range(len(probes)),
            key=lambda index: (
                probes[index]["clean"],
                probes[index]["probe_success"],
                -probes[index]["probe_limit_violations"],
                -probes[index]["probe_max_step_rad"],
                probes[index]["min_dexterity"],
            ),
            reverse=True,
        )
        full_reports = []
        selected = None
        max_full = max(1, int(getattr(args, "auto_full_candidates", 4)))
        for candidate_index in order[:max_full]:
            candidate = candidates[candidate_index]
            fine_path = build_sphere_path(
                center=candidate.center,
                radius=candidate.radius,
                e1=e1,
                e2=e2,
                e3=e3,
                step=args.sample_step,
                orient=args.orient,
                base_rpy=current_pose[3:],
            )
            self.logger.info(
                f"全量校验候选 #{candidate_index}：球心={np.round(candidate.center, 4).tolist()} m，"
                f"{len(fine_path.positions)} 点"
            )
            (
                full_solutions,
                full_ok,
                full_latency,
                full_messages,
                full_fk,
                full_rpys,
                full_calls,
            ) = self._solve_path(
                fine_path,
                current_joints,
                arm_angle,
                dof,
                probe_limits,
                current_pose[3:],
                verbose=False,
            )
            if self._stop:
                return None
            success = float(full_ok.mean()) if len(full_ok) else 0.0
            steps = [
                float(np.max(np.abs(full_solutions[i] - full_solutions[i - 1])))
                for i in range(1, len(full_ok))
                if full_ok[i] and full_ok[i - 1]
            ]
            max_step = max(steps) if steps else 0.0
            violations = 0
            if probe_limits is not None:
                low, high = probe_limits
                valid = full_ok & np.isfinite(full_solutions).all(axis=1)
                if valid.any():
                    below = full_solutions[valid] < (low - JOINT_LIMIT_MARGIN_RAD)
                    above = full_solutions[valid] > (high + JOINT_LIMIT_MARGIN_RAD)
                    violations = int(below.any(axis=1).sum() + above.any(axis=1).sum())
            passed = success >= 1.0 and max_step <= float(args.auto_max_step) and violations == 0
            full_reports.append(
                {
                    "index": candidate_index,
                    "success": success,
                    "max_step_rad": max_step,
                    "limit_violations": violations,
                    "passed": passed,
                }
            )
            self.logger.info(
                f"全量校验候选 #{candidate_index}：成功率 {success * 100:.0f}%，"
                f"max|dq|={max_step:.3f} rad，限位越界 {violations} → "
                f"{'通过' if passed else '拒绝'}"
            )
            if passed:
                selected = (
                    candidate,
                    candidate_index,
                    fine_path,
                    full_solutions,
                    full_ok,
                    full_latency,
                    full_messages,
                    full_fk,
                    full_rpys,
                    full_calls,
                )
                self._validated = {
                    "center": candidate.center.copy(),
                    "radius": float(candidate.radius),
                    "solutions": full_solutions,
                    "ok": full_ok,
                    "latency_ms": full_latency,
                    "messages": full_messages,
                    "fk_poses": full_fk,
                    "chosen_rpys": full_rpys,
                    "calls": full_calls,
                }
                break

        if selected is not None:
            (
                best,
                chosen_index,
                _fine_path,
                _solutions,
                _ok,
                _latency,
                _messages,
                _fk,
                _rpys,
                _calls,
            ) = selected
            probe = probes[chosen_index]
            report = next(item for item in full_reports if item["index"] == chosen_index)
            self.logger.info(
                f"选中球心={np.round(best.center, 4).tolist()} m（候选 #{chosen_index}，"
                f"min_dex={best.min_dexterity:.3f}, 首点偏差={best.start_gap * 1000.0:.0f} mm, "
                f"全量校验 max|dq|={report['max_step_rad']:.3f} rad, "
                f"限位越界={report['limit_violations']}）"
            )
        else:
            probe = probes[chosen_index]
            self.logger.warn(
                f"全量校验的 {len(full_reports)} 个候选都不满足 "
                f"max|dq| ≤ {float(args.auto_max_step):.2f} rad / 100% 成功 / 无越界；"
                f"仍选粗校验最优的候选 #{chosen_index}，执行前预检会再次拦截"
            )
        selection = {
            "source": "dexterous-workspace",
            "workspace_data": str(data_path),
            "arm_type": workspace.arm_type,
            "voxel_step_m": workspace.step,
            "tau_rad": workspace.tau_rad,
            "center": best.center,
            "radius": best.radius,
            "min_dexterity": best.min_dexterity,
            "mean_dexterity": best.mean_dexterity,
            "start_gap_m": best.start_gap,
            "candidate_index": chosen_index,
            "candidate_probes": probes,
            "full_validation": full_reports,
        }
        return selection

    def _build_path(
        self,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        arm_angle: float,
        dof: int,
    ):
        """构造上半球线框：默认从灵巧空间自动选球心，也可显式指定。"""
        args = self.args
        position = current_pose[:3]
        rpy = current_pose[3:]
        # 极轴固定为基座 z：两圆所在平面与当前 TCP 姿态无关，
        # xz 面圆用 e1-e3，yz 面圆用 e2-e3。
        e1 = np.array([1.0, 0.0, 0.0])
        e2 = np.array([0.0, 1.0, 0.0])
        e3 = np.array([0.0, 0.0, 1.0])
        if args.center is not None:
            center = np.asarray(args.center, dtype=float)
            self._selection = {
                "source": "explicit-center",
                "center_m": center.tolist(),
                "radius_m": args.radius,
            }
        else:
            selection = None
            if args.auto_center:
                selection = self._auto_select_half(current_pose, current_joints, arm_angle, dof)
            if selection is not None:
                center = selection["center"]
                self._selection = selection
            else:
                # 首段弧从球心 - R·e1 起步，把该点放在当前 TCP 上
                center = position + args.radius * e1
                self._selection = {
                    "source": "fixed-relative",
                    "center_m": center.tolist(),
                    "radius_m": args.radius,
                }
                if args.auto_center:
                    self.logger.warn("自动选球不可用，回退到当前 TCP + R·x")

        path = build_sphere_path(
            center=center,
            radius=args.radius,
            e1=e1,
            e2=e2,
            e3=e3,
            step=args.sample_step,
            orient=args.orient,
            base_rpy=rpy,
        )
        first = path.positions[0]
        gap = float(np.linalg.norm(first - position))
        self._selection["start_gap_m"] = gap
        if args.center is not None and gap > args.max_start_gap:
            target = f"轨迹首点 {np.round(first, 4).tolist()} 距当前 TCP {gap:.3f} m"
            if args.blend_time <= 0.0:
                self.logger.warn(f"{target}，且 --blend-time 为 0，首帧直接跳到首点解")
            else:
                self.logger.warn(f"{target}，blend 段（{args.blend_time:.2f}s）会先做关节过渡")
        self.logger.info(
            f"关节数 {dof}；轨迹首点与当前 TCP 距离 {np.linalg.norm(path.positions[0] - position):.4f} m"
        )
        return path, rpy

    # ------------------------------------------------------------------
    # 逆解扫描
    # ------------------------------------------------------------------

    def _fk_pose(self, joints: np.ndarray) -> np.ndarray | None:
        """用 /tl_driver/fk 回代关节角，返回 6 维位姿（m/rad）；失败返回 None。"""
        request = self._Fk.Request()
        request.frame = 1
        request.joints = joints.tolist()
        request.source = 0
        try:
            response = self._call(self.fk_cli, request, "fk")
        except Exception as exc:
            if self._stop:
                raise
            self.logger.warn(f"fk 失败: {exc}")
            return None
        if response is None or not response.success:
            return None
        return np.array(
            [
                response.pose.position.x,
                response.pose.position.y,
                response.pose.position.z,
                response.pose.rpy.x,
                response.pose.rpy.y,
                response.pose.rpy.z,
            ],
            dtype=float,
        )

    def _ik_candidate(
        self,
        position: np.ndarray,
        rpy: np.ndarray,
        reference: np.ndarray,
        arm_angle: float,
        dof: int,
    ):
        """对单个候选姿态调用一次控制器逆解，返回 (关节角|None, 耗时 ms, 失败原因)。"""
        request = self._Ik.Request()
        request.frame = 1
        request.pose.header.frame_id = "base_link"
        request.pose.position.x = float(position[0])
        request.pose.position.y = float(position[1])
        request.pose.position.z = float(position[2])
        request.pose.rpy.x = float(rpy[0])
        request.pose.rpy.y = float(rpy[1])
        request.pose.rpy.z = float(rpy[2])
        if self.args.ik_source == 1 and dof == 7:
            # 本地 7 轴逆解把 arm_angle 当 ψ 硬约束，NaN 表示不指定
            request.pose.arm_angle = float("nan")
        else:
            request.pose.arm_angle = arm_angle
        request.form = int(self.args.form)
        request.reference_pos = reference.tolist()
        request.source = int(self.args.ik_source)

        start = time.perf_counter()
        try:
            response = self._call(self.ik_cli, request, "ik")
        except Exception as exc:  # 单点失败不应中断整条扫描
            if self._stop:
                raise
            return None, (time.perf_counter() - start) * 1000.0, str(exc)
        latency = (time.perf_counter() - start) * 1000.0
        if response is None or not response.success:
            return None, latency, response.message if response is not None else "无响应"
        joints = np.asarray(response.joints[:dof], dtype=float)
        if not np.all(np.isfinite(joints)):
            return None, latency, "ik 返回非有限值"
        return joints, latency, ""

    def _solve_path(
        self,
        path: SpherePath,
        current_joints: np.ndarray,
        arm_angle: float,
        dof: int,
        limits,
        roll_reference: np.ndarray,
        check_fk: bool | None = None,
        search_roll: bool | None = None,
        verbose: bool = True,
    ):
        """逐点逆解；radial/inward 下把自由 roll 交给控制器挑。

        `check_fk` / `search_roll` 可覆盖命令行默认值，`verbose=False` 关闭逐点
        失败日志与进度输出——自动选球的粗校验用这组参数做快速扫描。

        每个候选姿态都用 FK 回代确认控制器真的达到了请求的 tool Z 与位置，并检查
        关节软限位——控制器在万向锁点上会「报 success 但姿态错」，也会直接返回
        超出软限位的解。tool X 已随路径平行移动、全程连续，roll 只预选一次并在
        全程沿用（只在硬失败时才回退到按上一帧距离排序的候选），接缝处不再换 roll。

        返回 solutions/ok/latency/messages/fk_poses/chosen_rpys/calls；chosen_rpys
        是每点实际请求并通过验证的姿态，调用方用它替换 path.rpys。
        """
        count = len(path.positions)
        solutions = np.full((count, dof), np.nan)
        latency_ms = np.zeros(count)
        calls = np.zeros(count, dtype=int)
        messages = [""] * count
        fk_poses = np.full((count, 6), np.nan)
        chosen_rpys = np.asarray(path.rpys, dtype=float).copy()
        reference = current_joints.copy()
        previous_rpy = np.asarray(roll_reference, dtype=float)
        if search_roll is None:
            search_roll = self.args.roll_search and self.args.orient in ("radial", "inward")
        if check_fk is None:
            check_fk = self.args.check_fk
        verify_fk = search_roll or check_fk
        margin = math.radians(max(float(self.args.min_joint_margin), 0.0))
        preferred_offset = None
        preferred_fail_streak = 0
        self._roll_offset_deg = None
        forced_offset = getattr(self.args, "roll_offset_deg", None)
        if search_roll and forced_offset is not None:
            preferred_offset = float(forced_offset)
            self._roll_offset_deg = preferred_offset
            self.logger.info(f"强制全程 roll 偏移 {preferred_offset:+.1f}°（--roll-offset-deg）")
        elif search_roll and limits is not None:
            preferred_offset = self._choose_roll(
                path, current_joints, arm_angle, dof, limits, margin
            )
            self._roll_offset_deg = preferred_offset
            if preferred_offset is not None:
                self.logger.info(
                    f"全程 roll 偏移 {preferred_offset:+.1f}°：姿态连续，接缝处不再换 roll"
                )

        for index in range(count):
            if self._stop:
                break
            if search_roll:
                candidates = order_roll_candidates(
                    path.rpys[index], previous_rpy, self.args.roll_step_deg
                )
                preferred = None
                if preferred_offset is not None:
                    preferred = rotate_rpy_about_z(path.rpys[index], preferred_offset)
                    if preferred_fail_streak < self.args.roll_switch_after:
                        # 单点失败（IK 偶发失败/该点该 roll 超限位）不值得换滚转相位：
                        # 换一次就是整腕重构（实测 45° roll 换来 1.96 rad 关节跳变），
                        # 宁可让该点空过去（--force 下保持上一关节）也不甩腕。
                        candidates = [preferred]
                    else:
                        candidates = [preferred] + [
                            c for c in candidates if rotation_error_deg(c, preferred) > 1e-6
                        ]
            else:
                candidates = [np.asarray(path.rpys[index], dtype=float)]
                preferred = None
            accepted = False
            fallback = None
            for candidate in candidates:
                joints, latency, message = self._ik_candidate(
                    path.positions[index], candidate, reference, arm_angle, dof
                )
                latency_ms[index] += latency
                calls[index] += 1
                if joints is None:
                    messages[index] = message
                    continue
                room = math.inf
                if limits is not None:
                    if np.any(joints < limits[0] - 1e-9) or np.any(joints > limits[1] + 1e-9):
                        messages[index] = "逆解超出关节软限位"
                        continue
                    room = float(min(np.min(joints - limits[0]), np.min(limits[1] - joints)))
                fk_pose = self._fk_pose(joints) if verify_fk else None
                if verify_fk and fk_pose is None:
                    messages[index] = "FK 回代失败"
                    continue
                if search_roll:
                    if (
                        np.linalg.norm(fk_pose[:3] - path.positions[index]) * 1000.0
                        > self.args.max_residual_mm
                    ):
                        messages[index] = "FK 位置残差超限"
                        continue
                    # roll 本身是自由量，只校验 tool Z 指向（万向锁点上 rpy 无法表达任意 roll）
                    z_error = tool_z_error_deg(fk_pose[3:], candidate)
                    if z_error > self.args.max_residual_deg:
                        messages[index] = f"FK tool Z 偏差 {z_error:.1f}°（控制器未达到请求指向）"
                        continue
                # 贴限位的解会在后续点把 IK 逼到换分支（实测 J6 64.7°/65° 时整段翻转），
                # 有别的 roll 可选时优先留余量；都不满足时退回这个可行解。
                # 已按整段预选过 roll 时不再为余量改 roll——那样会在段中间换分支，
                # 实测 45° 的 roll 改动换来 171° 关节重构、TCP 离球面 9 mm。
                if search_roll and room < margin and preferred is None:
                    if fallback is None:
                        fallback = (joints, fk_pose, candidate)
                    messages[index] = (
                        f"贴关节软限位（余量 {math.degrees(room):.1f}° < "
                        f"{math.degrees(margin):.1f}°）"
                    )
                    continue
                solutions[index] = joints
                if fk_pose is not None:
                    fk_poses[index] = fk_pose
                chosen_rpys[index] = candidate
                previous_rpy = candidate
                reference = joints
                messages[index] = ""
                accepted = True
                break
            # 预选 roll 的连续失败计数：只有连续失败够多才允许换滚转相位
            if accepted:
                kept = preferred is not None and (
                    rotation_error_deg(chosen_rpys[index], preferred) < 1e-6
                )
                preferred_fail_streak = 0 if kept else preferred_fail_streak + 1
            else:
                preferred_fail_streak += 1
            if not accepted and fallback is not None:
                joints, fk_pose, candidate = fallback
                solutions[index] = joints
                if fk_pose is not None:
                    fk_poses[index] = fk_pose
                chosen_rpys[index] = candidate
                previous_rpy = candidate
                reference = joints
                messages[index] = "无余量更优的 roll，沿用贴限位解"
                accepted = True
            if not accepted:
                if verbose:
                    self._log_failed_pose(index, count, path, messages[index])
                continue
            if verbose and ((index + 1) % 100 == 0 or index + 1 == count):
                print(f"\r逆解扫描 {index + 1}/{count}", end="", flush=True)
        if verbose:
            print()
        ok = np.isfinite(solutions).all(axis=1)
        if verbose:
            self.logger.info(
                f"逆解完成：成功 {int(ok.sum())}/{count}，IK 调用 {int(calls.sum())} 次"
                f"（roll 搜索{'开' if search_roll else '关'}）"
            )
        return solutions, ok, latency_ms, messages, fk_poses, chosen_rpys, calls

    def _choose_roll(self, path, current_joints, arm_angle, dof, limits, margin):
        """全程预选一个 roll 偏移（°）：整条轨迹只改一次滚转相位。

        tool X 已随路径平行移动、接缝连续，所以整条轨迹用同一个偏移即可，接缝不会
        出现 90° 级 roll 台阶（实测一次 135° 台阶要 3.61 rad 关节重构、TCP 离球面
        6 cm）。候选按 |偏移| 升序（0 = 最贴近当前 TCP 的滚转相位）：先用 |pitch|
        上限离线过滤，再沿整条轨迹抽样做逆解/限位/余量检查；全部候选都留不足余量
        时取余量最大者，返回 None 表示都不可行、交回逐点搜索。
        """
        count = len(path.positions)
        probe_count = min(ROLL_PRESCAN_POINTS, count)
        probes = sorted({(k * (count - 1)) // max(1, probe_count - 1) for k in range(probe_count)})
        best_offset = None
        best_key = None
        rejected = []
        for offset in roll_candidates(self.args.roll_step_deg):
            rotated = [rotate_rpy_about_z(rpy, offset) for rpy in path.rpys]
            if max(abs(pitch_deg(rpy)) for rpy in rotated) > PITCH_LIMIT_DEG:
                rejected.append(f"{offset:+.0f}°(pitch)")
                continue
            trial_reference = current_joints.copy()
            feasible = True
            room_min = math.inf
            reached = 0
            for index in probes:
                if self._stop:
                    return best_offset
                joints, _, _ = self._ik_candidate(
                    path.positions[index], rotated[index], trial_reference, arm_angle, dof
                )
                if joints is None:
                    feasible = False
                    rejected.append(f"{offset:+.0f}°(IK@{index})")
                    break
                if limits is not None and (
                    np.any(joints < limits[0] - 1e-9) or np.any(joints > limits[1] + 1e-9)
                ):
                    feasible = False
                    break
                fk_pose = self._fk_pose(joints)
                if fk_pose is None:
                    feasible = False
                    break
                if (
                    np.linalg.norm(fk_pose[:3] - path.positions[index]) * 1000.0
                    > self.args.max_residual_mm
                ):
                    feasible = False
                    break
                if tool_z_error_deg(fk_pose[3:], rotated[index]) > self.args.max_residual_deg:
                    feasible = False
                    break
                if limits is not None:
                    room_min = min(
                        room_min,
                        float(min(np.min(joints - limits[0]), np.min(limits[1] - joints))),
                    )
                trial_reference = joints
            if not feasible:
                continue
            reached = len(probes)
            if room_min >= margin:
                return offset
            key = (reached, room_min)
            if best_key is None or key > best_key:
                best_key = key
                best_offset = offset
        if best_offset is not None and best_key is not None:
            self.logger.warn(
                f"没有全程余量达标的 roll，取覆盖最广的 {best_offset:+.0f}°"
                f"（{best_key[0]}/{len(probes)} 个抽样点，最小余量 "
                f"{math.degrees(best_key[1]):.1f}°）"
            )
        elif rejected:
            self.logger.warn(f"所有 roll 候选不可行：{', '.join(rejected)}")
        return best_offset

    @staticmethod
    def _roll_offset_of(base_rpy, rolled_rpy) -> float:
        """求 rolled_rpy 相对 base_rpy 绕 tool Z 的偏移角（°）。"""
        rotation = rpy_to_matrix(base_rpy).T @ rpy_to_matrix(rolled_rpy)
        return math.degrees(math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))

    def _log_failed_pose(self, index: int, count: int, path: SpherePath, reason: str) -> None:
        """打印逆解失败的目标位姿（位置 m、rpy 弧度+角度），便于排查姿态或可达性。"""
        position = path.positions[index]
        rpy = path.rpys[index]
        degrees = np.degrees(rpy)
        self.logger.warn(
            f"逆解失败 {index + 1}/{count} [{path.segments[index]}] "
            f"pos=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) m "
            f"rpy=({rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}) rad "
            f"({degrees[0]:.1f}, {degrees[1]:.1f}, {degrees[2]:.1f})°"
            f" | {reason or '无返回信息'}"
        )

    # ------------------------------------------------------------------
    # 预检与指标
    # ------------------------------------------------------------------

    def _precheck(
        self,
        path,
        solutions,
        ok,
        latency_ms,
        messages,
        fk_poses,
        dof,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        timeline,
        exec_end: int,
        limits,
        calls,
    ) -> dict:
        count = len(solutions)
        metrics: dict = {"blocking": []}
        ik_metrics = {
            "samples": count,
            "success": int(ok.sum()),
            "fail": int(count - ok.sum()),
            "fail_indices": np.where(~ok)[0].tolist()[:20],
            "roll_offset_deg": getattr(self, "_roll_offset_deg", None),
            "executed_samples": int(exec_end),
            "coverage": float(exec_end) / count if count else 0.0,
            "roll_calls": {
                "total": int(np.sum(calls)),
                "max": int(np.max(calls)) if len(calls) else 0,
            },
        }
        finite_latency = latency_ms[(calls > 0) & np.isfinite(latency_ms)]
        if finite_latency.size:
            ik_metrics["latency_ms"] = {
                "median": float(np.median(finite_latency)),
                "p95": float(np.percentile(finite_latency, 95)),
                "max": float(np.max(finite_latency)),
                "mean": float(np.mean(finite_latency)),
            }

        # 首点跨度：blend 段在关节空间插值，跨度过大时过渡段 TCP 走线不受控
        start_gap = start_gap_metrics(path, current_pose, current_joints, solutions, ok)
        metrics["start_gap"] = start_gap
        if start_gap["position_m"] > self.args.max_start_gap:
            metrics["blocking"].append(
                f"首点距当前 TCP {start_gap['position_m'] * 1000.0:.1f} mm 超过 "
                f"{self.args.max_start_gap * 1000.0:.0f} mm（--max-start-gap）"
            )
        if start_gap["rotation_deg"] > self.args.max_start_rot:
            metrics["blocking"].append(
                f"首点姿态与当前 TCP 相差 {start_gap['rotation_deg']:.1f}° 超过 "
                f"{self.args.max_start_rot:.0f}°（--max-start-rot；"
                "radial/inward 按球面法向重定向）"
            )
        joint_gap = start_gap.get("joint_rad", 0.0)
        if joint_gap > self.args.max_joint_step:
            metrics["blocking"].append(
                f"首点逆解与当前关节相差 {joint_gap:.3f} rad 超过 "
                f"{self.args.max_joint_step:.3f} rad（--max-joint-step）"
            )
        elif timeline.kinds and timeline.kinds[0] == "blend_in":
            blend = float(timeline.times[1] - timeline.times[0])
            needed = joint_gap / blend if blend > 0.0 else math.inf
            if needed > math.radians(self.args.vmax_deg) * 1.001:
                metrics["blocking"].append(
                    f"首点在 blend 段需 {math.degrees(needed):.0f}°/s，超过 ServoJ "
                    f"--vmax-deg {self.args.vmax_deg:.0f}°/s（加大 --blend-time）"
                )

        # 关节连续性：只在连续两个成功点之间计算
        steps = np.full(count - 1, np.nan)
        for i in range(1, count):
            if ok[i] and ok[i - 1]:
                steps[i - 1] = float(np.max(np.abs(solutions[i] - solutions[i - 1])))
        if np.isfinite(steps).any():
            jump_indices = np.where(steps > self.args.jump_threshold)[0] + 1
            ik_metrics["max_step_rad"] = float(np.nanmax(steps))
            ik_metrics["max_step_index"] = int(np.nanargmax(steps) + 1)
            ik_metrics["jump_count"] = int(jump_indices.size)
            ik_metrics["jump_indices"] = jump_indices.tolist()[:50]
        else:
            ik_metrics["max_step_rad"] = 0.0
            ik_metrics["jump_count"] = 0
            ik_metrics["jump_indices"] = []

        metrics["ik"] = ik_metrics
        if ik_metrics["fail"] > 0:
            metrics["blocking"].append(
                f"{ik_metrics['fail']} 个采样点逆解失败（前 20 个索引 "
                f"{ik_metrics['fail_indices']}）"
            )
        if ik_metrics.get("max_step_rad", 0.0) > self.args.max_joint_step:
            metrics["blocking"].append(
                f"相邻解最大关节变化 {ik_metrics['max_step_rad']:.3f} rad "
                f"超过 {self.args.max_joint_step:.3f} rad；"
                "如为弧段切换，加 --allow-jumps 做平滑过渡"
            )
        if ik_metrics.get("jump_count", 0) > 0 and not self.args.allow_jumps:
            metrics["blocking"].append(
                f"{ik_metrics['jump_count']} 处关节跳变超过 {self.args.jump_threshold:.3f} rad，"
                "如需平滑过渡请加 --allow-jumps"
            )

        # 时间轴速度/加速度：按实际下发频率离散采样，与控制器看到的一致
        timeline_limits = self._timeline_limits(timeline)
        metrics["timeline"] = timeline_limits
        if timeline_limits["v_peak_rad_s"] > math.radians(self.args.vmax_deg) * 1.001:
            metrics["blocking"].append(
                f"轨迹关节速度峰值 {timeline_limits['v_peak_deg_s']:.0f}°/s 超过 "
                f"--vmax-deg {self.args.vmax_deg:.0f}°/s（降低 --speed 或加大 --ramp-time）"
            )
        if timeline_limits["a_peak_rad_s2"] > math.radians(self.args.amax_deg) * 1.001:
            metrics["blocking"].append(
                f"轨迹关节加速度峰值 {timeline_limits['a_peak_deg_s2']:.0f}°/s² 超过 "
                f"--amax-deg {self.args.amax_deg:.0f}°/s²（加大 --ramp-time 或降低 --speed）"
            )

        # FK 残差
        if self.args.check_fk:
            fk_ok = np.isfinite(fk_poses).all(axis=1)
            if fk_ok.any():
                residual_pos = (
                    np.linalg.norm(fk_poses[fk_ok, :3] - path.positions[fk_ok], axis=1) * 1000.0
                )
                # fixed 模式比较整姿态；radial/inward 的 roll 是自由量，只比较 tool Z
                rot_metric = rotation_error_deg if self.args.orient == "fixed" else tool_z_error_deg
                residual_rot = np.array(
                    [rot_metric(fk_poses[i, 3:], path.rpys[i]) for i in np.where(fk_ok)[0]]
                )
                metrics["fk"] = {
                    "samples": int(fk_ok.sum()),
                    "rot_metric": "orientation" if self.args.orient == "fixed" else "tool_z",
                    "pos_mm": {
                        "mean": float(np.mean(residual_pos)),
                        "p95": float(np.percentile(residual_pos, 95)),
                        "max": float(np.max(residual_pos)),
                    },
                    "rot_deg": {
                        "mean": float(np.mean(residual_rot)),
                        "max": float(np.max(residual_rot)),
                    },
                }
                if metrics["fk"]["pos_mm"]["max"] > self.args.max_residual_mm:
                    metrics["blocking"].append(
                        f"FK 位置残差最大 {metrics['fk']['pos_mm']['max']:.2f} mm "
                        f"超过 {self.args.max_residual_mm:.2f} mm"
                    )
                if metrics["fk"]["rot_deg"]["max"] > self.args.max_residual_deg:
                    rot_label = "姿态" if metrics["fk"]["rot_metric"] == "orientation" else "tool Z"
                    metrics["blocking"].append(
                        f"FK {rot_label}残差最大 {metrics['fk']['rot_deg']['max']:.2f}° "
                        f"超过 {self.args.max_residual_deg:.2f}°"
                    )
            else:
                metrics["fk"] = {"samples": 0}

        # 关节限位（逆解阶段已拒绝超限解，这里只对历史数据/关闭搜索时的结果复核）
        if self.args.check_limits and limits is not None:
            low, high = limits
            valid = ok & np.isfinite(solutions).all(axis=1)
            violations = []
            if valid.any():
                below = solutions[valid] < (low - DEFAULT_MIN_JOINT_MARGIN)
                above = solutions[valid] > (high + DEFAULT_MIN_JOINT_MARGIN)
                for axis in range(dof):
                    if below[:, axis].any() or above[:, axis].any():
                        count_axis = int(below[:, axis].sum() + above[:, axis].sum())
                        violations.append(f"J{axis + 1}:{count_axis}")
            metrics["limits"] = {
                "low_rad": low.tolist(),
                "high_rad": high.tolist(),
                "violations": violations,
            }
            if violations:
                metrics["blocking"].append("逆解超出关节软限位：" + ", ".join(violations))
            if valid.any():
                # 贴限位时控制器会直接逆解失败（实测 J7 剩 0.5° 时后续点全 EXCEPTION）
                margin = np.minimum(solutions[valid] - low, high - solutions[valid])
                per_axis = np.min(margin, axis=0)
                min_margin = float(np.min(per_axis))
                min_axis = int(np.argmin(per_axis))
                metrics["limits"]["min_margin_rad"] = min_margin
                metrics["limits"]["min_margin_joint"] = min_axis + 1
                if min_margin < DEFAULT_MIN_JOINT_MARGIN:
                    metrics["blocking"].append(
                        f"J{min_axis + 1} 距软限位只剩 {math.degrees(min_margin):.1f}°，"
                        f"低于 {math.degrees(DEFAULT_MIN_JOINT_MARGIN):.1f}° 余量；"
                        "继续走会逆解失败"
                    )
        return metrics

    def _timeline_limits(self, timeline) -> dict:
        """按下发频率离散采样时间轴，给出各关节速度/加速度峰值。"""
        rate = self.args.rate
        if timeline.total <= 0.0 or rate <= 0.0:
            return {
                "samples": 0,
                "v_peak_rad_s": 0.0,
                "v_peak_deg_s": 0.0,
                "a_peak_rad_s2": 0.0,
                "a_peak_deg_s2": 0.0,
            }
        times = np.arange(0.0, timeline.total + 0.5 / rate, 1.0 / rate)
        joints = np.array([timeline.sample(t)[0] for t in times])
        velocity = np.diff(joints, axis=0) * rate
        acceleration = np.diff(velocity, axis=0) * rate
        v_peak = float(np.max(np.abs(velocity))) if velocity.size else 0.0
        a_peak = float(np.max(np.abs(acceleration))) if acceleration.size else 0.0
        return {
            "samples": int(len(times)),
            "v_peak_rad_s": v_peak,
            "v_peak_deg_s": math.degrees(v_peak),
            "a_peak_rad_s2": a_peak,
            "a_peak_deg_s2": math.degrees(a_peak),
        }

    def _joint_limits(self, dof: int):
        low = np.full(dof, np.nan)
        high = np.full(dof, np.nan)
        for axis in range(1, dof + 1):
            request = self._GetRobotJointParam.Request()
            request.id = axis
            try:
                response = self._call(self.joint_param_cli, request, "get_robot_joint_param")
            except Exception as exc:
                self.logger.warn(f"读取 J{axis} 关节参数失败: {exc}")
                continue
            if response is None or not response.success:
                continue
            low[axis - 1] = response.param.neg_sw_limit
            high[axis - 1] = response.param.pos_sw_limit
        if not np.all(np.isfinite(low)) or not np.all(np.isfinite(high)):
            self.logger.warn("关节限位不完整，跳过限位检查")
            return None
        return low, high

    # ------------------------------------------------------------------
    # 执行
    # ------------------------------------------------------------------

    def _execute(self, path, solutions, timeline, current_joints, dof) -> None:
        args = self.args
        old_speed = None
        if args.speed_percent > 0.0:
            try:
                speed_response = self._call(
                    self.get_speed_cli, self._GetSpeed.Request(), "get_speed"
                )
                if speed_response is not None and speed_response.success:
                    old_speed = float(speed_response.speed)
                set_request = self._SetSpeed.Request()
                set_request.speed = float(args.speed_percent)
                self._call(self.set_speed_cli, set_request, "set_speed")
                self.logger.info(f"运行速度设为 {args.speed_percent:.0f}%")
            except Exception as exc:
                self.logger.warn(f"设置运行速度失败（继续）: {exc}")

        open_request = self._OpenServoJ.Request()
        vmax = math.radians(args.vmax_deg)
        amax = math.radians(args.amax_deg)
        jmax = math.radians(args.jmax_deg)
        open_request.vmax = [vmax] * dof
        open_request.amax = [amax] * dof
        open_request.jmax = [jmax] * dof
        opened = False
        try:
            response = self._call(self.open_cli, open_request, "open_servoj")
            if response is None or not response.success:
                raise RuntimeError(
                    "open_servoj 失败: " + (response.message if response else "无响应")
                )
            opened = True

            self._cmd_rec.clear()
            self._tcp_rec.clear()
            self._joint_rec.clear()
            self._errors.clear()
            self._recording = True
            started = time.time()
            start_monotonic = time.monotonic()
            period = 1.0 / args.rate
            next_tick = start_monotonic
            total_duration = timeline.total + args.hold_time
            from std_msgs.msg import Float64MultiArray

            window_start = start_monotonic
            window_frames = 0
            window_max_lag = 0.0
            window_missed = 0
            sent_frames = 0
            last_joints = current_joints.copy()
            self.logger.info(
                f"开始 ServoJ 流式下发：{timeline.total:.2f}s 轨迹 + {args.hold_time:.2f}s 保持, "
                f"{args.rate:.0f} Hz"
            )
            while True:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                    now = time.monotonic()
                elapsed = now - start_monotonic
                if elapsed > total_duration:
                    break
                if self._stop:
                    self.logger.warn("收到停止请求，提前结束")
                    break
                if args.stop_on_error and self._errors:
                    self.logger.error(f"收到控制器消息，停止下发：{self._errors[-1][3]}")
                    break
                if elapsed <= timeline.total:
                    joints, _, _, _ = timeline.sample(elapsed)
                else:
                    joints = solutions[-1]
                if np.all(np.isfinite(joints)):
                    message = Float64MultiArray()
                    message.data = [float(value) for value in joints]
                    self.servoj_pub.publish(message)
                    self._cmd_rec.append((time.time(), np.asarray(joints, dtype=float)))
                    last_joints = np.asarray(joints, dtype=float)
                    sent_frames += 1
                    window_frames += 1
                now = time.monotonic()
                lag = now - next_tick
                window_max_lag = max(window_max_lag, lag)
                if lag > period:
                    # 落后一个以上节拍（调度抖动/休眠）：轨迹时钟一起顺延，绝不跳过
                    # 轨迹点——单帧跳过 1 s 会让关节步长超过 vmax，控制器直接跟不上
                    window_missed += int(lag // period)
                    start_monotonic += lag
                    next_tick = now + period
                else:
                    next_tick += period
                span = now - window_start
                if span >= 2.0:
                    achieved = window_frames / span
                    if achieved < 0.95 * args.rate:
                        self.logger.warn(
                            f"最近 {span:.1f}s 实际 {achieved:.1f} Hz（目标 {args.rate:.0f} Hz），"
                            f"最大滞后 {window_max_lag * 1000.0:.1f} ms，"
                            f"延后 {window_missed} 个节拍"
                        )
                    window_start = now
                    window_frames = 0
                    window_max_lag = 0.0
                    window_missed = 0
            # 保持结束后补一帧当前目标，避免最后一帧抖动
            message = Float64MultiArray()
            message.data = [float(value) for value in last_joints]
            self.servoj_pub.publish(message)
            total_elapsed = max(time.monotonic() - start_monotonic, 1e-6)
            self.logger.info(
                f"下发完成：{sent_frames} 帧，平均 {sent_frames / total_elapsed:.1f} Hz"
            )
            # 末帧只是目标值，控制器仍需时间跟随；延时期间保持 _recording，
            # 让 actual_tcp 采集到收尾段，再在 finally 里 close_servoj
            self.logger.info(f"等待 {DEFAULT_SERVOJ_SETTLE_S:.1f}s 让 ServoJ 跟随到位")
            settle_deadline = time.monotonic() + DEFAULT_SERVOJ_SETTLE_S
            while not self._stop:
                remaining = settle_deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.05, remaining))
        finally:
            self._recording = False
            if opened:
                try:
                    close_response = self._call(
                        self.close_cli,
                        self._Trigger.Request(),
                        "close_servoj",
                        ignore_stop=True,
                    )
                    if close_response is None or not close_response.success:
                        self.logger.error("close_servoj 失败，请检查控制器状态")
                except Exception as exc:
                    self.logger.error(f"close_servoj 异常: {exc}")
            if old_speed is not None:
                try:
                    set_request = self._SetSpeed.Request()
                    set_request.speed = old_speed
                    self._call(self.set_speed_cli, set_request, "restore_speed", ignore_stop=True)
                except Exception as exc:
                    self.logger.warn(f"恢复运行速度失败: {exc}")

    # ------------------------------------------------------------------
    # 实采指标
    # ------------------------------------------------------------------

    def _tracking_metrics(self, path, timeline, dof) -> dict:
        result: dict = {
            "command_frames": len(self._cmd_rec),
            "tcp_frames": len(self._tcp_rec),
            "joint_frames": len(self._joint_rec),
            "controller_messages": [
                {"type": m[1], "code": m[2], "message": m[3]} for m in self._errors[:20]
            ],
        }
        if not self._cmd_rec:
            return result
        started = self._cmd_rec[0][0]

        # 实际 TCP 与计划轨迹的误差（跳过 blend 段）
        if self._tcp_rec:
            position_errors = []
            orientation_errors = []
            radius_errors = []
            path_points = []
            skipped = 0
            for stamp, pose in self._tcp_rec:
                _, planned, planned_rpy, kind = timeline.sample(stamp - started)
                if kind.startswith("blend") or kind == "jump":
                    skipped += 1
                    continue
                position_errors.append(float(np.linalg.norm(pose[:3] - planned)))
                orientation_errors.append(rotation_error_deg(pose[3:], planned_rpy))
                radius_errors.append(
                    abs(float(np.linalg.norm(pose[:3] - path.center)) - path.radius)
                )
                path_points.append(pose[:3])
            if position_errors:
                position_errors = np.asarray(position_errors)
                radius_errors = np.asarray(radius_errors)
                result["tcp_error_mm"] = {
                    "samples": int(position_errors.size),
                    "rms": float(np.sqrt(np.mean(position_errors**2)) * 1000.0),
                    "max": float(np.max(position_errors) * 1000.0),
                }
                orientation_errors = np.asarray(orientation_errors)
                result["orientation_error_deg"] = {
                    "samples": int(orientation_errors.size),
                    "rms": float(np.sqrt(np.mean(orientation_errors**2))),
                    "max": float(np.max(orientation_errors)),
                }
                result["radius_error_mm"] = {
                    "rms": float(np.sqrt(np.mean(radius_errors**2)) * 1000.0),
                    "max": float(np.max(radius_errors) * 1000.0),
                }
            result["blend_samples_skipped"] = skipped

            # 实际点云到理论轨迹的最近距离（不依赖时间对齐；同样排除 blend 段）
            if path_points:
                nearest = self._nearest_path_distance(np.asarray(path_points), path.positions)
                result["nearest_path_mm"] = {
                    "rms": float(np.sqrt(np.mean(nearest**2)) * 1000.0),
                    "max": float(np.max(nearest) * 1000.0),
                }

        # 实际关节与指令关节的误差
        joint_errors = []
        skipped_joint = 0
        for stamp, joints in self._joint_rec:
            commanded, _, _, kind = timeline.sample(stamp - started)
            if kind.startswith("blend") or kind == "jump":
                skipped_joint += 1
                continue
            if joints.shape[0] < dof:
                continue
            joint_errors.append(float(np.max(np.abs(joints[:dof] - commanded))))
        if joint_errors:
            joint_errors = np.asarray(joint_errors)
            result["joint_error_rad"] = {
                "samples": int(joint_errors.size),
                "rms": float(np.sqrt(np.mean(joint_errors**2))),
                "max": float(np.max(joint_errors)),
            }
        result["blend_joint_samples_skipped"] = skipped_joint
        return result

    @staticmethod
    def _nearest_path_distance(points: np.ndarray, path_points: np.ndarray) -> np.ndarray:
        """逐点最近距离，分块避免临时矩阵过大。"""
        nearest = np.empty(points.shape[0], dtype=float)
        chunk = 512
        for start in range(0, points.shape[0], chunk):
            block = points[start : start + chunk]
            diff = block[:, None, :] - path_points[None, :, :]
            nearest[start : start + chunk] = np.sqrt(np.min(np.sum(diff * diff, axis=2), axis=1))
        return nearest

    # ------------------------------------------------------------------
    # 结果输出
    # ------------------------------------------------------------------

    def _output_dir(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        out = Path(self.args.out_dir).expanduser().resolve() / stamp
        out.mkdir(parents=True, exist_ok=True)
        return out

    def _write_outputs(self, path, solutions, ok, latency_ms, messages, fk_poses, metrics) -> None:
        out = self._output_dir()
        # 点云
        write_pcd(
            out / "target_path.pcd",
            path.positions,
            path.segment_ids(),
            "target sphere arcs; intensity=segment id",
        )
        write_ply(
            out / "target_path.ply",
            path.positions,
            path.segment_ids(),
            "target sphere arcs; intensity=segment id",
        )

        fk_valid = np.isfinite(fk_poses).all(axis=1)
        if fk_valid.any():
            residual_mm = (
                np.linalg.norm(fk_poses[fk_valid, :3] - path.positions[fk_valid], axis=1) * 1000.0
            )
            fk_points = fk_poses[fk_valid, :3]
            write_pcd(
                out / "ik_fk.pcd",
                fk_points,
                residual_mm,
                "controller IK solutions forward-kinematics; intensity=residual mm",
            )
            write_ply(
                out / "ik_fk.ply",
                fk_points,
                residual_mm,
                "controller IK solutions forward-kinematics; intensity=residual mm",
            )

        if self._tcp_rec:
            actual_points = np.asarray([pose[:3] for _, pose in self._tcp_rec])
            start = self._tcp_rec[0][0]
            times = np.asarray([stamp - start for stamp, _ in self._tcp_rec])
            write_pcd(
                out / "actual_tcp.pcd",
                actual_points,
                times,
                "measured TCP path; intensity=time s",
            )
            write_ply(
                out / "actual_tcp.ply",
                actual_points,
                times,
                "measured TCP path; intensity=time s",
            )

        # 采样明细 CSV
        header = (
            ["index", "segment", "s_m", "ok", "latency_ms", "message"]
            + [f"target_{name}" for name in ("x", "y", "z", "rx", "ry", "rz")]
            + [f"j{axis + 1}" for axis in range(solutions.shape[1])]
            + [f"fk_{name}" for name in ("x", "y", "z", "rx", "ry", "rz")]
            + ["residual_mm", "residual_deg"]
        )
        rows = []
        for index in range(len(path.positions)):
            residual_mm = ""
            residual_deg = ""
            if fk_valid[index]:
                residual_mm = float(
                    np.linalg.norm(fk_poses[index, :3] - path.positions[index]) * 1000.0
                )
                residual_deg = rotation_error_deg(fk_poses[index, 3:], path.rpys[index])
            rows.append(
                [
                    index,
                    path.segments[index],
                    float(path.s[index]),
                    int(ok[index]),
                    latency_ms[index],
                    messages[index],
                ]
                + path.positions[index].tolist()
                + path.rpys[index].tolist()
                + np.where(np.isfinite(solutions[index]), solutions[index], np.nan).tolist()
                + np.where(fk_valid[index], fk_poses[index], np.nan).tolist()
                + [residual_mm, residual_deg]
            )
        write_csv(out / "samples.csv", header, rows)

        if self._tcp_rec:
            write_csv(
                out / "actual_tcp.csv",
                ["t_s", "x", "y", "z", "rx", "ry", "rz"],
                [[stamp - self._tcp_rec[0][0]] + pose.tolist() for stamp, pose in self._tcp_rec],
            )
        if self._joint_rec:
            write_csv(
                out / "actual_joints.csv",
                ["t_s"] + [f"j{axis + 1}" for axis in range(self._joint_rec[0][1].shape[0])],
                [
                    [stamp - self._joint_rec[0][0]] + joints.tolist()
                    for stamp, joints in self._joint_rec
                ],
            )
        if self._cmd_rec:
            write_csv(
                out / "commands.csv",
                ["t_s"] + [f"j{axis + 1}" for axis in range(self._cmd_rec[0][1].shape[0])],
                [
                    [stamp - self._cmd_rec[0][0]] + joints.tolist()
                    for stamp, joints in self._cmd_rec
                ],
            )

        summary = self._build_summary(path, metrics)
        (out / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        (out / "summary.txt").write_text(self._summary_text(path, metrics), encoding="utf-8")
        self._fk_preview = fk_poses
        self._write_preview(out, path)
        print(f"\n输出目录: {out}")
        print(self._summary_text(path, metrics))

    def _build_summary(self, path: SpherePath, metrics: dict) -> dict:
        return {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "args": vars(self.args),
            "path": {
                "center_m": path.center.tolist(),
                "radius_m": path.radius,
                "samples": len(path.positions),
                "length_m": path.length,
                "orient": self.args.orient,
                "segments": sorted(set(path.segments)),
            },
            "metrics": _json_safe(metrics),
        }

    def _summary_text(self, path: SpherePath, metrics: dict) -> str:
        ik = metrics.get("ik", {})
        orient_text = {
            "fixed": "fixed（姿态不变）",
            "inward": "inward（tool Z 指向球心）",
            "radial": "radial（tool Z 背离球心）",
        }.get(self.args.orient, self.args.orient)
        lines = [
            "球面逆解性能评估",
            f"  球心: {np.round(path.center, 4).tolist()} m, 半径: {path.radius:.3f} m",
            f"  采样: {ik.get('samples', 0)}, 弧长: {path.length:.3f} m, 姿态: {orient_text}",
            f"  逆解来源: {'控制器' if self.args.ik_source == 0 else '本地模型'} (form={self.args.form})",
            f"  成功率: {ik.get('success', 0)}/{ik.get('samples', 0)}",
        ]
        selection = metrics.get("selection") or {}
        if selection.get("source") == "dexterous-workspace":
            lines.append(
                f"  选球: 灵巧空间（{Path(selection.get('workspace_data', '')).name}），"
                f"min_dex={selection.get('min_dexterity', 0.0):.3f}，"
                f"首点偏差 {selection.get('start_gap_m', 0.0) * 1000.0:.0f} mm"
            )
        elif selection.get("source") == "explicit-center":
            lines.append("  选球: 显式 --center")
        elif selection:
            lines.append("  选球: 当前 TCP + R·x（自动选球未启用或不可用）")
        reports = selection.get("full_validation") or []
        chosen = next(
            (item for item in reports if item.get("index") == selection.get("candidate_index")),
            None,
        )
        if chosen:
            lines.append(
                f"  全量校验: {chosen['success'] * 100:.0f}% 成功, "
                f"max|dq|={chosen['max_step_rad']:.3f} rad, "
                f"限位越界 {chosen['limit_violations']}"
            )
        if ik.get("coverage", 1.0) < 1.0:
            lines.append(
                f"  可执行覆盖: {ik['executed_samples']}/{ik['samples']} "
                f"({ik['coverage'] * 100.0:.1f}%)，末段逆解失败已截断"
            )
        timeline = metrics.get("timeline")
        if timeline and timeline.get("samples"):
            lines.append(
                f"  指令峰值: 速度 {timeline['v_peak_deg_s']:.1f}°/s, "
                f"加速度 {timeline['a_peak_deg_s2']:.0f}°/s²"
            )
        latency = ik.get("latency_ms")
        if latency:
            lines.append(
                f"  延迟: median {latency['median']:.2f} ms, p95 {latency['p95']:.2f} ms, "
                f"max {latency['max']:.2f} ms"
            )
        roll_calls = ik.get("roll_calls")
        if roll_calls:
            lines.append(
                f"  IK 调用: 合计 {roll_calls['total']} 次, 单点最多 {roll_calls['max']} 次"
            )
        roll_offset = ik.get("roll_offset_deg")
        if roll_offset is not None:
            lines.append(f"  roll 偏移（全程固定）: {roll_offset:+.0f}°")
        lines.append(
            f"  关节连续性: max|dq|={ik.get('max_step_rad', 0.0):.4f} rad, "
            f"跳变 {ik.get('jump_count', 0)} 处"
        )
        start_gap = metrics.get("start_gap")
        if start_gap:
            joint_text = (
                f", 关节 {start_gap['joint_rad']:.3f} rad" if "joint_rad" in start_gap else ""
            )
            lines.append(
                f"  首点跨度: 位置 {start_gap['position_m'] * 1000.0:.1f} mm, "
                f"姿态 {start_gap['rotation_deg']:.1f}°{joint_text}"
            )
        fk = metrics.get("fk")
        if fk and fk.get("samples", 0) > 0:
            rot_label = "姿态" if fk.get("rot_metric") == "orientation" else "tool Z"
            lines.append(
                f"  FK 残差: 位置 max {fk['pos_mm']['max']:.3f} mm "
                f"(p95 {fk['pos_mm']['p95']:.3f} mm), {rot_label} max {fk['rot_deg']['max']:.3f}°"
            )
        limits = metrics.get("limits")
        if limits:
            text = "无" if not limits["violations"] else ", ".join(limits["violations"])
            lines.append(f"  关节限位: {text}")
            if "min_margin_rad" in limits:
                lines.append(
                    f"  限位最小余量: J{limits['min_margin_joint']} "
                    f"{math.degrees(limits['min_margin_rad']):.1f}°"
                )
        actual = metrics.get("actual")
        if actual:
            lines.append(
                f"  实采: 指令 {actual.get('command_frames', 0)} 帧, "
                f"TCP {actual.get('tcp_frames', 0)} 帧, 关节 {actual.get('joint_frames', 0)} 帧"
            )
            tcp_error = actual.get("tcp_error_mm")
            if tcp_error:
                lines.append(
                    f"  TCP 误差: RMS {tcp_error['rms']:.3f} mm, max {tcp_error['max']:.3f} mm"
                )
            orientation_error = actual.get("orientation_error_deg")
            if orientation_error:
                lines.append(
                    f"  姿态跟踪误差: RMS {orientation_error['rms']:.3f}°, "
                    f"max {orientation_error['max']:.3f}°"
                )
            radius_error = actual.get("radius_error_mm")
            if radius_error:
                lines.append(
                    f"  球面半径误差: RMS {radius_error['rms']:.3f} mm, "
                    f"max {radius_error['max']:.3f} mm"
                )
            nearest = actual.get("nearest_path_mm")
            if nearest:
                lines.append(
                    f"  点云到理论轨迹最近距离: RMS {nearest['rms']:.3f} mm, "
                    f"max {nearest['max']:.3f} mm"
                )
            joint_error = actual.get("joint_error_rad")
            if joint_error:
                lines.append(
                    f"  关节跟踪误差: RMS {joint_error['rms']:.4f} rad, "
                    f"max {joint_error['max']:.4f} rad"
                )
        if metrics.get("blocking"):
            lines.append("  预检阻断项:")
            lines.extend(f"    - {item}" for item in metrics["blocking"])
        return "\n".join(lines) + "\n"

    def _print_precheck(self, metrics: dict) -> None:
        ik = metrics.get("ik", {})
        if ik.get("fail", 0):
            self.logger.warn(f"逆解失败 {ik['fail']} 处，索引 {ik['fail_indices']}")
        if ik.get("jump_count", 0):
            self.logger.warn(f"关节跳变 {ik['jump_count']} 处，索引 {ik['jump_indices'][:10]}")
        for item in metrics.get("blocking", []):
            self.logger.error(f"预检: {item}")

    def _print_tracking(self, actual: dict) -> None:
        if actual.get("tcp_error_mm"):
            self.logger.info(
                f"TCP 跟踪误差 RMS {actual['tcp_error_mm']['rms']:.3f} mm, "
                f"max {actual['tcp_error_mm']['max']:.3f} mm"
            )
        if actual.get("orientation_error_deg"):
            self.logger.info(f"姿态跟踪误差 max {actual['orientation_error_deg']['max']:.3f}°")
        if actual.get("joint_error_rad"):
            self.logger.info(f"关节跟踪误差 max {actual['joint_error_rad']['max']:.4f} rad")

    def _write_preview(self, out: Path, path: SpherePath) -> None:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as exc:
            self.logger.warn(f"matplotlib 不可用，跳过预览图: {exc}")
            return
        try:
            figure = plt.figure(figsize=(7, 6))
            axes = figure.add_subplot(111, projection="3d")
            axes.scatter(
                path.positions[:, 0],
                path.positions[:, 1],
                path.positions[:, 2],
                s=1,
                c="tab:blue",
                label="target",
            )
            fk_mask = np.isfinite(self._fk_preview).all(axis=1)
            if fk_mask.any():
                axes.scatter(
                    self._fk_preview[fk_mask, 0],
                    self._fk_preview[fk_mask, 1],
                    self._fk_preview[fk_mask, 2],
                    s=1,
                    c="tab:orange",
                    label="ik fk",
                )
            if self._tcp_rec:
                actual = np.asarray([pose[:3] for _, pose in self._tcp_rec])
                axes.scatter(
                    actual[:, 0], actual[:, 1], actual[:, 2], s=1, c="tab:green", label="actual"
                )
            # 每 ~24 个点画一根 tool Z，直观检查姿态是否指向预期方向
            stride = max(1, len(path.rpys) // 24)
            for count, index in enumerate(range(0, len(path.rpys), stride)):
                axis_z = rpy_to_matrix(path.rpys[index])[:, 2]
                axes.quiver(
                    path.positions[index, 0],
                    path.positions[index, 1],
                    path.positions[index, 2],
                    axis_z[0],
                    axis_z[1],
                    axis_z[2],
                    length=0.008,
                    normalize=True,
                    color="tab:blue",
                    alpha=0.6,
                    label="target tool Z" if count == 0 else None,
                )
            if self._tcp_rec:
                stride = max(1, len(self._tcp_rec) // 24)
                for count, (_, pose) in enumerate(self._tcp_rec[::stride]):
                    axis_z = rpy_to_matrix(pose[3:])[:, 2]
                    axes.quiver(
                        pose[0],
                        pose[1],
                        pose[2],
                        axis_z[0],
                        axis_z[1],
                        axis_z[2],
                        length=0.008,
                        normalize=True,
                        color="tab:green",
                        alpha=0.8,
                        label="actual tool Z" if count == 0 else None,
                    )
            axes.scatter(
                [path.center[0]],
                [path.center[1]],
                [path.center[2]],
                c="tab:red",
                marker="+",
                label="center",
            )
            axes.set_xlabel("x [m]")
            axes.set_ylabel("y [m]")
            axes.set_zlabel("z [m]")
            axes.set_title("upper hemisphere IK bench")
            axes.legend(loc="upper right")
            axes.set_box_aspect((1, 1, 1))
            figure.tight_layout()
            figure.savefig(out / "preview.png", dpi=150)
            plt.close(figure)
        except Exception as exc:
            self.logger.warn(f"生成预览图失败: {exc}")


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return [_json_safe(item) for item in value.tolist()]
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return None if not math.isfinite(number) else number
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


# ---------------------------------------------------------------------------
# 自检：不依赖 ROS / 机械臂
# ---------------------------------------------------------------------------


def run_self_test() -> int:
    ok = True

    def check(condition: bool, message: str) -> None:
        nonlocal ok
        print(("PASS " if condition else "FAIL ") + message)
        ok = ok and bool(condition)

    rng = np.random.default_rng(0)
    max_matrix_error = 0.0
    for _ in range(2000):
        rpy = rng.uniform(-math.pi, math.pi, 3)
        rpy[1] = rng.uniform(-1.45, 1.45)
        matrix = rpy_to_matrix(rpy)
        restored = rpy_to_matrix(matrix_to_rpy(matrix))
        max_matrix_error = max(max_matrix_error, float(np.max(np.abs(matrix - restored))))
    check(max_matrix_error < 1e-9, f"rpy 往返一致性 max|dR|={max_matrix_error:.2e}")

    for pitch in (math.pi / 2, -math.pi / 2):
        rpy = np.array([0.3, pitch, -0.4])
        restored = rpy_to_matrix(matrix_to_rpy(rpy_to_matrix(rpy)))
        check(
            float(np.max(np.abs(rpy_to_matrix(rpy) - restored))) < 1e-9,
            f"万向锁 rpy 往返 ry={pitch:+.2f}",
        )

    try:
        from scipy.spatial.transform import Rotation

        max_scipy_error = 0.0
        for _ in range(1000):
            rpy = rng.uniform(-math.pi, math.pi, 3)
            rpy[1] = rng.uniform(-1.45, 1.45)
            reference = Rotation.from_euler("xyz", rpy).as_matrix()
            max_scipy_error = max(
                max_scipy_error, float(np.max(np.abs(rpy_to_matrix(rpy) - reference)))
            )
        check(
            max_scipy_error < 1e-12,
            f"rpy 与 scipy 'xyz' 外旋一致 max|dR|={max_scipy_error:.2e}",
        )
    except ImportError:
        print("SKIP scipy 不可用，跳过外旋约定交叉校验")

    center = np.array([0.30, 0.0, 0.40])
    radius = 0.05
    e1 = np.array([1.0, 0.0, 0.0])
    e2 = np.array([0.0, 1.0, 0.0])
    e3 = np.array([0.0, 0.0, 1.0])
    path = build_sphere_path(
        center, radius, e1, e2, e3, 0.001, "fixed", np.array([math.pi, 0.0, 0.0])
    )
    radii = np.linalg.norm(path.positions - center, axis=1)
    check(float(np.max(np.abs(radii - radius))) < 1e-12, "轨迹点全部落在球面上")
    segments = np.asarray(path.segments)
    check(
        list(dict.fromkeys(segments)) == ["circle_xz", "equator_bridge", "circle_yz"],
        "段顺序为 circle_xz → equator_bridge → circle_yz",
    )
    on_xz = path.positions[segments == "circle_xz"]
    on_yz = path.positions[segments == "circle_yz"]
    on_bridge = path.positions[segments == "equator_bridge"]
    check(
        len(on_xz) > 0 and len(on_yz) > 0 and len(on_bridge) > 0,
        f"三段均非空: {len(on_xz)} / {len(on_bridge)} / {len(on_yz)}",
    )
    steps = np.linalg.norm(np.diff(path.positions, axis=0), axis=1)
    check(
        float(steps.max()) < 1.5 * 0.001,
        f"轨迹连续：最大相邻点间距 {float(steps.max()) * 1000.0:.3f} mm" "（接缝 ≤ √2 个采样步长）",
    )
    check(
        len(on_bridge) > 0
        and float(np.max(np.abs((on_bridge - center) @ e3))) < 1e-9
        and float(np.linalg.norm(on_bridge[0] - (center + radius * e1))) < 1.5 * 0.001,
        "赤道连接弧在赤道平面内且两端各让开一个采样步长",
    )
    if len(on_xz) and len(on_yz):
        check(
            float(np.max(np.abs((on_xz - center) @ e2))) < 1e-9,
            f"xz 面圆 {len(on_xz)} 点全部落在 xz 平面",
        )
        check(
            float(np.max(np.abs((on_yz - center) @ e1))) < 1e-9,
            f"yz 面圆 {len(on_yz)} 点全部落在 yz 平面",
        )
        start_xz = center - radius * e1
        start_yz = center - radius * e2
        check(
            float(np.linalg.norm(path.positions[0] - start_xz)) < 1.5 * 0.001,
            "轨迹首点在球心 − R·e1 内移一个采样步长（避开赤道万向锁）",
        )
        check(
            float(np.linalg.norm(on_yz[0] - start_yz)) < 1.5 * 0.001,
            "yz 弧首点在球心 − R·e2 内移一个采样步长",
        )
        check(
            float(np.linalg.norm(on_xz[-1] - (center + radius * e1))) < 2 * 0.001,
            "xz 弧末点逼近球心 + R·e1",
        )
        check(
            float(np.linalg.norm(on_yz[-1] - (center + radius * e2))) < 2 * 0.001,
            "yz 弧末点逼近球心 + R·e2",
        )
        z_offsets = (path.positions - center) @ e3
        check(
            float(np.min(z_offsets)) > -1e-9,
            f"全部点位于上半球（最小 z-C_z={float(np.min(z_offsets)):.2e} m）",
        )
        north = center + radius * e3
        check(
            float(np.min(np.linalg.norm(path.positions - north, axis=1))) < 0.002,
            "两段弧均经过北极点",
        )
    # 两段半圆弧（2·πR）+ 赤道连接弧（πR/2），全部落在球面上
    expected = 2 * math.pi * radius + 0.5 * math.pi * radius
    check(
        abs(path.length - expected) / expected < 0.02,
        f"轨迹长度 {path.length:.4f} m vs 理论 {expected:.4f} m（含赤道连接弧）",
    )
    check(len(path.positions) > 250, f"采样点数 {len(path.positions)}")
    check(np.allclose(path.rpys, path.rpys[0]), "fixed 模式姿态恒定")

    radial = build_sphere_path(
        center, radius, e1, e2, e3, 0.002, "radial", np.array([math.pi, 0.0, 0.0])
    )
    check(np.all(np.isfinite(radial.rpys)), "radial 模式姿态全部有限")

    inward = build_sphere_path(
        center, radius, e1, e2, e3, 0.002, "inward", np.array([math.pi, 0.0, 0.0])
    )
    radial_z = np.array([rpy_to_matrix(rpy)[:, 2] for rpy in radial.rpys])
    inward_z = np.array([rpy_to_matrix(rpy)[:, 2] for rpy in inward.rpys])
    outward = (radial.positions - center) / radius
    toward_center = (center - inward.positions) / radius
    check(
        float(np.max(np.abs(radial_z - outward))) < 1e-9,
        "radial 模式 tool Z 始终背离球心",
    )
    check(
        float(np.max(np.abs(inward_z - toward_center))) < 1e-9,
        "inward 模式 tool Z 始终指向球心",
    )

    # 平行移动的 tool X：全程（含接缝）姿态连续，且存在某个 roll 偏移把 |pitch|
    # 压到万向锁之外（控制器在 ±90° 附近会「报 success 但姿态错」）
    step_deg = math.degrees(0.002 / radius)
    for name, sample_path in (("radial", radial), ("inward", inward)):
        max_rot_step = max(
            rotation_error_deg(sample_path.rpys[i], sample_path.rpys[i + 1])
            for i in range(len(sample_path.rpys) - 1)
        )
        check(
            max_rot_step < 1.5 * step_deg,
            f"{name} 姿态全程连续：max 相邻转角 {max_rot_step:.2f}°"
            f"（每步理论 {step_deg:.2f}°）",
        )
        best_pitch = min(
            max(abs(pitch_deg(rotate_rpy_about_z(rpy, offset))) for rpy in sample_path.rpys)
            for offset in roll_candidates(45.0)
        )
        check(
            best_pitch < PITCH_LIMIT_DEG,
            f"{name} 存在避开万向锁的 roll：best max|pitch|={best_pitch:.2f}°"
            f"（上限 {PITCH_LIMIT_DEG:.0f}°）",
        )

    # roll 候选：绕 tool Z 旋转不改变 tool Z；候选按与上一帧的距离排序
    check(
        roll_candidates(45.0) == (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0),
        f"roll 候选序列 {roll_candidates(45.0)}",
    )
    base_rpy = radial.rpys[10]
    rotated_rpy = rotate_rpy_about_z(base_rpy, 37.0)
    check(
        float(np.linalg.norm(rpy_to_matrix(rotated_rpy)[:, 2] - rpy_to_matrix(base_rpy)[:, 2]))
        < 1e-9,
        "绕 tool Z 旋转不改变 tool Z 指向",
    )
    check(
        tool_z_error_deg(base_rpy, rotated_rpy) < 1e-9
        and abs(rotation_error_deg(base_rpy, rotated_rpy) - 37.0) < 1e-6,
        "tool_z_error_deg 忽略 roll：整姿态差 37°，tool Z 差 0°",
    )
    ordered = order_roll_candidates(base_rpy, rotated_rpy, 45.0)
    distances = [rotation_error_deg(rpy, rotated_rpy) for rpy in ordered]
    check(
        distances == sorted(distances) and abs(distances[0] - 8.0) < 1e-9,
        f"roll 候选按接近上一帧排序（首选偏差 {distances[0]:.2f}°）",
    )

    # 整段 roll 预选的数学前提：偏移可从姿态反解回来；同一偏移逐点保持 tool Z
    for offset_deg in (0.0, 45.0, -90.0, 135.0, 180.0):
        rolled = rotate_rpy_about_z(base_rpy, offset_deg)
        check(
            abs(SphereIKBench._roll_offset_of(base_rpy, rolled) - offset_deg) < 1e-6,
            f"_roll_offset_of 往返 {offset_deg:+.0f}°",
        )
    rolled_all = [rotate_rpy_about_z(rpy, 45.0) for rpy in inward.rpys]
    check(
        max(tool_z_error_deg(a, b) for a, b in zip(inward.rpys, rolled_all)) < 1e-4,
        "整段同一 roll 偏移：逐点 tool Z 不变",
    )

    # 整段预选：首选偏移（帧连续）贴关节限位时改用余量足够的偏移
    class _RollStub(SphereIKBench):
        def __init__(self, path):
            self.args = argparse.Namespace(
                roll_step_deg=45.0,
                min_joint_margin=5.0,
                max_residual_mm=5.0,
                max_residual_deg=5.0,
            )
            self._stop = False
            self.path = path
            self._last = None
            self._roll_offset_deg = None
            self.logger = logging.getLogger("roll-stub")
            self.index_of = {p.tobytes(): i for i, p in enumerate(path.positions)}

        def _ik_candidate(self, position, rpy, reference, arm_angle, dof):
            index = self.index_of[position.tobytes()]
            offset = SphereIKBench._roll_offset_of(self.path.rpys[index], rpy)
            joints = np.zeros(7)
            joints[5] = 1.10 if abs(offset) < 1e-6 else 0.30  # 贴限位 / 余量充足
            self._last = (np.asarray(position, dtype=float), np.asarray(rpy, dtype=float))
            return joints, 1.0, ""

        def _fk_pose(self, joints):
            return np.concatenate(self._last)

    stub = _RollStub(inward)
    stub_limits = (np.full(7, -1.8), np.array([1.8] * 5 + [1.15, 1.8]))
    chosen = stub._choose_roll(inward, np.zeros(7), 0.0, 7, stub_limits, math.radians(5.0))

    class _BlipStub(_RollStub):
        """单点 IK 偶发失败时不应改 roll（改一次就是整腕重构）。"""

        def __init__(self, path, blip):
            super().__init__(path)
            self.args = argparse.Namespace(
                roll_step_deg=45.0,
                min_joint_margin=0.1,
                max_residual_mm=5.0,
                max_residual_deg=5.0,
                roll_search=True,
                orient="inward",
                check_fk=False,
                form=0,
                ik_source=0,
                roll_switch_after=3,
            )
            self.blip = set(blip)

        def _ik_candidate(self, position, rpy, reference, arm_angle, dof):
            index = self.index_of[position.tobytes()]
            offset = SphereIKBench._roll_offset_of(self.path.rpys[index], rpy)
            preferred = self._roll_offset_deg
            if index in self.blip and preferred is not None and abs(offset - preferred) < 1e-6:
                return None, 1.0, "偶发失败"
            return super()._ik_candidate(position, rpy, reference, arm_angle, dof)

    blip_path = inward.truncate(40)
    blip_stub = _BlipStub(blip_path, blip={7})
    _, blip_ok, _, _, _, blip_chosen, _ = blip_stub._solve_path(
        blip_path, np.zeros(7), 0.0, 7, stub_limits, roll_reference=blip_path.rpys[0]
    )
    preferred = blip_stub._roll_offset_deg
    kept = max(
        abs(SphereIKBench._roll_offset_of(blip_path.rpys[i], blip_chosen[i]) - preferred)
        for i in (6, 8)
    )
    check(
        not blip_ok[7] and kept < 1e-6,
        f"单点 IK 失败不换 roll（该点空过，相邻点仍用预选 {preferred:+.0f}°）",
    )
    blip_stub = _BlipStub(blip_path, blip={7, 8, 9, 10})
    _, blip_ok, _, _, _, blip_chosen, _ = blip_stub._solve_path(
        blip_path, np.zeros(7), 0.0, 7, stub_limits, roll_reference=blip_path.rpys[0]
    )
    switched = abs(SphereIKBench._roll_offset_of(blip_path.rpys[10], blip_chosen[10]) - preferred)
    check(
        blip_ok[10] and switched > 1e-6,
        "连续 4 点失败后才换 roll（roll-switch-after=3）",
    )
    check(
        chosen is not None and abs(chosen) > 1e-6,
        f"全程预选避开贴限位的首选 roll（偏移 {chosen}）",
    )

    # 首点位于 -e1 赤道：fixed 姿态不变；inward/radial 的 tool Z 分别指向
    # +e1 / -e1（首点的球面法向），与当前姿态无关
    now_pose = np.array([0.30, 0.0, 0.40, math.pi, 0.0, 0.0])
    now_center = now_pose[:3] + radius * e1
    now_joints = np.zeros(6)
    for orient_name, expected_z in (("fixed", -e3), ("inward", e1), ("radial", -e1)):
        gap_path = build_sphere_path(
            now_center, radius, e1, e2, e3, 0.002, orient_name, now_pose[3:]
        )
        gap_solutions = np.tile(now_joints, (len(gap_path.positions), 1))
        gap = start_gap_metrics(
            gap_path,
            now_pose,
            now_joints,
            gap_solutions,
            np.ones(len(gap_path.positions), dtype=bool),
        )
        first_z = rpy_to_matrix(gap_path.rpys[0])[:, 2]
        z_error_deg = math.degrees(
            math.acos(max(-1.0, min(1.0, float(np.dot(first_z, expected_z)))))
        )
        check(
            abs(gap["position_m"]) < 1.5 * 0.002
            and z_error_deg < 1.5 * math.degrees(0.002 / radius)
            and abs(gap.get("joint_rad", -1.0)) < 1e-12,
            f"首点跨度 {orient_name}: 位置 {gap['position_m']:.4f} m, "
            f"tool Z 偏 {z_error_deg:.2f}°",
        )
    shifted = build_sphere_path(
        now_center - np.array([0.0, 0.0, 0.15]), radius, e1, e2, e3, 0.002, "fixed", now_pose[3:]
    )
    gap_shifted = start_gap_metrics(
        shifted,
        now_pose,
        now_joints,
        np.tile(now_joints, (len(shifted.positions), 1)),
        np.ones(len(shifted.positions), dtype=bool),
    )
    check(
        abs(gap_shifted["position_m"] - 0.15) < 0.003,
        f"球心下移 0.15 m 时首点位置跨度 {gap_shifted['position_m']:.4f} m",
    )

    with tempfile.TemporaryDirectory() as temp_dir:
        temp = Path(temp_dir)
        points = path.positions[:64]
        intensity = np.arange(64, dtype=float)
        write_pcd(temp / "test.pcd", points, intensity)
        text = (temp / "test.pcd").read_text(encoding="ascii")
        check("POINTS 64" in text and "FIELDS x y z intensity" in text, "PCD 写出格式正确")
        write_ply(temp / "test.ply", points, intensity)
        text = (temp / "test.ply").read_text(encoding="ascii")
        check("element vertex 64" in text and "end_header" in text, "PLY 写出格式正确")

    solutions = np.zeros((len(path.positions), 6))
    current = np.zeros(6)
    vmax = math.radians(80.0)
    timeline = build_timeline(path, solutions, 0.02, current, 0.5, 0.05, False, vmax, 0.5)
    check(timeline.total > 0.0, f"时间轴总长 {timeline.total:.2f} s")
    joints_zero, _, _, kind_zero = timeline.sample(0.0)
    check(np.allclose(joints_zero, current) and kind_zero == "blend_in", "时间轴 0 时刻为当前关节")
    last_joints, _, _, _ = timeline.sample(timeline.total)
    check(np.allclose(last_joints, solutions[-1]), "时间轴终点为最后一个解")
    first_path_time = 0.5 + 0.5 * (timeline.times[2] - timeline.times[1])
    check(
        timeline.sample(first_path_time)[3] == "path", "blend 之后第一段标注为 path（不再错位一格）"
    )

    # 路径段匀速：中段相邻帧的关节速度应接近 --speed，不再每点归零
    linear_solutions = np.zeros((len(path.positions), 6))
    linear_solutions[:, 0] = path.s
    linear_timeline = build_timeline(
        path, linear_solutions, 0.02, np.zeros(6), 0.5, 0.05, False, vmax, 0.5
    )
    check_times = np.arange(0.0, linear_timeline.total, 0.01)
    commanded = np.array([linear_timeline.sample(t)[0] for t in check_times])
    speeds = np.abs(np.diff(commanded[:, 0])) / 0.01
    cruise = speeds[int(len(speeds) * 0.3) : int(len(speeds) * 0.7)]
    check(
        float(np.max(speeds)) < vmax and float(np.min(cruise)) > 0.9 * 0.02,
        f"路径段速度剖面：巡航 {float(np.min(cruise)):.4f} m/s，峰值 {float(np.max(speeds)):.4f} m/s",
    )

    # 段类型与跳变时长：synthetic 跳变段必须标注为 jump 且满足 vmax
    jump_positions = np.array(
        [
            [0.30, 0.0, 0.40],
            [0.3001, 0.0, 0.40],
            [0.3002, 0.0, 0.40],
            [0.3003, 0.0, 0.40],
            [0.3004, 0.0, 0.40],
        ]
    )
    jump_path = SpherePath(
        center=center,
        radius=radius,
        positions=jump_positions,
        rpys=np.zeros((5, 3)),
        s=np.array([0.0, 0.0001, 0.0002, 0.0003, 0.0004]),
        segments=["a"] * 5,
    )
    jump_solutions = np.zeros((5, 6))
    jump_solutions[3:, 0] = 0.4
    jump_timeline = build_timeline(
        jump_path, jump_solutions, 0.02, np.zeros(6), 0.5, 0.05, True, vmax, 0.5
    )
    jump_mid = 0.5 * (jump_timeline.times[3] + jump_timeline.times[4])
    jump_span = float(jump_timeline.times[4] - jump_timeline.times[3])
    check(
        jump_timeline.sample(jump_mid)[3] == "jump" and jump_span >= 1.5 * 0.4 / vmax - 1e-9,
        f"跳变段标注与时长正确（按 smoothstep 峰值限速）："
        f"kind={jump_timeline.sample(jump_mid)[3]}, {jump_span:.2f} s",
    )
    check(
        jump_span <= 0.5 + 1e-9,
        f"跳变段不再叠加固定 0.5 s 下限：{jump_span:.2f} s",
    )
    # 冗余轴每毫米都在动（实测 7 轴臂角约 0.06 rad/mm）不该判成跳变：旧判据只看
    # step>0.05，会把每个采样点都拉成固定停顿，实测 392 点里有 64 个这种"跳变"
    redundant = np.zeros((len(path.positions), 7))
    redundant[:, :6] = linear_solutions
    redundant[:, 6] = np.arange(len(redundant)) * 0.06
    redundant_timeline = build_timeline(
        path, redundant, 0.02, np.zeros(7), 0.5, 0.05, True, vmax, 0.5
    )
    check(
        "jump" not in redundant_timeline.kinds
        and abs(redundant_timeline.total - linear_timeline.total) < 0.3,
        f"冗余轴 0.06 rad/mm 不判跳变：jump 段 {redundant_timeline.kinds.count('jump')} 个，"
        f"总长 {redundant_timeline.total:.2f} s vs 纯路径 {linear_timeline.total:.2f} s",
    )
    # blend 时限不足时按 vmax 自动延长（smoothstep 峰值 1.5 倍）
    long_gap = build_timeline(
        path, linear_solutions, 0.02, np.full(6, 2.0), 1.0, 0.05, False, vmax, 0.5
    )
    check(
        long_gap.times[1] >= 1.5 * 2.0 / vmax,
        f"blend 自动延长到 {long_gap.times[1]:.2f} s（首步 2.0 rad，vmax {vmax:.2f} rad/s）",
    )
    check(
        last_valid_index(np.array([True, True, False, False])) == 1
        and last_valid_index(np.array([False, False])) == -1,
        "last_valid_index 正确（全失败返回 -1）",
    )

    # 灵巧空间 .npz 读取 + 上半球候选筛选（合成数据，不依赖 ROS）
    with tempfile.TemporaryDirectory() as tmp_dir:
        step = 0.05
        axis = np.arange(-0.30, 0.301, step)
        grid = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), axis=-1).reshape(-1, 3)
        data_path = Path(tmp_dir) / "fake_workspace.npz"
        np.savez(
            str(data_path),
            centers=grid,
            dexterity=np.ones(len(grid)),
            step=np.float64(step),
            tau=np.float64(0.35),
            min_samples=np.int64(1),
            samples=np.int64(100),
            arm_type="fake",
        )
        workspace = load_workspace_data(data_path)
        check(
            workspace.step == step and workspace.interior.shape[0] == len(axis),
            f"工作空间 npz 读取/网格重建：shape={workspace.interior.shape}, step={workspace.step}",
        )
        candidates = half_sphere_candidates(
            workspace,
            radius=0.10,
            tcp_position=np.array([-0.10, 0.0, 0.10]),
            max_start_gap=0.5,
            limit=5,
            verify_points=32,
            orient="fixed",
            base_rpy=np.zeros(3),
        )
        check(
            len(candidates) > 0
            and candidates[0].start_gap < 0.03
            and np.allclose(candidates[0].center, [0.0, 0.0, 0.10]),
            (
                f"上半球候选筛选：{len(candidates)} 个，首选 "
                f"{np.round(candidates[0].center, 3).tolist() if candidates else None}，"
                f"首点偏差 {candidates[0].start_gap * 1000.0:.1f} mm"
                if candidates
                else "上半球候选筛选：无候选"
            ),
        )

    print("SELF-TEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------


def parse_args(argv=None) -> argparse.Namespace:
    preset_parser = argparse.ArgumentParser(add_help=False)
    preset_parser.add_argument("--preset", choices=tuple(PRESETS), default=DEFAULT_PRESET)
    preset_args, _ = preset_parser.parse_known_args(argv)
    parser = argparse.ArgumentParser(
        description="上半球线框轨迹控制器逆解性能评估与点云导出",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--preset",
        choices=tuple(PRESETS),
        default=preset_args.preset,
        help="参数预设；tcb705f-inward 固化已实测的 inward/roll/速度/限位参数，"
        "none 回到脚本通用默认值",
    )
    parser.add_argument("--radius", type=float, default=0.05, help="球半径（m）")
    parser.add_argument(
        "--center",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="球心（基坐标系，m）；缺省从灵巧空间自动选球，--no-auto-center 时取当前 TCP + 半径·x",
    )
    parser.add_argument(
        "--auto-center",
        dest="auto_center",
        action="store_true",
        default=True,
        help="缺省从灵巧空间体素数据自动选上半球球心（需 .npz，见 --workspace-data）",
    )
    parser.add_argument(
        "--no-auto-center",
        dest="auto_center",
        action="store_false",
        help="禁用自动选球，回退到当前 TCP + 半径·x（轨迹首点落在当前 TCP）",
    )
    parser.add_argument(
        "--workspace-data",
        default=str(DEFAULT_WORKSPACE_DATA),
        help="灵巧空间 .npz（dexterous_workspace.py --save-data 生成）",
    )
    parser.add_argument(
        "--auto-candidates",
        type=int,
        default=DEFAULT_AUTO_CANDIDATES,
        help="自动选球进入 IK 粗校验的候选数",
    )
    parser.add_argument(
        "--auto-verify-samples",
        type=int,
        default=DEFAULT_AUTO_VERIFY_SAMPLES,
        help="每个候选粗校验的抽样点数（上半球线框合计）",
    )
    parser.add_argument(
        "--auto-start-gap",
        type=float,
        default=DEFAULT_AUTO_START_GAP,
        help="自动选球允许的轨迹首点距当前 TCP 偏差（m）",
    )
    parser.add_argument(
        "--auto-full-candidates",
        type=int,
        default=DEFAULT_AUTO_FULL_CANDIDATES,
        help="粗校验后进入全量细采样逆解校验的候选数；通过即采用",
    )
    parser.add_argument(
        "--auto-max-step",
        type=float,
        default=DEFAULT_AUTO_MAX_STEP,
        help="全量校验允许的相邻解最大关节变化（rad），超过视为换支并换候选",
    )
    parser.add_argument(
        "--allow-arm-mismatch",
        action="store_true",
        help="工作空间数据臂型与 tl_driver arm_type 不一致时仍继续（默认报错）",
    )
    parser.add_argument(
        "--sample-step", type=float, default=DEFAULT_SAMPLE_STEP, help="弧长采样步长（m）"
    )
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="沿弧线速度（m/s）")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="ServoJ 下发频率（Hz）")
    parser.add_argument(
        "--orient",
        choices=("fixed", "inward", "radial"),
        default="fixed",
        help="末端姿态：fixed=姿态不变，inward=tool Z 始终指向球心，radial=tool Z 背离球心",
    )
    parser.add_argument(
        "--ik-source", type=int, choices=(0, 1), default=0, help="0=控制器，1=本地模型"
    )
    parser.add_argument(
        "--roll-search",
        dest="roll_search",
        action="store_true",
        default=True,
        help="radial/inward 下逐点试绕 tool Z 的候选 roll，取 FK 姿态/位置达标且不超"
        "关节限位的解（控制器在万向锁点会报 success 但姿态错，也会返回超限位解）",
    )
    parser.add_argument(
        "--no-roll-search", dest="roll_search", action="store_false", help="禁用 roll 搜索"
    )
    parser.add_argument(
        "--roll-switch-after",
        type=int,
        default=3,
        help="预选的全程 roll 连续失败多少个点后才允许换 roll（默认 3）；单点失败只是"
        "空过该点，避免为个别点整腕重构",
    )
    parser.add_argument(
        "--roll-step-deg",
        type=float,
        default=DEFAULT_ROLL_STEP_DEG,
        help="roll 候选步长（°）",
    )
    parser.add_argument(
        "--min-joint-margin",
        type=float,
        default=DEFAULT_ROLL_MARGIN_DEG,
        help="roll 搜索优先保留的关节软限位余量（°）；贴限位的解会把控制器 IK 逼到换分支",
    )
    parser.add_argument(
        "--roll-offset-deg",
        type=float,
        default=None,
        help="强制全程绕 tool Z 的 roll 偏移（°），跳过自动预选；仅 radial/inward 生效，"
        "用于复现已验证的平滑滚转相位",
    )
    parser.add_argument("--form", type=int, default=0, help="逆解形态（0=自动，1~8 指定分支）")
    parser.add_argument("--execute", action="store_true", help="实际执行 ServoJ 运动")
    parser.add_argument(
        "--start-pose-file",
        default=str(DEFAULT_START_POSE_FILE),
        help="记录的起始关节/位姿 JSON；成功执行后写入，--restore-start 读取",
    )
    parser.add_argument(
        "--restore-start",
        action="store_true",
        help="执行前用 move_joint 回到 --start-pose-file 记录的起始位姿（仅 --execute）",
    )
    parser.add_argument(
        "--return-start",
        dest="return_start",
        action="store_true",
        default=True,
        help="画完后 MoveJ 回本次起始关节位姿，保证下一遍可复现",
    )
    parser.add_argument(
        "--no-return-start", dest="return_start", action="store_false", help="画完不回起始位姿"
    )
    parser.add_argument(
        "--record-start",
        dest="record_start",
        action="store_true",
        default=True,
        help="成功执行后把本次起始位姿写入 --start-pose-file",
    )
    parser.add_argument(
        "--no-record-start", dest="record_start", action="store_false", help="不记录起始位姿"
    )
    parser.add_argument(
        "--move-velocity",
        type=float,
        default=DEFAULT_MOVE_VELOCITY,
        help="回起始位姿 MoveJ 的速度百分比（%%）",
    )
    parser.add_argument(
        "--move-timeout",
        type=float,
        default=DEFAULT_MOVE_TIMEOUT,
        help="回起始位姿 MoveJ 的超时（s）",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="忽略预检阻断项仍然执行，并对关节跳变做平滑过渡（危险，用于姿态验证）",
    )
    parser.add_argument(
        "--blend-time", type=float, default=1.0, help="从当前关节过渡到首点的时间（s）"
    )
    parser.add_argument("--hold-time", type=float, default=1.0, help="轨迹结束后保持时间（s）")
    parser.add_argument(
        "--ramp-time",
        type=float,
        default=DEFAULT_RAMP_TIME,
        help="路径段起步/收尾的加减速时间（s），0 表示立即达到巡航速度",
    )
    parser.add_argument(
        "--jump-threshold", type=float, default=DEFAULT_JUMP_THRESHOLD, help="关节跳变判据（rad）"
    )
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=DEFAULT_MAX_JOINT_STEP,
        help="允许的最大单步关节变化（rad）",
    )
    parser.add_argument(
        "--max-start-gap",
        type=float,
        default=DEFAULT_START_GAP_TOL_M,
        help="首点距当前 TCP 的允许偏差（m），超过即阻断执行",
    )
    parser.add_argument(
        "--max-start-rot",
        type=float,
        default=DEFAULT_START_ROT_TOL_DEG,
        help="首点姿态与当前 TCP 的允许偏差（°），超过即阻断执行（radial 会反向）",
    )
    parser.add_argument(
        "--allow-jumps", action="store_true", help="对超过阈值的关节跳变做关节空间平滑过渡"
    )
    parser.add_argument(
        "--no-allow-jumps",
        dest="allow_jumps",
        action="store_false",
        help="禁止对关节跳变做平滑过渡（覆盖预设）",
    )
    parser.add_argument(
        "--max-residual-mm",
        type=float,
        default=DEFAULT_MAX_RESIDUAL_MM,
        help="允许的最大 FK 位置残差（mm）",
    )
    parser.add_argument(
        "--max-residual-deg",
        type=float,
        default=5.0,
        help="允许的最大 FK 姿态残差（°）",
    )
    parser.add_argument(
        "--check-fk",
        dest="check_fk",
        action="store_true",
        default=True,
        help="逐点回代 FK 计算残差",
    )
    parser.add_argument(
        "--no-check-fk", dest="check_fk", action="store_false", help="跳过 FK 残差检查"
    )
    parser.add_argument(
        "--check-limits",
        dest="check_limits",
        action="store_true",
        default=True,
        help="检查关节软限位",
    )
    parser.add_argument(
        "--no-check-limits", dest="check_limits", action="store_false", help="跳过关节限位检查"
    )
    parser.add_argument("--vmax-deg", type=float, default=80.0, help="ServoJ 最大关节速度（°/s）")
    parser.add_argument(
        "--amax-deg", type=float, default=800.0, help="ServoJ 最大关节加速度（°/s²）"
    )
    parser.add_argument(
        "--jmax-deg", type=float, default=4800.0, help="ServoJ 最大加加速度（°/s³）"
    )
    parser.add_argument(
        "--speed-percent", type=float, default=0.0, help="执行前设置全局速度（百分比，0=不修改）"
    )
    parser.add_argument(
        "--stop-on-error",
        dest="stop_on_error",
        action="store_true",
        default=True,
        help="收到控制器消息立即停止",
    )
    parser.add_argument(
        "--no-stop-on-error", dest="stop_on_error", action="store_false", help="忽略控制器消息继续"
    )
    parser.add_argument("--timeout", type=float, default=10.0, help="单次服务调用超时（s）")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="输出根目录")
    parser.add_argument("--self-test", action="store_true", help="运行不依赖 ROS 的数学与文件自检")
    parser.set_defaults(**PRESETS[preset_args.preset])
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test()
    try:
        import rclpy
    except ImportError as exc:
        print(f"无法导入 rclpy: {exc}", file=sys.stderr)
        return 1
    try:
        from tl_ros2_interface.srv import Ik  # noqa: F401
    except ImportError:
        print(
            "无法导入 tl_ros2_interface，请先构建接口包并 source install/setup.bash",
            file=sys.stderr,
        )
        return 1

    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SphereIKBench(args)
        return 0 if node.run() else 1
    except Exception as exc:
        if node is not None:
            node.logger.error(f"运行失败: {exc}")
        else:
            print(f"运行失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
