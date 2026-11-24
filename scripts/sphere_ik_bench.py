#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sphere_ik_bench.py — 线框球轨迹的控制器逆解性能评估 + 点云导出

用两个互相垂直的大圆（极轴默认指向当前 TCP，球面起点为球面上离当前 TCP
最近的点）组成线框球，逐点调用 `/tl_driver/ik`（source=0，控制器逆解），统计
延迟、成功率、关节连续性、FK 残差与关节限位；可选通过 ServoJ 连续画出球面，
并采集实际 TCP 轨迹。末端姿态默认 `--orient fixed`（全程等于当前 TCP 姿态），
不做任何朝向球心的计算；`inward` / `radial` 是显式开启的额外模式。

球心不再固定为「当前 TCP − 半径·z」：默认按 `--workspace-data` 指定的灵巧
工作空间体素数据（`scripts/workspace_measure/results/<arm>_dexterous_workspace.npz`，
由 `dexterous_workspace.py --save-data` 生成）搜索一整个球面都落在可达体积内部、
且灵巧度最高的球心，再用控制器 IK 粗校验候选（成功率、相邻解跳变、关节软限位），
挑出能连续解出的球。`inward`/`radial` 的 tool X 沿法向序列平行移动，接缝处姿态
连续，不再有 90° roll 跳变；tool X 以当前 TCP 姿态为种子，`inward` 首点姿态与
当前 TCP 一致。数据臂型
会与驱动的只读参数 `arm_type` 比对，不一致时默认报错（`--allow-arm-mismatch`
可强制继续；其他臂型用 `--workspace-data` 指向对应的 `.npz`）。球面起点
取球面上离当前 TCP 最近的点；执行时若起点偏差超过阈值，会先用 `move_joint`
把机械臂移动到起点（`--prepose`，默认开启）。预定位走关节空间，执行前请确认
机械臂到球面起点之间无障碍。

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
    # 自动从灵巧空间选球，只解算、不动臂
    python3 scripts/sphere_ik_bench.py

    # 实画：必要时先 MoveJ 到球面起点，再 ServoJ 100 Hz 跟随
    python3 scripts/sphere_ik_bench.py --execute

    # 指定球心/半径，并用本地逆解做对比
    python3 scripts/sphere_ik_bench.py --center 0.30 0.00 0.40 --radius 0.04 \\
        --orient radial --ik-source 1 --allow-jumps

    # 工具 Z 始终指向球心（姿态随球面变化；起点姿态由 MoveJ 预定位到位）
    python3 scripts/sphere_ik_bench.py --execute --orient inward

    # 关闭自动选球，回到旧的「球心 = 当前 TCP − R·z」行为
    python3 scripts/sphere_ik_bench.py --no-auto

自检（不需要 ROS/机械臂）：
    python3 scripts/sphere_ik_bench.py --self-test
"""

from __future__ import annotations

import argparse
import csv
import json
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
# 轨迹首点与当前 TCP 的允许位置偏差（m）：blend 只在关节空间插值，
# 首点偏离越远，过渡段 TCP 的实际走线越不受控
DEFAULT_START_GAP_TOL_M = 0.02
# 轨迹首点与当前 TCP 的允许姿态偏差（°）：radial/inward 会按球面法向重定向
DEFAULT_START_ROT_TOL_DEG = 10.0
# 球面采样步长（m）
DEFAULT_SAMPLE_STEP = 0.001
# 沿弧线速度（m/s）
DEFAULT_SPEED = 0.02
# ServoJ 下发频率（Hz）
DEFAULT_RATE = 50.0
# 最后一帧下发后的等待时间（s）：让控制器把 ServoJ 目标跟随到位，
# 期间保持录制，使 actual_tcp 点云包含收尾段
DEFAULT_SERVOJ_SETTLE_S = 5.0
# 灵巧工作空间体素数据（dexterous_workspace.py --save-data 生成）
DEFAULT_WORKSPACE_DATA = (
    SCRIPT_DIR / "workspace_measure" / "results" / "tcb705f_dexterous_workspace.npz"
)
# 自动选球：起点（球面最近点）与当前 TCP 的最大允许偏差（m）
DEFAULT_AUTO_START_GAP_M = 0.30
# 自动选球：最多用控制器 IK 粗校验的候选球数量
DEFAULT_AUTO_CANDIDATES = 8
# 自动选球：离线几何检查时球面覆盖的采样方向数
DEFAULT_AUTO_DIRECTIONS = 128
# 自动选球：控制器 IK 粗校验时两圆合计的采样点数
DEFAULT_AUTO_VERIFY_SAMPLES = 64
# 粗校验时相邻解允许的最大关节变化（rad）：用于识别分支翻转（平滑球面轨迹远小于此）
PROBE_MAX_STEP_RAD = 0.8
# 预定位 action：goal 发送超时、结果等待额外余量（s）。结果等待 = prepose_timeout +
# PREPOSE_RESULT_MARGIN_S，与驱动 motion_action_timeout（默认 60 s）解耦。
PREPOSE_GOAL_TIMEOUT_S = 10.0
PREPOSE_RESULT_MARGIN_S = 10.0
# 预定位 MoveJ 的加速度/减速度（%）与平滑等级（0=精确到达）
PREPOSE_ACC_PERCENT = 20.0
PREPOSE_PL = 0
# 预定位后等待状态流刷新并到位的最长时间（s）；位置容差取 --max-start-gap
PREPOSE_STATE_TIMEOUT_S = 2.0
# 关节软限位检查的额外余量（rad）
JOINT_LIMIT_MARGIN_RAD = 0.02
# 自动选球方向数上限：按球面采样面积随 (R/step)^2 增长自动抬高
MAX_AUTO_DIRECTIONS = 4096


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


def unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("无法归一化零向量")
    return vector / norm


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


def parallel_frames(points: np.ndarray, normals: np.ndarray, start_x: np.ndarray) -> np.ndarray:
    """tool 坐标系：tool Z = 法向，tool X 沿法向序列平行移动。

    与逐点取切向当 tool X 不同，平行移动在接缝（同一法向）处不产生额外 roll，
    避免控制器为一次 90° roll 台阶做整腕重构。法向反向/退化时回退到固定参考轴。
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


# ---------------------------------------------------------------------------
# 灵巧工作空间数据与自动选球
# ---------------------------------------------------------------------------


def fibonacci_directions(count: int) -> np.ndarray:
    """球面均匀方向（Fibonacci 螺旋），返回 (count, 3) 单位向量。"""
    count = max(4, int(count))
    indices = np.arange(count, dtype=float) + 0.5
    phi = np.arccos(1.0 - 2.0 * indices / count)
    theta = math.pi * (1.0 + 5.0**0.5) * indices
    return np.column_stack(
        [
            np.cos(theta) * np.sin(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(phi),
        ]
    )


def perpendicular_basis(axis: np.ndarray):
    """构造与 axis 正交的 e1、e2，使 (e1, e2, axis) 构成右手系。"""
    axis = unit(np.asarray(axis, dtype=float))
    reference = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    e1 = unit(np.cross(reference, axis))
    e2 = unit(np.cross(axis, e1))
    return e1, e2


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


def sphere_candidates(
    workspace: WorkspaceData,
    radius: float,
    tcp_position: np.ndarray,
    max_start_gap: float,
    limit: int,
    directions: int = DEFAULT_AUTO_DIRECTIONS,
) -> list[SphereCandidate]:
    """在可达体积内部搜索整球可解的球心，按灵巧度排序。

    判定：球面按 Fibonacci 方向采样后，每个采样点所在体素都必须是「内部」
    体素（3×3×3 邻域全可达），整球因此落在可达体积内部而非边界薄壳上。
    方向数按 (R/step)² 自动抬高（上限 MAX_AUTO_DIRECTIONS），避免体素较细时
    球面采样间隙漏掉空洞。评分取球面采样点的最小/平均灵巧度；同时限制球面
    最近点与当前 TCP 的距离（|‖TCP−C‖ − R|）不超过 max_start_gap。
    """
    if radius <= 0.0:
        raise ValueError("radius 必须大于 0")
    if limit <= 0:
        return []
    required = int(math.ceil(4.0 * math.pi * (radius / workspace.step) ** 2))
    directions = min(max(int(directions), required), MAX_AUTO_DIRECTIONS)
    dirs = fibonacci_directions(directions)
    cells = np.argwhere(np.isfinite(workspace.dexterity) & workspace.interior)
    if cells.shape[0] == 0:
        return []
    centers = workspace.origin + cells * workspace.step
    tcp_position = np.asarray(tcp_position, dtype=float)
    shape = np.asarray(workspace.dexterity.shape, dtype=int)
    candidates: list[SphereCandidate] = []
    for center in centers:
        gap = abs(float(np.linalg.norm(tcp_position - center)) - radius)
        if gap > max_start_gap:
            continue
        points = center + radius * dirs
        ijk = np.round((points - workspace.origin) / workspace.step).astype(int)
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
                start_gap=float(gap),
            )
        )
    candidates.sort(
        key=lambda item: (item.min_dexterity, item.mean_dexterity, -item.start_gap),
        reverse=True,
    )
    return candidates[:limit]


# ---------------------------------------------------------------------------
# 球面轨迹：两个互相垂直的大圆，均从极轴方向（自动选球时指向当前 TCP）起步
# ---------------------------------------------------------------------------


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


def _append_arc(
    positions: list[np.ndarray],
    segment_names: list[str],
    center: np.ndarray,
    radius: float,
    u: np.ndarray,
    v: np.ndarray,
    sweep: float,
    step: float,
    name: str,
) -> None:
    """把 p(θ)=C+R(cosθ·u+sinθ·v) 按弧长等步长采样追加到列表。

    只发出 θ∈[0, sweep) 的 count 个点，不含终点；整圈闭合留给下一段首点。
    """
    arc_length = abs(sweep) * radius
    count = max(2, int(math.ceil(arc_length / max(step, 1e-6))))
    for i in range(count):
        theta = sweep * i / count
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        positions.append(center + radius * (cos_t * u + sin_t * v))
        segment_names.append(name)


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
    """生成线框球轨迹：两个互相垂直的大圆（极轴由调用方给定）。

    circle_xz：p(θ)=C+R(cosθ·e3+sinθ·e1)，θ∈[0, 2π)；
    circle_yz：p(θ)=C+R(cosθ·e3+sinθ·e2)，θ∈[0, 2π)。
    两圆都以 e3 为极轴、从极点 C+R·e3 起步，且都不发终点：除最后一段外，
    每段末点距极点一个采样步长、由下一段首点接上，最后一段停在距极点
    一个采样步长处（不闭环）。自动选球时 `_build_path` 取 e3 = unit(TCP−C)，
    首点就是球面上离当前 TCP 最近的点；显式 `--center` 时沿用基座 z 轴，
    首点为北极点。
    非 fixed 模式的首点姿态由几何决定（inward: tool Z = -e3，radial:
    tool Z = +e3，tool X 由 e1 平行移动），不保证等于当前 TCP 姿态；
    _precheck 会按首点位置/姿态跨度给阻断项，执行时可由 move_joint 预定位
    到首点解决。tool X 沿法向序列平行移动，两圆接缝处姿态连续，不再有
    90° 偏航跳变。
    """
    if radius <= 0.0:
        raise ValueError("radius 必须大于 0")
    positions: list[np.ndarray] = []
    segment_names: list[str] = []

    for name, in_plane in (("circle_xz", e1), ("circle_yz", e2)):
        _append_arc(
            positions,
            segment_names,
            center,
            radius,
            e3,
            in_plane,
            2.0 * math.pi,
            step,
            name,
        )

    points = np.asarray(positions)
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
# 时间轴：把采样点按速度展开成可 100 Hz 跟随的关节序列
# ---------------------------------------------------------------------------


@dataclass
class Timeline:
    """分段线性的指令时间轴，segment 以平滑插值连接。"""

    times: np.ndarray  # 每个断点的累计时刻 s
    joints: np.ndarray  # (M, dof) 断点关节角
    positions: np.ndarray  # (M, 3) 断点目标位置
    rpys: np.ndarray  # (M, 3) 断点目标姿态
    kinds: list[str]  # 每个断点所属段类型

    @property
    def total(self) -> float:
        return float(self.times[-1])

    def sample(self, t: float):
        """返回 (关节角, 目标位置, 目标姿态, 段类型)。"""
        if t <= 0.0:
            return self.joints[0], self.positions[0], self.rpys[0], self.kinds[0]
        if t >= self.total:
            return self.joints[-1], self.positions[-1], self.rpys[-1], self.kinds[-1]
        index = int(np.searchsorted(self.times, t, side="right"))
        index = min(max(index, 1), len(self.times) - 1)
        t0, t1 = float(self.times[index - 1]), float(self.times[index])
        alpha = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)  # smoothstep
        joints = self.joints[index - 1] + alpha * (self.joints[index] - self.joints[index - 1])
        position = self.positions[index - 1] + alpha * (
            self.positions[index] - self.positions[index - 1]
        )
        rpy = self.rpys[index - 1] + alpha * (self.rpys[index] - self.rpys[index - 1])
        return joints, position, rpy, self.kinds[index - 1]


def build_timeline(
    path: SpherePath,
    solutions: np.ndarray,
    speed: float,
    current_joints: np.ndarray,
    blend_time: float,
    jump_threshold: float,
    allow_jumps: bool,
    blend_jump_time: float,
) -> Timeline:
    """把逆解序列展开成时间轴。

    blend_time>0 时先用 blend_time 从当前关节平滑过渡到首点；
    相邻解跳变超过 jump_threshold 时，allow_jumps=True 则拉长到
    blend_jump_time 做关节空间过渡，否则保持原节奏（执行前会被拦截）。
    """
    times = [0.0]
    if blend_time > 0.0:
        joints = [np.asarray(current_joints, dtype=float)]
        kinds = ["blend_in"]
    else:
        joints = [solutions[0].copy()]
        kinds = ["path"]
    positions = [path.positions[0].copy()]
    rpys = [path.rpys[0].copy()]

    if blend_time > 0.0:
        times.append(blend_time)
        joints.append(solutions[0].copy())
        positions.append(path.positions[0].copy())
        rpys.append(path.rpys[0].copy())
        kinds.append("blend_in")

    for i in range(1, len(solutions)):
        ds = float(path.s[i] - path.s[i - 1])
        duration = max(ds / speed, 1e-4) if speed > 0.0 else 1e-4
        step = float(np.max(np.abs(solutions[i] - solutions[i - 1])))
        kind = "path"
        if step > jump_threshold:
            kind = "jump"
            if allow_jumps:
                duration = max(duration, blend_jump_time)
        times.append(times[-1] + duration)
        joints.append(solutions[i].copy())
        positions.append(path.positions[i].copy())
        rpys.append(path.rpys[i].copy())
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
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
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
        self._selection: dict = {}
        # 状态流序号：MoveJ 后用它判断读到的是新帧而不是移动前的旧值
        self._tcp_seq = 0
        self._joint_seq = 0
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

        # 预定位 action（把机械臂移动到球面起点）
        from rclpy.action import ActionClient
        from tl_ros2_interface.action import Move

        self._Move = Move
        self.move_cli = ActionClient(
            self.node, Move, "/tl_driver/move_joint", callback_group=self.cb_group
        )

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

    def _wait_future(self, future, name: str, timeout: float, ignore_stop: bool = False):
        """等待 action future 完成（节点在独立线程 spin）。"""
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

    def _move_to_start(self, joints: np.ndarray, dof: int) -> None:
        """调用 move_joint action 把机械臂移动到球面起点关节角。"""
        args = self.args
        if not self.move_cli.wait_for_server(timeout_sec=PREPOSE_GOAL_TIMEOUT_S):
            raise RuntimeError("/tl_driver/move_joint action 不可用")
        goal = self._Move.Goal()
        command = goal.command
        values = [0.0] * 14
        for axis in range(min(7, dof)):
            values[axis] = float(joints[axis])
        command.target_pos_value = values
        command.coord = 0
        command.velocity = float(args.prepose_velocity)
        command.velocity_sync = 0.0
        command.acc = PREPOSE_ACC_PERCENT
        command.dec = PREPOSE_ACC_PERCENT
        command.pl = PREPOSE_PL
        command.tool_num = 0
        command.user_num = 0
        command.para_sync = False
        goal.timeout_sec = float(args.prepose_timeout)
        self.logger.warn(
            f"预定位：MoveJ 到球面起点（{args.prepose_velocity:.0f}%，"
            f"超时 {args.prepose_timeout:.0f}s）"
        )
        send_future = self.move_cli.send_goal_async(goal)
        handle = self._wait_future(send_future, "move_joint goal", PREPOSE_GOAL_TIMEOUT_S)
        if not handle.accepted:
            raise RuntimeError("move_joint goal 被拒绝")
        result = self._wait_future(
            handle.get_result_async(),
            "move_joint",
            args.prepose_timeout + PREPOSE_RESULT_MARGIN_S,
            ignore_stop=True,
        )
        response = result.result
        if response is None or not response.success:
            message = response.message if response is not None else "无响应"
            raise RuntimeError(f"move_joint 失败: {message}")
        self.logger.info("MoveJ 已到达目标关节，等待状态流确认")

    def _prepose_if_needed(
        self,
        path,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        solutions: np.ndarray,
        ok: np.ndarray,
        dof: int,
        metrics: dict,
        prepose_info: dict,
    ):
        """必要时 MoveJ 到球面起点。

        返回 (current_joints, current_pose, prepose_info, abort)。
        abort=True 表示预定位已失败，调用方应写结果并停止。
        """
        if not bool(ok[0]):
            self.logger.error("球面首点逆解失败，无法预定位到起点")
            prepose_info["error"] = "球面首点逆解失败"
            return current_joints, current_pose, prepose_info, False

        limits_ok = True
        low = high = None
        if self.args.check_limits:
            limits = metrics.get("limits")
            if limits:
                low = np.asarray(limits["low_rad"], dtype=float)
                high = np.asarray(limits["high_rad"], dtype=float)
            else:
                pair = self._joint_limits(dof)
                if pair is not None:
                    low, high = pair
            if low is not None and high is not None:
                target = np.asarray(solutions[0], dtype=float)
                limits_ok = bool(
                    np.all(target >= low - JOINT_LIMIT_MARGIN_RAD)
                    and np.all(target <= high + JOINT_LIMIT_MARGIN_RAD)
                )
        if not limits_ok:
            self.logger.error("球面首点逆解超出关节软限位，拒绝预定位")
            prepose_info["error"] = "球面首点超出关节软限位"
            return current_joints, current_pose, prepose_info, False

        try:
            since_tcp, since_joint = self._tcp_seq, self._joint_seq
            self._move_to_start(solutions[0], dof)
            joints, pose = self._read_state(
                dof,
                since_tcp,
                since_joint,
                expected_position=path.positions[0],
                tolerance=self.args.max_start_gap,
            )
            prepose_info.update(
                {
                    "joints_rad": joints.tolist(),
                    "performed": True,
                }
            )
            return joints, pose, prepose_info, False
        except Exception as exc:
            self.logger.error(f"预定位失败，放弃执行: {exc}")
            prepose_info["error"] = str(exc)
            return current_joints, current_pose, prepose_info, True

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

    def _tcp_pose_array(self) -> np.ndarray:
        tcp = self._tcp_msg
        return np.array(
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

    def _read_state(
        self,
        dof: int,
        since_tcp: int = 0,
        since_joint: int = 0,
        expected_position: np.ndarray | None = None,
        tolerance: float | None = None,
    ):
        """读取 MoveJ 之后的新状态：要求两类消息都收到新帧且 TCP 已到位。"""
        tolerance = self.args.max_start_gap if tolerance is None else tolerance
        deadline = time.monotonic() + PREPOSE_STATE_TIMEOUT_S
        while True:
            if self._stop:
                raise RuntimeError("已中断")
            fresh = self._tcp_seq > since_tcp and self._joint_seq > since_joint
            if fresh and self._tcp_msg is not None and self._joint_msg is not None:
                pose = self._tcp_pose_array()
                arrived = (
                    expected_position is None
                    or float(np.linalg.norm(pose[:3] - expected_position)) <= tolerance
                )
                if arrived:
                    joints = np.asarray(self._joint_msg.position[:dof], dtype=float)
                    return joints, pose
            if time.monotonic() >= deadline:
                raise RuntimeError("等待 MoveJ 后的新状态超时（TCP 未到达起点）")
            time.sleep(0.01)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else time.time()

    def _on_tcp_pose(self, msg) -> None:
        self._tcp_msg = msg
        self._tcp_seq += 1
        if not self._recording:
            return
        pose = self._tcp_pose_array()
        self._tcp_rec.append((self._stamp_seconds(msg.header.stamp), pose))

    def _on_joint_state(self, msg) -> None:
        self._joint_msg = msg
        self._joint_seq += 1
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
            if self.args.radius <= 0.0:
                raise RuntimeError(f"radius 必须大于 0，当前 {self.args.radius}")
            if self.args.auto_start_gap < 0.0:
                raise RuntimeError("auto-start-gap 不能为负")
            if self.args.auto_candidates < 1:
                raise RuntimeError("auto-candidates 至少为 1")
            if self.args.auto_directions < 4:
                raise RuntimeError("auto-directions 至少为 4")
            if self.args.auto_verify_samples < 4:
                raise RuntimeError("auto-verify-samples 至少为 4")
            if self.args.prepose_velocity <= 0.0 or self.args.prepose_timeout <= 0.0:
                raise RuntimeError("prepose-velocity / prepose-timeout 必须大于 0")
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

            path, _ = self._build_path(current_pose, current_joints, arm_angle, dof)
            if self._stop:
                self.logger.warn("收到中断，停止扫描，不执行运动")
                return False
            self.logger.info(
                f"球心={np.round(path.center, 4).tolist()} m, R={path.radius:.3f} m, "
                f"采样={len(path.positions)}, 弧长={path.length:.3f} m, 姿态={self.args.orient}"
            )

            solutions, ok, latency_ms, messages, fk_poses = self._solve_path(
                path, current_joints, arm_angle, dof
            )
            if self._stop:
                self.logger.warn("收到中断，停止扫描，不执行运动")
                return False

            # 先计算与起点无关的可行性（IK 失败/跳变/FK/限位）；存在阻断项时
            # 不预定位，避免把臂白移到球面起点后再拒绝执行。
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
                include_start_gap=False,
            )

            # 起点离当前 TCP 太远时，先用 move_joint 把机械臂送到球面起点，
            # 避免 blend 段在关节空间做长距离过渡（TCP 走线不可控）。
            prepose_info: dict = {
                "enabled": bool(self.args.execute and self.args.prepose),
                "required": False,
                "performed": False,
            }
            if self.args.execute and self.args.prepose:
                initial_gap = start_gap_metrics(path, current_pose, current_joints, solutions, ok)
                needs_prepose = (
                    initial_gap["position_m"] > self.args.max_start_gap
                    or initial_gap["rotation_deg"] > self.args.max_start_rot
                )
                prepose_info["required"] = bool(needs_prepose)
                if needs_prepose and metrics["blocking"]:
                    self.logger.warn("存在与起点无关的阻断项，跳过预定位，避免无效移动")
                elif needs_prepose:
                    current_joints, current_pose, prepose_info, abort = self._prepose_if_needed(
                        path,
                        current_pose,
                        current_joints,
                        solutions,
                        ok,
                        dof,
                        metrics,
                        prepose_info,
                    )
                    prepose_info["initial_gap_m"] = initial_gap["position_m"]
                    prepose_info["initial_rotation_deg"] = initial_gap["rotation_deg"]
                    if abort:
                        self._apply_start_gap_checks(
                            metrics, path, current_pose, current_joints, solutions, ok
                        )
                        metrics["selection"] = self._selection
                        metrics["prepose"] = prepose_info
                        self._write_outputs(
                            path, solutions, ok, latency_ms, messages, fk_poses, metrics
                        )
                        return False

            self._apply_start_gap_checks(metrics, path, current_pose, current_joints, solutions, ok)
            exec_solutions = _fill_missing_solutions(solutions, ok, current_joints)
            timeline = build_timeline(
                path,
                exec_solutions,
                self.args.speed,
                current_joints,
                self.args.blend_time,
                self.args.jump_threshold,
                self.args.allow_jumps or self.args.force,
                self.args.blend_jump_time,
            )
            metrics["selection"] = self._selection
            metrics["prepose"] = prepose_info
            self._print_precheck(metrics)

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
                    self._execute(path, exec_solutions, timeline, current_joints, dof)
                    executed = True
                except Exception as exc:
                    self.logger.error(f"执行失败: {exc}")
                    metrics["execution_error"] = str(exc)
                metrics["actual"] = self._tracking_metrics(path, timeline, dof)
                self._print_tracking(metrics["actual"])

            metrics["executed"] = executed
            self._write_outputs(path, solutions, ok, latency_ms, messages, fk_poses, metrics)
            return True
        except KeyboardInterrupt:
            self.logger.warn("收到 Ctrl-C，正在停止")
            return False
        finally:
            self._stop = True

    def _build_path(
        self,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        arm_angle: float,
        dof: int,
    ):
        """自动选球（默认）或使用显式球心，构造两个互相垂直的大圆。"""
        args = self.args
        position = current_pose[:3]
        rpy = current_pose[3:]
        self._selection = {"source": "fixed-below-tcp"}
        if args.center is not None:
            center = np.asarray(args.center, dtype=float)
            e3 = np.array([0.0, 0.0, 1.0])
            e1 = np.array([1.0, 0.0, 0.0])
            e2 = np.array([0.0, 1.0, 0.0])
            first = center + args.radius * e3
            gap = float(np.linalg.norm(first - position))
            if gap > args.max_start_gap:
                target = f"轨迹首点 {np.round(first, 4).tolist()} 距当前 TCP {gap:.3f} m"
                if args.blend_time <= 0.0:
                    self.logger.warn(f"{target}，且 --blend-time 为 0，首帧直接跳到首点解")
                else:
                    self.logger.warn(f"{target}，blend 段（{args.blend_time:.2f}s）会先做关节过渡")
            self._selection = {
                "source": "explicit-center",
                "center_m": center.tolist(),
                "radius_m": args.radius,
                "start_gap_m": gap,
            }
        else:
            selection = None
            if args.auto_select:
                selection = self._auto_select(current_pose, current_joints, arm_angle, dof)
            if selection is not None:
                center = selection["center"]
                direction = position - center
                e3 = (
                    unit(direction)
                    if float(np.linalg.norm(direction)) > 1e-9
                    else np.array([0.0, 0.0, 1.0])
                )
                # 极轴指向当前 TCP：球面首点就是球面上离当前 TCP 最近的点
                e1, e2 = perpendicular_basis(e3)
                self._selection = selection
            else:
                e3 = np.array([0.0, 0.0, 1.0])
                e1 = np.array([1.0, 0.0, 0.0])
                e2 = np.array([0.0, 1.0, 0.0])
                center = position - args.radius * e3

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
        self.logger.info(
            f"关节数 {dof}；球心={np.round(path.center, 4).tolist()} m，"
            f"R={path.radius:.3f} m，首点距当前 TCP "
            f"{np.linalg.norm(path.positions[0] - position):.4f} m"
        )
        return path, rpy

    def _auto_select(
        self,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        arm_angle: float,
        dof: int,
    ):
        """按灵巧空间体素数据选球，并用控制器 IK 粗校验候选。

        返回 None 表示数据不可用或没有候选，调用方回退到旧行为。
        """
        args = self.args
        data_path = Path(args.workspace_data).expanduser()
        if not data_path.is_file():
            self.logger.warn(
                f"工作空间数据不存在: {data_path}；回退到 当前 TCP − R·z。"
                "请先运行 scripts/workspace_measure/dexterous_workspace.py --save-data"
            )
            return None
        try:
            workspace = load_workspace_data(data_path)
        except Exception as exc:
            self.logger.warn(f"读取工作空间数据失败: {exc}；回退到 当前 TCP − R·z")
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

        position = current_pose[:3]
        candidates = sphere_candidates(
            workspace,
            radius=args.radius,
            tcp_position=position,
            max_start_gap=args.auto_start_gap,
            limit=args.auto_candidates,
            directions=args.auto_directions,
        )
        if not candidates:
            self.logger.warn(
                f"灵巧空间内没有满足 R={args.radius:.3f} m、起点偏差 ≤ "
                f"{args.auto_start_gap:.2f} m 的球心；回退到 当前 TCP − R·z"
            )
            return None
        self.logger.info(
            f"灵巧空间候选 {len(candidates)} 个（数据: {data_path.name}, "
            f"臂型 {workspace.arm_type or '未知'}, 体素 {workspace.step:.3f} m），"
            "开始控制器 IK 粗校验"
        )

        probe_limits = None
        if args.check_limits:
            probe_limits = self._joint_limits(dof)

        best = None
        best_score = None
        chosen_index = 0
        probes = []
        for index, candidate in enumerate(candidates):
            direction = position - candidate.center
            if float(np.linalg.norm(direction)) < 1e-9:
                direction = np.array([0.0, 0.0, 1.0])
            e3 = unit(direction)
            e1, e2 = perpendicular_basis(e3)
            verify_points = max(4, int(args.auto_verify_samples))
            step = 2.0 * math.pi * candidate.radius / max(2, verify_points // 2)
            probe_path = build_sphere_path(
                center=candidate.center,
                radius=candidate.radius,
                e1=e1,
                e2=e2,
                e3=e3,
                step=step,
                orient=args.orient,
                base_rpy=current_pose[3:],
            )
            probe_solutions, probe_ok, _, _, _ = self._solve_path(
                probe_path,
                current_joints,
                arm_angle,
                dof,
                check_fk=False,
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
            if clean:
                break

        if best is None:
            return None
        best_success = float(probes[chosen_index]["probe_success"])
        best_step = float(probes[chosen_index]["probe_max_step_rad"])
        best_violations = int(probes[chosen_index]["probe_limit_violations"])
        if best_success < 1.0 or best_step > PROBE_MAX_STEP_RAD or best_violations:
            self.logger.warn(
                f"候选球粗校验未完全通过（成功率 {best_success * 100:.0f}%，"
                f"max|dq|={best_step:.3f} rad，限位越界 {best_violations} 点）；"
                "完整扫描可能仍会被预检阻断"
            )
        self.logger.info(
            f"选中球心={np.round(best.center, 4).tolist()} m（候选 #{chosen_index}，"
            f"min_dex={best.min_dexterity:.3f}, 起点偏差={best.start_gap * 1000.0:.0f} mm, "
            f"粗校验={best_success * 100:.0f}%, max|dq|={best_step:.3f} rad, "
            f"限位越界={best_violations}）"
        )
        self.logger.warn(
            "灵巧空间来自 FK 蒙特卡洛，不含自碰撞检查；首次 --execute 前请人工确认球面路径安全"
        )
        return {
            "source": "dexterous-workspace",
            "workspace_data": str(data_path),
            "arm_type": workspace.arm_type,
            "voxel_step_m": workspace.step,
            "tau_rad": workspace.tau_rad,
            "center": best.center,
            "radius_m": best.radius,
            "min_dexterity": best.min_dexterity,
            "mean_dexterity": best.mean_dexterity,
            "start_gap_m": best.start_gap,
            "candidate_index": chosen_index,
            "candidate_probes": probes,
        }

    # ------------------------------------------------------------------
    # 逆解扫描
    # ------------------------------------------------------------------

    def _solve_path(
        self,
        path: SpherePath,
        current_joints: np.ndarray,
        arm_angle: float,
        dof: int,
        check_fk: bool | None = None,
        verbose: bool = True,
    ):
        count = len(path.positions)
        solutions = np.full((count, dof), np.nan)
        latency_ms = np.full(count, np.nan)
        messages = [""] * count
        fk_poses = np.full((count, 6), np.nan)
        reference = current_joints.copy()
        check_fk = self.args.check_fk if check_fk is None else bool(check_fk)

        for index in range(count):
            if self._stop:
                break
            request = self._Ik.Request()
            request.frame = 1
            request.pose.header.frame_id = "base_link"
            request.pose.position.x = float(path.positions[index, 0])
            request.pose.position.y = float(path.positions[index, 1])
            request.pose.position.z = float(path.positions[index, 2])
            request.pose.rpy.x = float(path.rpys[index, 0])
            request.pose.rpy.y = float(path.rpys[index, 1])
            request.pose.rpy.z = float(path.rpys[index, 2])
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
                messages[index] = str(exc)
                response = None
            latency_ms[index] = (time.perf_counter() - start) * 1000.0

            if response is None or not response.success:
                if response is not None:
                    messages[index] = response.message
                if verbose:
                    self._log_failed_pose(index, count, path, messages[index])
                continue
            joints = np.asarray(response.joints[:dof], dtype=float)
            if not np.all(np.isfinite(joints)):
                messages[index] = "ik 返回非有限值"
                if verbose:
                    self._log_failed_pose(index, count, path, messages[index])
                continue
            solutions[index] = joints
            reference = joints

            if check_fk:
                fk_request = self._Fk.Request()
                fk_request.frame = 1
                fk_request.joints = joints.tolist()
                fk_request.source = 0
                try:
                    fk_response = self._call(self.fk_cli, fk_request, "fk")
                except Exception as exc:
                    messages[index] = f"fk 失败: {exc}"
                    fk_response = None
                if fk_response is not None and fk_response.success:
                    fk_poses[index] = [
                        fk_response.pose.position.x,
                        fk_response.pose.position.y,
                        fk_response.pose.position.z,
                        fk_response.pose.rpy.x,
                        fk_response.pose.rpy.y,
                        fk_response.pose.rpy.z,
                    ]

            if verbose and ((index + 1) % 100 == 0 or index + 1 == count):
                print(f"\r逆解扫描 {index + 1}/{count}", end="", flush=True)
        if verbose:
            print()
        ok = np.isfinite(solutions).all(axis=1)
        if verbose:
            self.logger.info(f"逆解完成：成功 {int(ok.sum())}/{count}")
        return solutions, ok, latency_ms, messages, fk_poses

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
        include_start_gap: bool = True,
    ) -> dict:
        count = len(solutions)
        metrics: dict = {"blocking": []}
        if include_start_gap:
            self._apply_start_gap_checks(metrics, path, current_pose, current_joints, solutions, ok)
        ik_metrics = {
            "samples": count,
            "success": int(ok.sum()),
            "fail": int(count - ok.sum()),
            "fail_indices": np.where(~ok)[0].tolist()[:20],
        }
        finite_latency = latency_ms[np.isfinite(latency_ms)]
        if finite_latency.size:
            ik_metrics["latency_ms"] = {
                "median": float(np.median(finite_latency)),
                "p95": float(np.percentile(finite_latency, 95)),
                "max": float(np.max(finite_latency)),
                "mean": float(np.mean(finite_latency)),
            }

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
                "如为弧段切换，可提高 --max-joint-step 并加 --allow-jumps"
            )
        if ik_metrics.get("jump_count", 0) > 0 and not self.args.allow_jumps:
            metrics["blocking"].append(
                f"{ik_metrics['jump_count']} 处关节跳变超过 {self.args.jump_threshold:.3f} rad，"
                "如需平滑过渡请加 --allow-jumps（必要时提高 --max-joint-step）"
            )

        # FK 残差
        if self.args.check_fk:
            fk_ok = np.isfinite(fk_poses).all(axis=1)
            if fk_ok.any():
                residual_pos = (
                    np.linalg.norm(fk_poses[fk_ok, :3] - path.positions[fk_ok], axis=1) * 1000.0
                )
                residual_rot = np.array(
                    [rotation_error_deg(fk_poses[i, 3:], path.rpys[i]) for i in np.where(fk_ok)[0]]
                )
                metrics["fk"] = {
                    "samples": int(fk_ok.sum()),
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
                    metrics["blocking"].append(
                        f"FK 姿态残差最大 {metrics['fk']['rot_deg']['max']:.2f}° "
                        f"超过 {self.args.max_residual_deg:.2f}°"
                    )
            else:
                metrics["fk"] = {"samples": 0}

        # 关节限位
        if self.args.check_limits:
            limits = self._joint_limits(dof)
            if limits is not None:
                low, high = limits
                valid = ok & np.isfinite(solutions).all(axis=1)
                violations = []
                if valid.any():
                    below = solutions[valid] < (low - JOINT_LIMIT_MARGIN_RAD)
                    above = solutions[valid] > (high + JOINT_LIMIT_MARGIN_RAD)
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
        return metrics

    def _apply_start_gap_checks(
        self,
        metrics: dict,
        path,
        current_pose: np.ndarray,
        current_joints: np.ndarray,
        solutions: np.ndarray,
        ok: np.ndarray,
    ) -> None:
        """用给定当前位姿计算首点跨度并追加阻断项（可在预定位后重复调用）。"""
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
        elif self.args.blend_time > 0.0:
            needed = joint_gap / self.args.blend_time
            if needed > math.radians(self.args.vmax_deg):
                metrics["blocking"].append(
                    f"首点在 blend 段需 {math.degrees(needed):.0f}°/s，超过 ServoJ "
                    f"--vmax-deg {self.args.vmax_deg:.0f}°/s（加大 --blend-time）"
                )

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
                    # 已落后一个以上节拍：从当前时刻重新对齐，避免连续补发造成抖动
                    window_missed += int(lag // period)
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
                            f"错过 {window_missed} 个节拍"
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
                if kind.startswith("blend"):
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
            if kind.startswith("blend"):
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
                f"起点偏差 {selection.get('start_gap_m', 0.0) * 1000.0:.0f} mm"
            )
        elif selection.get("source") == "explicit-center":
            lines.append("  选球: 显式 --center")
        elif selection:
            lines.append("  选球: 当前 TCP − R·z（自动选球未启用或不可用）")
        prepose = metrics.get("prepose") or {}
        if prepose.get("performed"):
            gap_mm = float(prepose.get("initial_gap_m", 0.0)) * 1000.0
            lines.append(f"  预定位: 已 MoveJ 到球面起点（原偏差 {gap_mm:.0f} mm）")
        elif prepose.get("error"):
            lines.append(f"  预定位失败: {prepose['error']}")
        latency = ik.get("latency_ms")
        if latency:
            lines.append(
                f"  延迟: median {latency['median']:.2f} ms, p95 {latency['p95']:.2f} ms, "
                f"max {latency['max']:.2f} ms"
            )
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
            lines.append(
                f"  FK 残差: 位置 max {fk['pos_mm']['max']:.3f} mm "
                f"(p95 {fk['pos_mm']['p95']:.3f} mm), 姿态 max {fk['rot_deg']['max']:.3f}°"
            )
        limits = metrics.get("limits")
        if limits:
            text = "无" if not limits["violations"] else ", ".join(limits["violations"])
            lines.append(f"  关节限位: {text}")
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
            axes.set_title("sphere IK bench")
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

    directions = fibonacci_directions(128)
    check(
        directions.shape == (128, 3)
        and float(np.max(np.abs(np.linalg.norm(directions, axis=1) - 1.0))) < 1e-12,
        "Fibonacci 方向全部为单位向量",
    )
    e1, e2 = perpendicular_basis(np.array([0.3, -0.5, 0.8]))
    basis = np.column_stack([e1, e2, unit(np.array([0.3, -0.5, 0.8]))])
    check(
        float(np.max(np.abs(basis.T @ basis - np.eye(3)))) < 1e-12
        and float(np.linalg.det(basis) - 1.0) < 1e-12,
        "正交基右手指向正确",
    )

    mask = np.zeros((5, 5, 5), dtype=bool)
    mask[1:4, 1:4, 1:4] = True
    eroded = _erode(mask)
    check(
        int(eroded.sum()) == 1 and bool(eroded[2, 2, 2]),
        f"3×3×3 腐蚀结果 {int(eroded.sum())} 个内部体素",
    )

    with tempfile.TemporaryDirectory() as temp_dir:
        temp = Path(temp_dir)
        step = 0.02
        values = np.arange(-3, 4) * step
        xx, yy, zz = np.meshgrid(values, values, values + 0.30, indexing="ij")
        fake_centers = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        np.savez(
            temp / "fake_ws.npz",
            centers=fake_centers,
            dexterity=np.full(fake_centers.shape[0], 0.8),
            step=step,
            tau=0.35,
            min_samples=100,
            samples=1000,
            arm_type="fake",
        )
        workspace = load_workspace_data(temp / "fake_ws.npz")
        check(workspace.interior.shape == (7, 7, 7), "工作空间网格重建为 7×7×7")
        check(int(workspace.interior.sum()) == 125, f"内部体素 {int(workspace.interior.sum())} 个")
        fake_tcp = np.array([0.30, 0.0, 0.30])
        candidates = sphere_candidates(
            workspace, radius=0.03, tcp_position=fake_tcp, max_start_gap=0.30, limit=8
        )
        check(len(candidates) > 0, f"合成工作空间得到 {len(candidates)} 个候选球")
        if candidates:
            best = candidates[0]
            cell = np.round((best.center - workspace.origin) / workspace.step).astype(int)
            check(
                bool(workspace.interior[cell[0], cell[1], cell[2]])
                and abs(best.min_dexterity - 0.8) < 1e-12,
                f"最优候选球心 {np.round(best.center, 3).tolist()} 位于内部体素",
            )
            axis = unit(fake_tcp - best.center)
            plane_e1, plane_e2 = perpendicular_basis(axis)
            auto_path = build_sphere_path(
                best.center,
                best.radius,
                plane_e1,
                plane_e2,
                axis,
                0.01,
                "fixed",
                np.array([0.0, 0.0, 0.0]),
            )
            check(
                abs(float(np.linalg.norm(auto_path.positions[0] - best.center)) - best.radius)
                < 1e-12,
                "自动选球的首点位于球面",
            )
            check(
                abs(float(np.linalg.norm(auto_path.positions[0] - fake_tcp)) - best.start_gap)
                < 1e-9,
                "首点与当前 TCP 的距离等于候选 start_gap",
            )

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
        list(dict.fromkeys(segments)) == ["circle_xz", "circle_yz"],
        "段顺序为 circle_xz → circle_yz",
    )
    on_xz = path.positions[segments == "circle_xz"]
    on_yz = path.positions[segments == "circle_yz"]
    check(len(on_xz) > 0 and len(on_yz) > 0, f"两段均非空: {len(on_xz)} / {len(on_yz)}")
    if len(on_xz) and len(on_yz):
        check(
            float(np.max(np.abs((on_xz - center) @ e2))) < 1e-9,
            f"xz 面圆 {len(on_xz)} 点全部落在 xz 平面",
        )
        check(
            float(np.max(np.abs((on_yz - center) @ e1))) < 1e-9,
            f"yz 面圆 {len(on_yz)} 点全部落在 yz 平面",
        )
        north = center + radius * e3
        check(
            float(np.linalg.norm(path.positions[0] - north)) < 1e-12,
            "轨迹首点为球心 + R·e3（北极点）",
        )
        check(
            float(np.linalg.norm(on_yz[0] - north)) < 1e-12,
            "yz 面圆首点同为北极点",
        )
    expected = 2 * 2 * math.pi * radius
    check(
        abs(path.length - expected) / expected < 0.01,
        f"轨迹长度 {path.length:.4f} m vs 理论 {expected:.4f} m",
    )
    check(len(path.positions) > 500, f"采样点数 {len(path.positions)}")
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
    for name, oriented_path in (("radial", radial), ("inward", inward)):
        angles = [
            rotation_error_deg(oriented_path.rpys[i - 1], oriented_path.rpys[i])
            for i in range(1, len(oriented_path.rpys))
        ]
        check(
            max(angles) < 5.0,
            f"{name} 相邻姿态连续（含接缝）max={max(angles):.3f}°",
        )
    check(
        rotation_error_deg(inward.rpys[0], np.array([math.pi, 0.0, 0.0])) < 1e-6,
        "inward 首点姿态与当前 TCP 一致（tool X 平行移动的种子）",
    )

    # 首点跨度：fixed/inward 与当前姿态一致（tool Z = -e3），radial 反向
    now_pose = np.array([0.30, 0.0, 0.40, math.pi, 0.0, 0.0])
    now_center = now_pose[:3] - radius * e3
    now_joints = np.zeros(6)
    for orient_name, expected_rot in (("fixed", 0.0), ("inward", 0.0), ("radial", 180.0)):
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
        check(
            abs(gap["position_m"]) < 1e-12
            and abs(gap["rotation_deg"] - expected_rot) < 1e-6
            and abs(gap.get("joint_rad", -1.0)) < 1e-12,
            f"首点跨度 {orient_name}: 位置 {gap['position_m']:.4f} m, "
            f"姿态 {gap['rotation_deg']:.1f}°",
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
        abs(gap_shifted["position_m"] - 0.15) < 1e-9,
        f"球心下移 0.15 m 时首点位置跨度 {gap_shifted['position_m']:.3f} m",
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
    timeline = build_timeline(path, solutions, 0.02, current, 0.5, 0.05, False, 0.5)
    check(timeline.total > 0.0, f"时间轴总长 {timeline.total:.2f} s")
    joints_zero, _, _, kind_zero = timeline.sample(0.0)
    check(np.allclose(joints_zero, current) and kind_zero == "blend_in", "时间轴 0 时刻为当前关节")
    last_joints, _, _, _ = timeline.sample(timeline.total)
    check(np.allclose(last_joints, solutions[-1]), "时间轴终点为最后一个解")

    print("SELF-TEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="线框球轨迹控制器逆解性能评估与点云导出",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--radius", type=float, default=0.05, help="球半径（m）；自动选球按该半径搜索球心"
    )
    parser.add_argument(
        "--center",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="显式球心（基坐标系，m）；缺省自动从灵巧空间数据选球",
    )
    parser.add_argument(
        "--workspace-data",
        default=str(DEFAULT_WORKSPACE_DATA),
        help="灵巧工作空间 .npz（dexterous_workspace.py --save-data 生成）",
    )
    parser.add_argument(
        "--no-auto",
        dest="auto_select",
        action="store_false",
        default=True,
        help="禁用自动选球，回退到「球心 = 当前 TCP − 半径·z」",
    )
    parser.add_argument(
        "--auto-start-gap",
        type=float,
        default=DEFAULT_AUTO_START_GAP_M,
        help="自动选球允许的球面起点偏差上限（m）",
    )
    parser.add_argument(
        "--auto-candidates",
        type=int,
        default=DEFAULT_AUTO_CANDIDATES,
        help="最多用控制器 IK 粗校验的候选球数量",
    )
    parser.add_argument(
        "--auto-directions",
        type=int,
        default=DEFAULT_AUTO_DIRECTIONS,
        help="离线几何检查时球面覆盖的采样方向数",
    )
    parser.add_argument(
        "--auto-verify-samples",
        type=int,
        default=DEFAULT_AUTO_VERIFY_SAMPLES,
        help="控制器 IK 粗校验时两圆合计的采样点数",
    )
    parser.add_argument(
        "--prepose",
        dest="prepose",
        action="store_true",
        default=True,
        help="执行时起点偏差超限则先用 move_joint 移动到球面起点",
    )
    parser.add_argument(
        "--no-prepose",
        dest="prepose",
        action="store_false",
        help="禁止自动移动到球面起点（起点偏差超限时预检会阻断执行）",
    )
    parser.add_argument(
        "--prepose-velocity",
        type=float,
        default=20.0,
        help="预定位 MoveJ 速度（%%）",
    )
    parser.add_argument(
        "--prepose-timeout",
        type=float,
        default=60.0,
        help="预定位 MoveJ 超时（s）",
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
    parser.add_argument("--form", type=int, default=0, help="逆解形态（0=自动，1~8 指定分支）")
    parser.add_argument("--execute", action="store_true", help="实际执行 ServoJ 运动")
    parser.add_argument(
        "--allow-arm-mismatch",
        action="store_true",
        help="工作空间数据臂型与 tl_driver arm_type 不一致时仍继续（默认报错）",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="忽略预检阻断项仍然执行，并对关节跳变做平滑过渡（危险；不覆盖预定位失败）",
    )
    parser.add_argument(
        "--blend-time", type=float, default=1.0, help="从当前关节过渡到首点的时间（s）"
    )
    parser.add_argument("--hold-time", type=float, default=1.0, help="轨迹结束后保持时间（s）")
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
    parser.add_argument("--blend-jump-time", type=float, default=0.5, help="单次跳变过渡时间（s）")
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
    parser.add_argument("--vmax-deg", type=float, default=200.0, help="ServoJ 最大关节速度（°/s）")
    parser.add_argument(
        "--amax-deg", type=float, default=800.0, help="ServoJ 最大关节加速度（°/s²）"
    )
    parser.add_argument(
        "--jmax-deg", type=float, default=4800.0, help="ServoJ 最大加加速度（°/s³）"
    )
    parser.add_argument(
        "--speed-percent", type=float, default=0.0, help="执行前设置全局速度（%%，0=不修改）"
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
