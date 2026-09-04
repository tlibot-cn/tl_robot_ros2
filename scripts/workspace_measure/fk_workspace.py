#!/usr/bin/env python3
"""
机械臂工作空间蒙特卡洛分析 + 包络图

- 从 URDF 加载运动学链
- 蒙特卡洛采样 100k 个关节构型 → 计算末端位置
- 输出工作空间统计
- 绘制 3D 包络图 + XZ 截面 + XY 截面 + YZ 截面
"""

import argparse
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
import xml.etree.ElementTree as ET
from pathlib import Path

# ── 1. 从 URDF 提取关节信息 ──────────────────────────────────────────────


def parse_urdf_joints(urdf_path):
    """从 URDF 提取运动学链：关节名、固定偏移、旋转轴、关节限位"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # 按 URDF 中的出现顺序收集关节
    joints = []
    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name")
        jtype = joint_elem.get("type")
        if jtype != "revolute":
            continue

        # 固定偏移：origin
        origin = joint_elem.find("origin")
        if origin is not None:
            xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in origin.get("rpy", "0 0 0").split()])
        else:
            xyz = np.zeros(3)
            rpy = np.zeros(3)

        # 旋转轴
        axis_elem = joint_elem.find("axis")
        if axis_elem is not None:
            axis = np.array([float(v) for v in axis_elem.get("xyz", "0 0 1").split()])
        else:
            axis = np.array([0, 0, 1])

        # 限位
        limit = joint_elem.find("limit")
        if limit is not None:
            lower = float(limit.get("lower", -np.pi))
            upper = float(limit.get("upper", np.pi))
        else:
            lower, upper = -np.pi, np.pi

        joints.append(
            {
                "name": name,
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis / np.linalg.norm(axis),
                "lower": lower,
                "upper": upper,
            }
        )

    return joints


# ── 2. 运动学工具 ────────────────────────────────────────────────────────


def rpy_to_rot(roll, pitch, yaw):
    """XYZ 欧拉角 → 旋转矩阵（内旋，scipy 约定的 'XYZ'）"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle_rot(axis, angle):
    """轴角 → 旋转矩阵 (Rodrigues)"""
    kx, ky, kz = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    v = 1 - c
    return np.array(
        [
            [kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
            [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
            [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
        ]
    )


def make_homogeneous(R, t):
    """旋转矩阵 R (3x3) + 平移 t (3,) → 4x4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def fk_single(joints, q):
    """计算单一关节构型的末端位置 (link7 原点在 base 坐标系下)"""
    T = np.eye(4)  # 起始于 link0
    for j, qv in zip(joints, q):
        # 1. 固定偏移 (origin)
        R_fixed = rpy_to_rot(*j["rpy"])
        T_fixed = make_homogeneous(R_fixed, j["xyz"])
        T = T @ T_fixed

        # 2. 绕关节轴旋转 qv
        R_joint = axis_angle_rot(j["axis"], qv)
        T_joint = make_homogeneous(R_joint, np.zeros(3))
        T = T @ T_joint

    return T[:3, 3]  # 末端位置


# ── 3. 蒙特卡洛采样 ──────────────────────────────────────────────────────


def sample_workspace(joints, n_samples=100000, seed=42):
    """在关节空间均匀采样，返回末端位置点云"""
    rng = np.random.default_rng(seed)
    limits = np.array([[j["lower"], j["upper"]] for j in joints])
    n_joints = len(joints)

    # 向量化采样
    q_samples = rng.uniform(limits[:, 0], limits[:, 1], size=(n_samples, n_joints))

    # 逐点计算 FK（numpy 向量化困难，因为每步依赖上一步的变换）
    positions = np.zeros((n_samples, 3))
    for i in range(n_samples):
        positions[i] = fk_single(joints, q_samples[i])

    return positions


# ── 4. 包络计算 ──────────────────────────────────────────────────────────


def compute_envelope(pts, n_theta=72, n_phi=36):
    """用球坐标网格样本计算径向包络边界"""
    # 转球坐标
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arctan2(y, x)  # 方位角 [-π, π]
    phi = np.arccos(z / np.clip(r, 1e-10, None))  # 仰角 [0, π]

    # 网格划分：每个 (theta, phi) bin 取最大半径
    theta_bins = np.linspace(-np.pi, np.pi, n_theta + 1)
    phi_bins = np.linspace(0, np.pi, n_phi + 1)
    theta_idx = np.digitize(theta, theta_bins) - 1
    phi_idx = np.digitize(phi, phi_bins) - 1

    # 剔除越界索引
    valid = (theta_idx >= 0) & (theta_idx < n_theta) & (phi_idx >= 0) & (phi_idx < n_phi)
    theta_idx, phi_idx, r_v = theta_idx[valid], phi_idx[valid], r[valid]

    # 每个 bin 取最大半径
    envelope = {}
    for ti, pi, rv in zip(theta_idx, phi_idx, r_v):
        key = (ti, pi)
        if key not in envelope or rv > envelope[key]:
            envelope[key] = rv

    # 转回笛卡尔坐标
    r_env = np.array(list(envelope.values()))
    idx = np.array(list(envelope.keys()))
    t_centers = (theta_bins[:-1] + theta_bins[1:]) / 2
    p_centers = (phi_bins[:-1] + phi_bins[1:]) / 2
    theta_pts = t_centers[idx[:, 0]]
    phi_pts = p_centers[idx[:, 1]]

    x_env = r_env * np.sin(phi_pts) * np.cos(theta_pts)
    y_env = r_env * np.sin(phi_pts) * np.sin(theta_pts)
    z_env = r_env * np.cos(phi_pts)

    return np.column_stack([x_env, y_env, z_env])


# ── 5. 主程序 ────────────────────────────────────────────────────────────


def main():
    arm_types = [
        "tcb605",
        "tcb605f",
        "tcb605l",
        "tcb605lv",
        "tcb605v",
        "tcb610",
        "tcb610v",
        "tcb705",
        "tcb705f",
        "tcb705l",
        "tcb705lv",
        "tcb705v",
        "tcb710",
        "tcb710v",
    ]
    parser = argparse.ArgumentParser(description="机械臂工作空间蒙特卡洛分析 + 包络图")
    parser.add_argument(
        "--arm-type",
        required=True,
        choices=arm_types,
        help="机械臂型号（小写），对应 tl_description/urdf/<arm_type>.urdf",
    )
    args = parser.parse_args()
    arm_type = args.arm_type

    script_dir = Path(__file__).resolve().parent
    urdf_path = script_dir.parent.parent / "src" / "tl_description" / "urdf" / f"{arm_type}.urdf"
    results_dir = script_dir / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    if not urdf_path.exists():
        print(f"❌ URDF 文件不存在: {urdf_path}")
        return

    print(f"📄 加载 URDF: {urdf_path}")
    joints = parse_urdf_joints(str(urdf_path))
    n_joints = len(joints)
    print(f"🔧 检测到 {n_joints} 个旋转关节:")
    for j in joints:
        print(
            f"   {j['name']:8s}  range: [{j['lower']:.4f}, {j['upper']:.4f}] rad  "
            f"axis: ({j['axis'][0]:.2f}, {j['axis'][1]:.2f}, {j['axis'][2]:.2f})"
        )

    # ── 采样 ──
    n_samples = 150000
    print(f"\n🔄 蒙特卡洛采样 {n_samples} 个构型...")
    pts = sample_workspace(joints, n_samples=n_samples)

    # ── 统计 ──
    bounds = {
        "x": (pts[:, 0].min(), pts[:, 0].max()),
        "y": (pts[:, 1].min(), pts[:, 1].max()),
        "z": (pts[:, 2].min(), pts[:, 2].max()),
    }
    radii = np.linalg.norm(pts, axis=1)
    max_reach = radii.max()
    mean_reach = radii.mean()

    x_range = bounds["x"][1] - bounds["x"][0]
    y_range = bounds["y"][1] - bounds["y"][0]
    z_range = bounds["z"][1] - bounds["z"][0]

    # 近似体积（轴向包围盒）
    bbox_volume = x_range * y_range * z_range

    print(f"\n📐 工作空间统计 (单位: 米)")
    print(f"{'='*50}")
    print(f"  X 范围:    [{bounds['x'][0]:+.4f}, {bounds['x'][1]:+.4f}]  (跨度 {x_range:.4f})")
    print(f"  Y 范围:    [{bounds['y'][0]:+.4f}, {bounds['y'][1]:+.4f}]  (跨度 {y_range:.4f})")
    print(f"  Z 范围:    [{bounds['z'][0]:+.4f}, {bounds['z'][1]:+.4f}]  (跨度 {z_range:.4f})")
    print(f"  最大半径:  {max_reach:.4f}")
    print(f"  平均半径:  {mean_reach:.4f}")
    print(f"  包围盒体积: {bbox_volume:.4f} m³")

    # ── 包络 ──
    print("\n📦 计算径向包络...")
    env_pts = compute_envelope(pts, n_theta=60, n_phi=30)
    print(f"   包络点数量: {len(env_pts)}")

    # ── 绘图 ──
    print("\n🎨 绘制包络图...")
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(2, 2)

    # ── 图1: 3D 包络 ──
    ax1 = fig.add_subplot(gs[0], projection="3d")
    ax1.scatter(
        pts[::20, 0],
        pts[::20, 1],
        pts[::20, 2],
        c=radii[::20],
        cmap="viridis",
        s=0.3,
        alpha=0.15,
        marker=".",
    )

    # 绘制包络面（三角剖分）
    try:
        hull = ConvexHull(env_pts)
        for simplex in hull.simplices:
            tri = env_pts[simplex]
            poly = Poly3DCollection(
                [tri], alpha=0.08, facecolor="steelblue", edgecolor="navy", linewidth=0.3
            )
            ax1.add_collection3d(poly)
    except Exception as e:
        print(f"   包络面警告: {e}")

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title(f"{arm_type.upper()} Workspace Envelope (3D)\n({n_samples} samples)")
    ax1.set_box_aspect([1, 1, 1])

    # ── 图2: XZ 截面 ──
    ax2 = fig.add_subplot(gs[1])
    slice_xz_mask = np.abs(pts[:, 1]) < 0.05
    slice_xz = pts[slice_xz_mask]
    slice_xz_r = radii[slice_xz_mask]

    ax2.scatter(slice_xz[:, 0], slice_xz[:, 2], c=slice_xz_r, cmap="viridis", s=0.5, alpha=0.3)

    if len(slice_xz) > 3:
        try:
            hull_xz = ConvexHull(slice_xz[:, [0, 2]])
            for simplex in hull_xz.simplices:
                xz = slice_xz[simplex, [0, 2]]
                ax2.plot(xz[:, 0], xz[:, 1], "r-", lw=0.8, alpha=0.6)
        except Exception:
            pass

    ax2.plot(0, 0, "ks", markersize=8, label="Base")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")
    ax2.set_title("XZ Slice (|Y| < 0.05 m)")
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    # ── 图3: XY 截面（多层 Z） ──
    ax3 = fig.add_subplot(gs[2])
    z_levels = [0.0, 0.2, 0.4, 0.6, 0.8]
    colors = plt.cm.plasma(np.linspace(0.2, 0.9, len(z_levels)))

    for z, c in zip(z_levels, colors):
        tol = 0.03
        mask = np.abs(pts[:, 2] - z) < tol
        slice_xy = pts[mask]
        if len(slice_xy) < 5:
            continue
        ax3.scatter(slice_xy[:, 0], slice_xy[:, 1], c=[c], s=0.5, alpha=0.2, label=f"Z={z:.1f}m")
        # 截面包络
        try:
            hull_xy = ConvexHull(slice_xy[:, [0, 1]])
            for simplex in hull_xy.simplices:
                xy = slice_xy[simplex, [0, 1]]
                ax3.plot(xy[:, 0], xy[:, 1], "-", color=c, lw=1.2, alpha=0.8)
        except Exception:
            pass

    ax3.plot(0, 0, "ks", markersize=8, label="Base")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("XY Slices at Z heights")
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8)

    # ── 图4: YZ 截面 ──
    ax4 = fig.add_subplot(gs[3])
    slice_yz_mask = np.abs(pts[:, 0]) < 0.05
    slice_yz = pts[slice_yz_mask]
    slice_yz_r = radii[slice_yz_mask]

    ax4.scatter(slice_yz[:, 1], slice_yz[:, 2], c=slice_yz_r, cmap="viridis", s=0.5, alpha=0.3)

    if len(slice_yz) > 3:
        try:
            hull_yz = ConvexHull(slice_yz[:, [1, 2]])
            for simplex in hull_yz.simplices:
                yz = slice_yz[simplex, [1, 2]]
                ax4.plot(yz[:, 0], yz[:, 1], "r-", lw=0.8, alpha=0.6)
        except Exception:
            pass

    ax4.plot(0, 0, "ks", markersize=8, label="Base")
    ax4.set_xlabel("Y (m)")
    ax4.set_ylabel("Z (m)")
    ax4.set_title("YZ Slice (|X| < 0.05 m)")
    ax4.set_aspect("equal")
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=8)
    # ── 保存 ──
    output_path = results_dir / f"{arm_type}_workspace.png"
    plt.tight_layout()
    plt.savefig(str(output_path), dpi=200, bbox_inches="tight")
    print(f"\n✅ 图已保存: {output_path}")

    # 也输出统计数据到文件
    stats_path = results_dir / f"{arm_type}_workspace_stats.txt"
    with open(stats_path, "w") as f:
        f.write(f"{arm_type.upper()} Workspace Analysis\n")
        f.write(f"{'='*50}\n")
        f.write(f"Samples: {n_samples}\n")
        f.write(f"Joints: {n_joints}\n\n")
        f.write(f"X range: [{bounds['x'][0]:+.4f}, {bounds['x'][1]:+.4f}]  span: {x_range:.4f} m\n")
        f.write(f"Y range: [{bounds['y'][0]:+.4f}, {bounds['y'][1]:+.4f}]  span: {y_range:.4f} m\n")
        f.write(f"Z range: [{bounds['z'][0]:+.4f}, {bounds['z'][1]:+.4f}]  span: {z_range:.4f} m\n")
        f.write(f"Max reach: {max_reach:.4f} m\n")
        f.write(f"Mean reach: {mean_reach:.4f} m\n")
        f.write(f"Bounding box volume: {bbox_volume:.4f} m³\n")
    print(f"📊 统计已保存: {stats_path}")

    # 打印中英双语摘要
    print(f"\n{'='*50}")
    print(f"📋 摘要")
    print(f"{'='*50}")
    print(f"  X轴工作范围: {x_range*1000:.0f} mm")
    print(f"  Y轴工作范围: {y_range*1000:.0f} mm")
    print(f"  Z轴工作范围: {z_range*1000:.0f} mm")
    print(f"  最大工作半径: {max_reach*1000:.0f} mm")
    print(f"  平均工作半径: {mean_reach*1000:.0f} mm")


if __name__ == "__main__":
    main()
