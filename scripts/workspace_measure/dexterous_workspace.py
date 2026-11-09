#!/usr/bin/env python3
"""
机械臂灵巧工作空间 — FK 蒙特卡洛 + 体素姿态覆盖

不依赖 ROS / MoveIt：直接读 URDF，在关节空间均匀采样，批量正向运动学得到
末端位姿 (位置, 姿态)，按位置分箱后估计每个体素内可达姿态的覆盖范围。

定义与判定：
  - 灵巧工作空间 = 末端能以**任意姿态**到达的位置集合；
  - 体素灵巧度 d(v) = 1 - θ_max(v)/π，θ_max 是该体素内「最坏方向」的覆盖半径：
      θ_max(v) = max_{探针姿态 r} min_{样本 i∈v} arccos( 2·(q_i·q_r)² - 1 )
    即对每个探针姿态，取体素内最接近它的样本；再取最差探针。
  - θ_max ≤ --tau-deg 的体素计为灵巧点。τ 是方法固有的角容差：样本有限时
    覆盖半径存在约 K^(-1/3) 的量化下限（K 为该体素样本数），τ 取 10-20° 合理。

统计偏差说明：
  - 样本有限使 θ_max 偏大（保守、漏判）；体素内位置混合（样本来自不同位置）
    使 θ_max 偏小（乐观、误判）。净效应取决于 --step 与体素样本数；
  - 关节空间均匀采样 ≠ 工作空间均匀采样，稀疏体素用 --min-samples 剔除，
    被剔除的区域按「不可达」处理（保守）；
  - 结果给的是统计意义下的灵巧空间，不是逐位姿 IK 可解性证明；需要严格判据
    时用 dexterous_ik.py（MoveIt2 IK 基准）。
"""

import argparse
import time
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fk_workspace import make_homogeneous, parse_urdf_joints, rpy_to_rot

# ── 1. 批量正向运动学 ────────────────────────────────────────────────────


def rodrigues_batch(axis, angles):
    """Rodrigues 公式批量版：axis 固定，(N,) 关节角 → (N, 3, 3) 旋转矩阵"""
    k = axis / np.linalg.norm(axis)
    skew = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    sin_a = np.sin(angles)[:, None, None]
    cos_a = np.cos(angles)[:, None, None]
    return np.eye(3)[None] + sin_a * skew[None] + (1.0 - cos_a) * (skew @ skew)[None]


def batch_fk(joints, q_batch):
    """批量 FK：q_batch (N, n_joints) → 位置 (N,3)、四元数 (N,4)（xyzw）"""
    n = q_batch.shape[0]
    transforms = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    for joint, angles in zip(joints, q_batch.T):
        fixed = make_homogeneous(rpy_to_rot(*joint["rpy"]), joint["xyz"])
        joint_mats = np.zeros((n, 4, 4))
        joint_mats[:, :3, :3] = rodrigues_batch(joint["axis"], angles)
        joint_mats[:, 3, 3] = 1.0
        # URDF 语义：先固定偏移，再绕关节轴旋转
        transforms = transforms @ (fixed @ joint_mats)

    from scipy.spatial.transform import Rotation

    quats = Rotation.from_matrix(transforms[:, :3, :3]).as_quat()
    return transforms[:, :3, 3], quats


# ── 2. 探针姿态 / 体素覆盖 ───────────────────────────────────────────────


def sample_orientations(n_orient, rng):
    """Shoemake 方法在 SO(3) 上均匀采样，返回 (n, 4) 的 xyzw 四元数"""
    u = rng.random((n_orient, 3))
    q = np.empty((n_orient, 4))
    q[:, 0] = np.sqrt(1.0 - u[:, 0]) * np.sin(2 * np.pi * u[:, 1])
    q[:, 1] = np.sqrt(1.0 - u[:, 0]) * np.cos(2 * np.pi * u[:, 1])
    q[:, 2] = np.sqrt(u[:, 0]) * np.sin(2 * np.pi * u[:, 2])
    q[:, 3] = np.sqrt(u[:, 0]) * np.cos(2 * np.pi * u[:, 2])
    return q


def voxel_index(positions, origin, step, n_cells):
    """笛卡尔坐标 → 体素线性索引（越界为 -1）"""
    ijk = np.floor((positions - origin) / step).astype(np.int64)
    inside = np.all((ijk >= 0) & (ijk < n_cells), axis=1)
    flat = (ijk[:, 0] * n_cells + ijk[:, 1]) * n_cells + ijk[:, 2]
    flat[~inside] = -1
    return flat


def voxel_angle_coverage(flat, quats, probes):
    """按体素统计姿态覆盖半径

    返回 (voxel_ids, counts, theta_max[rad])，theta_max 为最坏探针姿态到
    体素内最近样本的测地角。
    """
    order = np.argsort(flat, kind="stable")
    flat_sorted = flat[order]
    quats_sorted = quats[order]

    starts = np.flatnonzero(np.r_[True, flat_sorted[1:] != flat_sorted[:-1]])
    counts = np.diff(np.r_[starts, flat_sorted.size])
    voxel_ids = flat_sorted[starts]

    # cos(相对转角) = 2·(q_i·q_r)² - 1；对每个探针取体素内最大值（最近样本），
    # 再对探针取最小值（最坏方向）。用余弦比较避免逐样本反三角运算。
    min_cos = np.ones(voxel_ids.size)
    for probe in probes:
        dots = quats_sorted @ probe.astype(np.float32)
        cos_to_probe = np.clip(2.0 * dots * dots - 1.0, -1.0, 1.0)
        per_probe = np.maximum.reduceat(cos_to_probe, starts)
        np.minimum(min_cos, per_probe, out=min_cos)

    return voxel_ids, counts, np.arccos(np.clip(min_cos, -1.0, 1.0))


# ── 3. 蒙特卡洛采样 ──────────────────────────────────────────────────────


def measure_workspace(joints, args, rng):
    """关节空间采样 + 批量 FK + 体素姿态覆盖估计"""
    origin = -args.r_max
    ticks = np.arange(-args.r_max, args.r_max + 1e-9, args.step)
    n_cells = len(ticks)

    limits = np.array([[j["lower"], j["upper"]] for j in joints])
    probes = sample_orientations(args.n_probe, rng)

    flat_parts, quat_parts = [], []
    n_samples = 0
    t0 = time.time()

    while n_samples < args.samples:
        chunk = min(args.chunk, args.samples - n_samples)
        q_batch = rng.uniform(limits[:, 0], limits[:, 1], size=(chunk, len(joints)))
        positions, quats = batch_fk(joints, q_batch)

        flat = voxel_index(positions, origin, args.step, n_cells)
        # 只保留网格内、搜索球内、且不低于 z_min 的样本
        radii = np.linalg.norm(positions, axis=1)
        keep = (flat >= 0) & (radii <= args.r_max + 1e-9) & (positions[:, 2] >= args.z_min)
        flat_parts.append(flat[keep])
        quat_parts.append(quats[keep].astype(np.float32))
        n_samples += chunk

        kept = int(keep.sum())
        elapsed = time.time() - t0
        rate = n_samples / elapsed if elapsed > 0 else 0.0
        eta = (args.samples - n_samples) / rate if rate > 0 else 0.0
        print(
            f"  [{n_samples}/{args.samples}] {rate / 1000:.0f} k样本/s, "
            f"本轮保留 {kept}/{chunk}, ETA {eta:.0f}s",
            flush=True,
        )

    flat = np.concatenate(flat_parts)
    quats = np.concatenate(quat_parts)
    if flat.size == 0:
        raise RuntimeError("没有样本落在搜索区域内，请检查 --r-max / --z-min / --step")

    print(f"🔍 姿态覆盖统计: {flat.size} 个样本, {args.n_probe} 个探针姿态...", flush=True)
    voxel_ids, counts, theta_max = voxel_angle_coverage(flat, quats, probes)

    grid = np.stack(
        [
            voxel_ids // (n_cells * n_cells),
            (voxel_ids // n_cells) % n_cells,
            voxel_ids % n_cells,
        ],
        axis=1,
    )
    centers = origin + (grid + 0.5) * args.step

    return {
        "centers": centers,
        "counts": counts,
        "theta_max": theta_max,
        "samples": int(flat.size),
        "elapsed": time.time() - t0,
    }


# ── 4. 统计与绘图 ────────────────────────────────────────────────────────


def summarize(result, tau, min_samples, step):
    counts = result["counts"]
    theta_max = result["theta_max"]
    centers = result["centers"]

    valid = counts >= min_samples
    centers, theta_max = centers[valid], theta_max[valid]
    dexterity = 1.0 - theta_max / np.pi
    dexterous = theta_max <= tau
    dexterous_pts = centers[dexterous]

    stats = {
        "reachable_cells": int(valid.sum()),
        "dexterous_cells": int(dexterous.sum()),
        "dexterous_volume": float(dexterous.sum() * step**3),
        "reachable_volume": float(valid.sum() * step**3),
        "min_samples": min_samples,
        "tau": tau,
        "centers": centers,
        "dexterity": dexterity,
        "dexterous_positions": dexterous_pts,
    }
    for axis_index, axis in enumerate(("x", "y", "z")):
        stats[f"{axis}_span"] = (
            float(np.ptp(dexterous_pts[:, axis_index])) if len(dexterous_pts) else 0.0
        )
    stats["max_radius"] = (
        float(np.linalg.norm(dexterous_pts, axis=1).max()) if len(dexterous_pts) else 0.0
    )
    stats["mean_dexterity"] = float(dexterity.mean()) if len(dexterity) else 0.0
    stats["median_count"] = float(np.median(counts[valid])) if np.any(valid) else 0.0
    return stats


def plot_results(stats, arm_type, step, output_path):
    centers = stats["centers"]
    dexterity = stats["dexterity"]
    dexterous_pts = stats["dexterous_positions"]

    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(2, 2)

    # ── 图1: 3D 灵巧度分布 ──
    ax1 = fig.add_subplot(gs[0], projection="3d")
    sc = ax1.scatter(
        centers[:, 0],
        centers[:, 1],
        centers[:, 2],
        c=dexterity,
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
        s=6,
        alpha=0.35,
        marker=".",
    )
    if len(dexterous_pts):
        ax1.scatter(
            dexterous_pts[:, 0],
            dexterous_pts[:, 1],
            dexterous_pts[:, 2],
            c="tab:red",
            s=14,
            alpha=0.9,
            marker="s",
            label=f"dexterous (τ = {np.degrees(stats['tau']):.0f}°)",
        )
        ax1.legend(loc="upper right", fontsize=8)
    fig.colorbar(sc, ax=ax1, shrink=0.5, label="dexterity")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title(
        f"{arm_type.upper()} Dexterous Workspace (Monte Carlo, 3D)\n"
        f"{stats['reachable_cells']} cells, step {step:.3f} m"
    )
    ax1.set_box_aspect([1, 1, 1])

    # ── 图2: XZ 截面（|Y| < step/2） ──
    ax2 = fig.add_subplot(gs[1])
    mask = np.abs(centers[:, 1]) < step / 2
    if np.any(mask):
        sc2 = ax2.scatter(
            centers[mask, 0],
            centers[mask, 2],
            c=dexterity[mask],
            cmap="viridis",
            vmin=0.0,
            vmax=1.0,
            s=20,
            marker="s",
        )
        fig.colorbar(sc2, ax=ax2, shrink=0.8, label="dexterity")
    if len(dexterous_pts):
        mask_d = np.abs(dexterous_pts[:, 1]) < step / 2
        if np.any(mask_d):
            ax2.scatter(
                dexterous_pts[mask_d, 0], dexterous_pts[mask_d, 2], c="tab:red", s=16, marker="s"
            )
    ax2.plot(0, 0, "ks", markersize=8, label="Base")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")
    ax2.set_title(f"XZ Slice (|Y| < {step / 2:.3f} m)")
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    # ── 图3: 多个高度的 XY 截面 ──
    ax3 = fig.add_subplot(gs[2])
    z_levels = np.linspace(centers[:, 2].min(), centers[:, 2].max(), 4)
    colors = plt.cm.plasma(np.linspace(0.2, 0.9, len(z_levels)))
    for z, color in zip(z_levels, colors):
        mask_z = np.abs(centers[:, 2] - z) < step / 2
        if not np.any(mask_z):
            continue
        ax3.scatter(
            centers[mask_z, 0],
            centers[mask_z, 1],
            c=dexterity[mask_z],
            cmap="viridis",
            vmin=0.0,
            vmax=1.0,
            s=18,
            marker="s",
            edgecolors=color,
            linewidths=0.3,
            label=f"Z={z:+.2f} m",
        )
    ax3.plot(0, 0, "ks", markersize=8, label="Base")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("XY Slices at Z heights")
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8, loc="upper right")

    # ── 图4: YZ 截面（|X| < step/2） ──
    ax4 = fig.add_subplot(gs[3])
    mask_x = np.abs(centers[:, 0]) < step / 2
    if np.any(mask_x):
        sc4 = ax4.scatter(
            centers[mask_x, 1],
            centers[mask_x, 2],
            c=dexterity[mask_x],
            cmap="viridis",
            vmin=0.0,
            vmax=1.0,
            s=20,
            marker="s",
        )
        fig.colorbar(sc4, ax=ax4, shrink=0.8, label="dexterity")
    if len(dexterous_pts):
        mask_d = np.abs(dexterous_pts[:, 0]) < step / 2
        if np.any(mask_d):
            ax4.scatter(
                dexterous_pts[mask_d, 1], dexterous_pts[mask_d, 2], c="tab:red", s=16, marker="s"
            )
    ax4.plot(0, 0, "ks", markersize=8, label="Base")
    ax4.set_xlabel("Y (m)")
    ax4.set_ylabel("Z (m)")
    ax4.set_title(f"YZ Slice (|X| < {step / 2:.3f} m)")
    ax4.set_aspect("equal")
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"\n✅ 图已保存: {output_path}")


# ── 5. 主程序 ────────────────────────────────────────────────────────────


def parse_args(arm_types):
    parser = argparse.ArgumentParser(description="机械臂灵巧工作空间测量（FK 蒙特卡洛，无需 ROS）")
    parser.add_argument("--arm-type", required=True, choices=arm_types, help="机械臂型号（小写）")
    parser.add_argument("--samples", type=int, default=5_000_000, help="关节空间采样数")
    parser.add_argument("--step", type=float, default=0.1, help="体素边长 (m)")
    parser.add_argument("--r-max", type=float, default=1.0, help="搜索球半径 (m)")
    parser.add_argument("--z-min", type=float, default=-1.0, help="最低 Z (m)")
    parser.add_argument("--tau-deg", type=float, default=20.0, help="灵巧判定角容差 (°)")
    parser.add_argument("--n-probe", type=int, default=100, help="姿态覆盖探针数")
    parser.add_argument(
        "--min-samples",
        type=int,
        default=0,
        help="体素最少样本数；0 = 由 τ 自动推导 (100/τ³)",
    )
    parser.add_argument("--chunk", type=int, default=200_000, help="批量 FK 分块大小")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    parser.add_argument("--output-dir", default=None, help="输出目录，默认同目录 results/")
    parser.add_argument(
        "--save-data",
        action="store_true",
        help="同时保存体素数据 .npz（供 scripts/sphere_ik_bench.py 自动选球使用）",
    )
    return parser.parse_args()


def main():
    script_dir = Path(__file__).resolve().parent
    urdf_dir = script_dir.parent.parent / "src" / "tl_description" / "urdf"
    arm_types = sorted(p.stem for p in urdf_dir.glob("*.urdf"))
    args = parse_args(arm_types)

    urdf_path = urdf_dir / f"{args.arm_type}.urdf"
    results_dir = Path(args.output_dir) if args.output_dir else script_dir / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    if args.step <= 0 or args.r_max <= 0 or args.samples <= 0:
        print("❌ --step / --r-max / --samples 必须为正数")
        return 1
    if not 0.0 < args.tau_deg < 180.0:
        print("❌ --tau-deg 必须在 (0, 180) 内")
        return 1

    print(f"📄 加载 URDF: {urdf_path}")
    joints = parse_urdf_joints(str(urdf_path))
    if not joints:
        print("❌ URDF 中没有 revolute 关节")
        return 1
    print(f"🔧 {len(joints)} 个旋转关节: {', '.join(j['name'] for j in joints)}")
    print(
        f"🎲 采样: {args.samples} 构型, 体素 {args.step} m, 搜索球 {args.r_max} m, "
        f"τ = {args.tau_deg}°"
    )
    tau = np.radians(args.tau_deg)
    # 样本数 K 的覆盖半径下限约为 (100/K)^(1/3) rad；K 不足时体素必然判不出灵巧
    # （只会漏判、不会误判），因此默认按 τ 自动抬高体素最少样本数。
    resolution_samples = int(np.ceil(100.0 / tau**3))
    min_samples = args.min_samples if args.min_samples > 0 else resolution_samples
    print(
        f"🧮 体素最少样本数: {min_samples}"
        + ("" if args.min_samples > 0 else f"（由 τ 推导，覆盖分辨率 {np.degrees(tau):.0f}°）")
    )

    rng = np.random.default_rng(args.seed)
    result = measure_workspace(joints, args, rng)

    stats = summarize(result, tau, min_samples, args.step)

    dexterous_volume = stats["dexterous_volume"]
    reachable_volume = stats["reachable_volume"]
    ratio_volume = dexterous_volume / reachable_volume if reachable_volume > 0 else 0.0
    x_span, y_span, z_span = stats["x_span"], stats["y_span"], stats["z_span"]

    print("\n" + "=" * 56)
    print(f"{args.arm_type.upper()} 灵巧工作空间统计 (FK 蒙特卡洛)")
    print("=" * 56)
    print(f"  采样构型: {result['samples']}   (耗时 {result['elapsed']:.0f}s)")
    print(f"  可达体素: {stats['reachable_cells']}   灵巧体素: {stats['dexterous_cells']}")
    print(f"  灵巧空间体积: {dexterous_volume:.4f} m³   (可达空间 {reachable_volume:.4f} m³)")
    print(f"  灵巧/可达体积比: {ratio_volume * 100:.1f}%")
    print(f"  平均灵巧度: {stats['mean_dexterity']:.3f}")
    print(f"  体素样本数中位数: {stats['median_count']:.0f}   (最少 {stats['min_samples']})")
    print(
        f"  灵巧空间跨度: X {x_span * 1000:.0f} mm  Y {y_span * 1000:.0f} mm  "
        f"Z {z_span * 1000:.0f} mm"
    )
    print(f"  灵巧空间最大半径: {stats['max_radius'] * 1000:.0f} mm")

    stats_str = (
        f"Samples: {result['samples']}  Time: {result['elapsed']:.0f}s\n"
        f"Voxel step: {args.step:.3f} m  tau: {args.tau_deg:.0f} deg  "
        f"probes: {args.n_probe}\n"
        f"Reachable: {stats['reachable_cells']}  Dexterous: {stats['dexterous_cells']}\n"
        f"Dexterous volume: {dexterous_volume:.4f} m3\n"
        f"Reachable volume: {reachable_volume:.4f} m3\n"
        f"Ratio: {ratio_volume * 100:.1f}%\n"
        f"Mean dexterity: {stats['mean_dexterity']:.3f}\n"
        f"Median samples/cell: {stats['median_count']:.0f}  min_samples: {stats['min_samples']}\n"
        f"Spans: X {x_span * 1000:.0f} Y {y_span * 1000:.0f} Z {z_span * 1000:.0f} mm\n"
        f"Max radius: {stats['max_radius'] * 1000:.0f} mm"
    )
    print(f"\n{stats_str}")

    img_path = results_dir / f"{args.arm_type}_dexterous_workspace.png"
    plot_results(stats, args.arm_type, args.step, img_path)

    stats_path = results_dir / f"{args.arm_type}_dexterous_workspace_stats.txt"
    with open(str(stats_path), "w") as f:
        f.write(f"{args.arm_type.upper()} Dexterous Workspace (Monte Carlo FK)\n")
        f.write("=" * 56 + "\n")
        f.write(f"URDF: {urdf_path}\n")
        f.write(f"Joints: {', '.join(j['name'] for j in joints)}\n")
        f.write(
            f"min_samples: {stats['min_samples']}  r_max: {args.r_max} m  z_min: {args.z_min} m\n"
        )
        f.write(stats_str + "\n")
    print(f"📊 统计已保存: {stats_path}")

    if args.save_data:
        valid = result["counts"] >= min_samples
        data_path = results_dir / f"{args.arm_type}_dexterous_workspace.npz"
        np.savez(
            str(data_path),
            centers=stats["centers"],
            dexterity=stats["dexterity"],
            counts=result["counts"][valid],
            theta_max=result["theta_max"][valid],
            step=np.float64(args.step),
            tau=np.float64(tau),
            min_samples=np.int64(min_samples),
            samples=np.int64(result["samples"]),
            arm_type=np.str_(args.arm_type),
        )
        print(f"💾 体素数据已保存: {data_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
