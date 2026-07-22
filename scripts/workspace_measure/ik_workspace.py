#!/usr/bin/env python3
"""
TCB705 工作空间测量 — 调用 MoveIt2 IK 服务

在球坐标网格上采样，对每个方向从内向外搜索可达/不可达边界。
使用真实 IK 求解器（KDL），比纯运动学采样更准确。
"""

import sys
import time
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class WorkspaceIKMeasurer(Node):
    """使用 IK 服务测量机械臂工作空间"""

    def __init__(self):
        super().__init__("workspace_ik_measurer")

        # ── 参数 ──
        self.group_name = "tcb_group"
        self.base_frame = "link0"
        self.tip_link = "link7"

        # RPY(0, 0, -90°): 绕基座 Z 轴转 -90°
        self.default_orient = Pose().orientation
        self.default_orient.x = 0.0
        self.default_orient.y = 0.0
        self.default_orient.z = -0.7071068
        self.default_orient.w = 0.7071068

        # IK 超时
        self.ik_timeout_ns = 80_000_000  # 80ms

        # ── 获取初始关节状态作为 IK seed ──
        self.home_state = self._get_home_state()

        # ── 服务客户端 ──
        cb_group = MutuallyExclusiveCallbackGroup()
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb_group)
        while not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("等待 /compute_ik 服务...")
        self.get_logger().info("✓ IK 服务就绪")

    def _get_home_state(self):
        """从参数服务器获取 home 关节值，或使用 SRDF 中的 home 状态"""
        state = RobotState()
        state.joint_state.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        # Home: 全零 (match SRDF)
        state.joint_state.position = [0.0] * 7
        state.joint_state.velocity = []
        state.joint_state.effort = []
        state.is_diff = False
        return state

    def make_ik_request(self, x, y, z, seed_state=None):
        """构建 IK 请求"""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation = self.default_orient

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = self.ik_timeout_ns
        req.ik_request.avoid_collisions = False

        if seed_state is not None:
            req.ik_request.robot_state = seed_state
        else:
            req.ik_request.robot_state = self.home_state

        return req

    def check_reachable(self, x, y, z):
        """检查 (x, y, z) 是否可达（存在 IK 解）"""
        request = self.make_ik_request(x, y, z)
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            return future.result().error_code.val == 1  # SUCCESS
        return False

    def binary_search_boundary(self, theta, phi, r_min, r_max, n_tries=12):
        """
        沿 (theta, phi) 方向二分搜索可达边界
        返回: (reachable_r, unreachable_r) 或 None(全不可达)
        """
        dx = np.cos(theta) * np.sin(phi)
        dy = np.sin(theta) * np.sin(phi)
        dz = np.cos(phi)

        # 先找第一个可达点（从内向外）
        r_low, r_high = None, None
        for i, r in enumerate(np.linspace(r_min, r_max, 12)):
            x, y, z = r * dx, r * dy, r * dz
            if self.check_reachable(x, y, z):
                r_low = r
                break

        if r_low is None:
            # 最近点也不可达
            return None, None

        # 二分搜索边界
        lo, hi = r_low, r_max
        for _ in range(n_tries):
            mid = (lo + hi) / 2
            x, y, z = mid * dx, mid * dy, mid * dz
            if self.check_reachable(x, y, z):
                lo = mid
            else:
                hi = mid

        return lo, hi

    def measure_workspace(self, n_theta=32, n_phi=16, max_radius=1.0):
        """在球坐标网格上测量工作空间"""
        self.get_logger().info(f"开始测量 ({n_theta}x{n_phi} 方向)...")

        theta_vals = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
        phi_vals = np.linspace(0, np.pi, n_phi)
        r_min = 0.05  # 起始搜索半径

        # 记录结果
        reachable_pts = []
        boundary_pts = []
        unreachable_pts = []
        all_pts = []
        all_reachable = []

        total = len(theta_vals) * len(phi_vals)
        count = 0
        t0 = time.time()

        for theta in theta_vals:
            for phi in phi_vals:
                count += 1
                if count % 10 == 0:
                    elapsed = time.time() - t0
                    rate = count / elapsed if elapsed > 0 else 0
                    eta = (total - count) / rate if rate > 0 else 0
                    self.get_logger().info(f"  [{count}/{total}] {rate:.1f} dir/s, ETA {eta:.0f}s")

                d = np.array(
                    [np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)]
                )

                lo, hi = self.binary_search_boundary(theta, phi, r_min, max_radius)

                if lo is None:
                    pts = []
                    for r in np.linspace(r_min, max_radius, 8):
                        p = r * d
                        pts.append(p)
                        unreachable_pts.append(p)
                        all_pts.append(p)
                        all_reachable.append(False)
                    continue

                # 记录边界点
                boundary_pts.append(lo * d)

                # 记录内部可达点
                for r in np.linspace(r_min, lo, 6):
                    p = r * d
                    reachable_pts.append(p)
                    all_pts.append(p)
                    all_reachable.append(True)

                # 记录外部不可达点
                if hi is not None:
                    for r in np.linspace(hi, min(hi + 0.1, max_radius), 3):
                        p = r * d
                        unreachable_pts.append(p)
                        all_pts.append(p)
                        all_reachable.append(False)

        elapsed = time.time() - t0
        self.get_logger().info(
            f"完成: {elapsed:.0f}s, " f"{len(reachable_pts)} 可达, {len(unreachable_pts)} 不可达"
        )

        return {
            "reachable": np.array(reachable_pts),
            "unreachable": np.array(unreachable_pts),
            "boundary": np.array(boundary_pts),
            "all_pts": np.array(all_pts),
            "all_reachable": np.array(all_reachable),
            "theta_vals": theta_vals,
            "phi_vals": phi_vals,
        }


def plot_results(result, stats_str, output_path="tcb705_workspace_ik.png"):
    reachable = result["reachable"]
    unreachable = result["unreachable"]
    boundary = result["boundary"]

    fig = plt.figure(figsize=(22, 7))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.1, 0.9, 0.9])

    # ── 图1: 3D 包络 ──
    ax1 = fig.add_subplot(gs[0], projection="3d")
    if len(reachable) > 0:
        r_norm = np.linalg.norm(reachable, axis=1)
        ax1.scatter(
            reachable[:, 0],
            reachable[:, 1],
            reachable[:, 2],
            c=r_norm,
            cmap="viridis",
            s=1,
            alpha=0.3,
            marker=".",
        )
    if len(unreachable) > 0:
        ax1.scatter(
            unreachable[:, 0],
            unreachable[:, 1],
            unreachable[:, 2],
            c="red",
            s=0.5,
            alpha=0.05,
            marker=".",
        )

    # 包络面
    if len(boundary) >= 4:
        try:
            from scipy.spatial import ConvexHull

            hull = ConvexHull(boundary)
            for simplex in hull.simplices:
                tri = boundary[simplex]
                poly = Poly3DCollection(
                    [tri], alpha=0.1, facecolor="steelblue", edgecolor="navy", linewidth=0.3
                )
                ax1.add_collection3d(poly)
        except Exception:
            pass

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title("TCB705 Workspace (IK-based)\nReachable (color=radius) / Unreachable (red)")
    ax1.set_box_aspect([1, 1, 1])

    # ── 图2: XZ 截面 ──
    ax2 = fig.add_subplot(gs[1])
    for pts, color, alpha, s in [
        (reachable, None, 0.15, 0.5),
        (unreachable, "red", 0.03, 0.3),
    ]:
        mask = np.abs(pts[:, 1]) < 0.08 if len(pts) > 0 else []
        if np.any(mask):
            xz = pts[mask]
            if color is None:
                ax2.scatter(
                    xz[:, 0],
                    xz[:, 2],
                    c=np.linalg.norm(xz, axis=1),
                    cmap="viridis",
                    s=s,
                    alpha=alpha,
                )
            else:
                ax2.scatter(xz[:, 0], xz[:, 2], c=color, s=s, alpha=alpha)

    # 包络线
    if len(boundary) >= 4:
        mask_b = np.abs(boundary[:, 1]) < 0.08
        if np.any(mask_b):
            bxz = boundary[mask_b]
            try:
                from scipy.spatial import ConvexHull

                hull_xz = ConvexHull(bxz[:, [0, 2]])
                for sim in hull_xz.simplices:
                    xz = bxz[sim, [0, 2]]
                    ax2.plot(xz[:, 0], xz[:, 1], "r-", lw=1)
            except Exception:
                pass

    ax2.plot(0, 0, "ks", markersize=8, label="Base")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")
    ax2.set_title("XZ Slice (|Y| < 0.08 m)")
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    # ── 图3: XY 截面 ──
    ax3 = fig.add_subplot(gs[2])
    z_levels = [0.0, 0.2, 0.4, 0.6, 0.8]
    colors = plt.cm.plasma(np.linspace(0.2, 0.9, len(z_levels)))

    for zi, c in zip(z_levels, colors):
        tol = 0.04
        mask = np.abs(reachable[:, 2] - zi) < tol if len(reachable) > 0 else []
        if np.any(mask):
            xy = reachable[mask]
            ax3.scatter(xy[:, 0], xy[:, 1], c=[c], s=0.5, alpha=0.2)
            try:
                from scipy.spatial import ConvexHull

                hull_xy = ConvexHull(xy[:, [0, 1]])
                for sim in hull_xy.simplices:
                    pts_h = xy[sim, [0, 1]]
                    ax3.plot(pts_h[:, 0], pts_h[:, 1], "-", color=c, lw=1.2, alpha=0.8)
            except Exception:
                pass

    ax3.plot(0, 0, "ks", markersize=8, label="Base")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("XY Slices at Z heights (IK)")
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8)

    # 统计信息
    fig.text(
        0.02,
        0.01,
        stats_str,
        fontsize=9,
        family="monospace",
        verticalalignment="bottom",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    plt.tight_layout(rect=[0, 0.06, 1, 1])
    plt.savefig(str(output_path), dpi=200, bbox_inches="tight")
    print(f"\n✅ 图已保存: {output_path}")


def main():
    rclpy.init()
    script_dir = Path(__file__).resolve().parent
    results_dir = script_dir / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    measurer = WorkspaceIKMeasurer()

    try:
        result = measurer.measure_workspace(n_theta=32, n_phi=16, max_radius=0.95)

        # 统计
        n_dirs = len(result["theta_vals"]) * len(result["phi_vals"])
        n_boundary = len(result["boundary"])
        b = result["boundary"]

        print()
        print("=" * 50)
        print("TCB705 工作空间统计 (IK 测量)")
        print("=" * 50)
        print(f"  方向数: {n_dirs}")
        print(f"  边界点数: {n_boundary}")

        if n_boundary > 0:
            x_r = (b[:, 0].min(), b[:, 0].max())
            y_r = (b[:, 1].min(), b[:, 1].max())
            z_r = (b[:, 2].min(), b[:, 2].max())
            radii = np.linalg.norm(b, axis=1)
            max_r = radii.max()
            mean_r = radii.mean()

            print(f"  X 范围: [{x_r[0]:+.4f}, {x_r[1]:+.4f}]  span: {(x_r[1]-x_r[0]):.4f} m")
            print(f"  Y 范围: [{y_r[0]:+.4f}, {y_r[1]:+.4f}]  span: {(y_r[1]-y_r[0]):.4f} m")
            print(f"  Z 范围: [{z_r[0]:+.4f}, {z_r[1]:+.4f}]  span: {(z_r[1]-z_r[0]):.4f} m")
            print(f"  最大半径: {max_r:.4f} m")
            print(f"  平均半径: {mean_r:.4f} m")
            print(f"  X 跨度: {(x_r[1]-x_r[0])*1000:.0f} mm")
            print(f"  Y 跨度: {(y_r[1]-y_r[0])*1000:.0f} mm")
            print(f"  Z 跨度: {(z_r[1]-z_r[0])*1000:.0f} mm")
            print(f"  最大半径: {max_r*1000:.0f} mm")

            stats_str = (
                f"X: [{x_r[0]:.3f}, {x_r[1]:.3f}]  span {(x_r[1]-x_r[0]):.3f}m\n"
                f"Y: [{y_r[0]:.3f}, {y_r[1]:.3f}]  span {(y_r[1]-y_r[0]):.3f}m\n"
                f"Z: [{z_r[0]:.3f}, {z_r[1]:.3f}]  span {(z_r[1]-z_r[0]):.3f}m\n"
                f"Max radius: {max_r:.3f}m  Mean: {mean_r:.3f}m"
            )
            img_path = results_dir / "tcb705_workspace_ik.png"
            plot_results(result, stats_str, output_path=img_path)

            stats_path = results_dir / "tcb705_workspace_ik_stats.txt"
            with open(str(stats_path), "w") as f:
                f.write("TCB705 Workspace (IK-based)\n")
                f.write(f"{'='*50}\n")
                f.write(f"X range: [{x_r[0]:+.4f}, {x_r[1]:+.4f}]  span: {(x_r[1]-x_r[0]):.4f} m\n")
                f.write(f"Y range: [{y_r[0]:+.4f}, {y_r[1]:+.4f}]  span: {(y_r[1]-y_r[0]):.4f} m\n")
                f.write(f"Z range: [{z_r[0]:+.4f}, {z_r[1]:+.4f}]  span: {(z_r[1]-z_r[0]):.4f} m\n")
                f.write(f"Max radius: {max_r:.4f} m\n")
                f.write(f"Orientation: RPY(0, 0, -90°) / q(0, 0, -0.707, 0.707)\n")
                f.write(f"Solver: KDL (via MoveIt2 /compute_ik)\n")
            print(f"\n📊 统计已保存: {stats_path}")

    finally:
        measurer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
