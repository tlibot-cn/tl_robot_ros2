#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
servoj_test.py — 简单的 ServoJ 关节插补测试节点

流程（示教模式下即可，无需切换运行模式）：
    1. open_servoj          开启跟踪模式
    2. 从零点位置出发，J1（joint 0）插值到 50°，共 200 个插值点，100Hz 频率下发
    3. 到达目标后保持一小段时间，然后 close_servoj 关闭跟踪模式

用法（需先 source install/setup.bash，且 tl_driver 已连接机械臂并运行）：
    python3 src/tl_example/scripts/servoj_test.py
    带参数（--ros-args -p 键:=值）：
    python3 src/tl_example/scripts/servoj_test.py --ros-args -p target_angle:=50.0 -p num_points:=200

常用参数：
    joint        要运动的关节索引（0 起始，J1 为 0），默认 0
    target_angle 目标角度（度），默认 50.0
    num_points   插值点数，默认 200
    rate         下发频率（Hz），默认 100.0
    hold_time    到达目标后保持时间（秒），默认 1.0
"""

import signal
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tl_ros2_interface.srv import OpenServoJ


class ServoJTestNode(Node):
    def __init__(self):
        super().__init__("servoj_test_node")

        # ========== 可调参数 ==========
        self.declare_parameter("arm_joints", 6)       # 机械臂关节数
        self.declare_parameter("joint", 0)            # 要运动的关节索引（J1 = 0）
        self.declare_parameter("target_angle", 50.0)  # 目标角度（度）
        self.declare_parameter("num_points", 200)     # 插值点数
        self.declare_parameter("rate", 100.0)         # 下发频率（Hz）
        self.declare_parameter("hold_time", 1.0)      # 到达目标后保持时间（秒）

        # ServoJ 运动学参数（沿用 tl_teleop 默认值）
        self.declare_parameter("vmax", 80.0)
        self.declare_parameter("amax", 3000.0)
        self.declare_parameter("jmax", 50000.0)

        self.arm_joints = self.get_parameter("arm_joints").value
        self.joint = self.get_parameter("joint").value
        self.target_angle = self.get_parameter("target_angle").value
        self.num_points = self.get_parameter("num_points").value
        self.rate = self.get_parameter("rate").value
        self.hold_time = self.get_parameter("hold_time").value
        self.vmax = self.get_parameter("vmax").value
        self.amax = self.get_parameter("amax").value
        self.jmax = self.get_parameter("jmax").value

        if self.joint >= self.arm_joints:
            self.get_logger().error(f"joint 索引 {self.joint} 超出关节数 {self.arm_joints}")
            raise SystemExit(1)

        # ========== 客户端 / 发布器 ==========
        self._open_cli = self.create_client(OpenServoJ, "/tl_driver/open_servoj")
        self._close_cli = self.create_client(Trigger, "/tl_driver/close_servoj")
        self._servoj_pub = self.create_publisher(Float64MultiArray, "/tl_driver/set_servoj_pos", 10)

        self.get_logger().info(
            f"ServoJ 插补测试: J{self.joint + 1} 从 0° -> {self.target_angle}°, "
            f"插值 {self.num_points} 点 @ {self.rate}Hz"
        )

    # ---------- 工具函数 ----------
    def _wait_for_service(self, cli, name, timeout_s=5.0):
        if not cli.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error(f"服务 {name} 不可用，请确认 tl_driver 已启动并连接机械臂")
            return False
        return True

    def _call_service(self, executor, cli, request, name, timeout_s=5.0):
        future = cli.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=timeout_s)
        if not future.done():
            self.get_logger().error(f"{name} 调用超时")
            return None
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error(f"{name} 失败: {getattr(resp, 'message', '无响应')}")
            return None
        self.get_logger().info(f"{name} 成功")
        return resp

    # ---------- 主流程 ----------
    def run(self, executor):
        # 1. 等待服务
        if not self._wait_for_service(self._open_cli, "/tl_driver/open_servoj"):
            return False
        if not self._wait_for_service(self._close_cli, "/tl_driver/close_servoj"):
            return False

        # 2. 开启 ServoJ 跟踪模式（示教模式下即可，无需 set_current_mode）
        open_req = OpenServoJ.Request()
        open_req.vmax = [self.vmax] * self.arm_joints
        open_req.amax = [self.amax] * self.arm_joints
        open_req.jmax = [self.jmax] * self.arm_joints
        if not self._call_service(executor, self._open_cli, open_req, "open_servoj"):
            return False

        # 3. 从零点位置开始，对指定关节做线性插值（0° -> target_angle°）
        self.get_logger().warn(
            "请确保机械臂当前处于零点位置（关节角全为 0°），即将开始运动，Ctrl-C 可随时中断 ..."
        )
        start_deg = [0.0] * self.arm_joints
        rate = self.create_rate(self.rate)
        try:
            # 3.1 插值 num_points 个点
            for i in range(self.num_points):
                if not rclpy.ok():
                    break
                fraction = i / (self.num_points - 1) if self.num_points > 1 else 1.0
                pos = list(start_deg)
                pos[self.joint] = self.target_angle * fraction
                self._publish(pos)
                rate.sleep()

            # 3.2 到达目标后保持 hold_time，避免停发导致机械臂急停
            pos = list(start_deg)
            pos[self.joint] = self.target_angle
            hold_end = self.get_clock().now() + rclpy.duration.Duration(seconds=self.hold_time)
            while rclpy.ok() and self.get_clock().now() < hold_end:
                self._publish(pos)
                rate.sleep()

            self.get_logger().info("插补完成")
        except KeyboardInterrupt:
            self.get_logger().info("收到中断信号")
        finally:
            self._close_servoj(executor)

        return True

    def _publish(self, pos):
        msg = Float64MultiArray()
        msg.data = pos
        self._servoj_pub.publish(msg)

    def _close_servoj(self, executor):
        self.get_logger().info("关闭 ServoJ 跟踪模式 ...")
        self._call_service(executor, self._close_cli, Trigger.Request(), "close_servoj")


def main():
    rclpy.init()
    node = ServoJTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Ctrl-C 优雅退出
    signal.signal(signal.SIGINT, lambda *_: node.get_logger().info("Ctrl-C 按下，正在关闭 ..."))

    try:
        ok = node.run(executor)
        sys.exit(0 if ok else 1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
