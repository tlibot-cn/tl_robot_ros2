#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class SevenDoFJointPublisher(Node):
    def __init__(self):
        super().__init__("seven_dof_joint_publisher")
        self.publisher = self.create_publisher(Float64MultiArray, "/tl_driver/set_servoj_pos", 10)
        # 定时器，100Hz = 0.01秒间隔
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 7个关节角度（度数），全部初始化为0
        # 索引: 0,1,2,3,4,5,6 对应7个关节（关节1-7）
        # 第6关节对应索引5
        self.joint_angles = [0.0] * 7  # [J0, J1, J2, J3, J4, J5, J6]

        # 控制第6关节（索引5）
        self.target_joint = 5
        self.current_angle = 0.0  # 当前角度（度数）
        self.increment = 1.0  # 每次增加1度
        self.max_angle = 20.0  # 最大角度20度

        self.get_logger().info("=" * 50)
        self.get_logger().info("7-DOF Joint Publisher Started")
        self.get_logger().info(f"  - Total joints: 7 (index 0-6)")
        self.get_logger().info(f"  - Controlled joint: Joint 6 (index {self.target_joint})")
        self.get_logger().info(f"  - Increment: {self.increment} degree per step")
        self.get_logger().info(f"  - Target angle: {self.max_angle} degrees")
        self.get_logger().info(f"  - Frequency: 100 Hz")
        self.get_logger().info(f"  - Unit: Degrees")
        self.get_logger().info("=" * 50)

    def timer_callback(self):
        # 检查是否达到最大角度
        if self.current_angle >= self.max_angle:
            # 达到20度，停止增加，保持当前角度
            return

        # 增加第6关节的角度
        self.current_angle += self.increment

        # 确保不超过最大角度
        if self.current_angle > self.max_angle:
            self.current_angle = self.max_angle

        # 更新第6关节的角度（其他关节保持0）
        self.joint_angles[self.target_joint] = self.current_angle

        # 创建并发布消息
        msg = Float64MultiArray()
        msg.data = self.joint_angles.copy()
        self.publisher.publish(msg)

        # 打印信息
        self.get_logger().info(
            f"Joint 6 (index {self.target_joint}) = {self.current_angle:.1f}° | "
            f'All joints: [{", ".join([f"{a:.1f}" for a in self.joint_angles])}]'
        )

        # 到达目标角度时打印完成信息
        if self.current_angle >= self.max_angle:
            self.get_logger().info(f"Joint 6 reached target angle: {self.max_angle}°. Stopping.")


def main(args=None):
    rclpy.init(args=args)
    node = SevenDoFJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nShutting down joint publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
