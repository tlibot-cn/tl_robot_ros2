#!/usr/bin/env python3
import math
import signal
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState

import tkinter as tk
from tkinter import ttk, messagebox

def normalize(v):
    v = np.array(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n

def trans_matrix(xyz):
    T = np.eye(4, dtype=np.float64)
    T[0:3, 3] = np.array(xyz, dtype=np.float64)
    return T

def rot_x(a):
    c = math.cos(a)
    s = math.sin(a)

    T = np.eye(4, dtype=np.float64)
    T[1, 1] = c
    T[1, 2] = -s
    T[2, 1] = s
    T[2, 2] = c
    return T

def rot_y(a):
    c = math.cos(a)
    s = math.sin(a)

    T = np.eye(4, dtype=np.float64)
    T[0, 0] = c
    T[0, 2] = s
    T[2, 0] = -s
    T[2, 2] = c
    return T

def rot_z(a):
    c = math.cos(a)
    s = math.sin(a)

    T = np.eye(4, dtype=np.float64)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T

def rpy_matrix(rpy):
    """
    URDF origin rpy 定义：
        fixed-axis roll-pitch-yaw

    对应旋转矩阵：
        R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    roll, pitch, yaw = rpy
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def axis_angle_matrix(axis, angle):
    axis = normalize(axis)
    x, y, z = axis

    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c

    R3 = np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = R3
    return T

def prismatic_matrix(axis, distance):
    axis = normalize(axis)

    T = np.eye(4, dtype=np.float64)
    T[0:3, 3] = axis * distance
    return T

def matrix_to_rpy(T):
    """
    将旋转矩阵转成 roll pitch yaw，单位 rad。

    约定：
        R = Rz(yaw) * Ry(pitch) * Rx(roll)

    也就是 ROS / URDF 常用 fixed-axis RPY 表达。
    """
    R = T[0:3, 0:3]

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return np.array([roll, pitch, yaw], dtype=np.float64)

def parse_float_list(text, default):
    if text is None:
        return default

    text = text.strip()

    if not text:
        return default

    return [float(x) for x in text.split()]

class URDFJoint:
    def __init__(self):
        self.name = ''
        self.joint_type = ''
        self.parent = ''
        self.child = ''
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]
        self.axis = [1.0, 0.0, 0.0]
        self.lower = None
        self.upper = None

class URDFFKModel:
    """
    轻量级 URDF 正解模型。

    支持：
        fixed
        revolute
        continuous
        prismatic

    正解计算：
        T_parent_child = T_origin * T_motion

    其中：
        T_origin 来自 joint 的 origin xyz/rpy
        T_motion 来自 joint 的 axis 和当前关节值
    """

    def __init__(self, urdf_xml, base_link, tip_link):
        self.urdf_xml = urdf_xml
        self.base_link = base_link
        self.tip_link = tip_link

        self.joints = {}
        self.children_map = {}

        self.chain_joints = []
        self.active_joint_names = []

        self.parse_urdf()
        self.build_chain()

    def parse_urdf(self):
        root = ET.fromstring(self.urdf_xml)

        for j in root.findall('joint'):
            joint = URDFJoint()

            joint.name = j.attrib.get('name', '')
            joint.joint_type = j.attrib.get('type', 'fixed')

            parent_elem = j.find('parent')
            child_elem = j.find('child')

            if parent_elem is None or child_elem is None:
                continue

            joint.parent = parent_elem.attrib.get('link', '')
            joint.child = child_elem.attrib.get('link', '')

            origin_elem = j.find('origin')
            if origin_elem is not None:
                joint.origin_xyz = parse_float_list(
                    origin_elem.attrib.get('xyz'),
                    [0.0, 0.0, 0.0]
                )
                joint.origin_rpy = parse_float_list(
                    origin_elem.attrib.get('rpy'),
                    [0.0, 0.0, 0.0]
                )

            axis_elem = j.find('axis')
            if axis_elem is not None:
                joint.axis = parse_float_list(
                    axis_elem.attrib.get('xyz'),
                    [1.0, 0.0, 0.0]
                )

            limit_elem = j.find('limit')
            if limit_elem is not None:
                if 'lower' in limit_elem.attrib:
                    joint.lower = float(limit_elem.attrib['lower'])
                if 'upper' in limit_elem.attrib:
                    joint.upper = float(limit_elem.attrib['upper'])

            self.joints[joint.name] = joint

            if joint.parent not in self.children_map:
                self.children_map[joint.parent] = []

            self.children_map[joint.parent].append(joint)

    def build_chain(self):
        """
        从 base_link 搜索到 tip_link 的关节链。
        """
        queue = deque()
        queue.append((self.base_link, []))

        visited = set()
        found_chain = None

        while queue:
            link, chain = queue.popleft()

            if link == self.tip_link:
                found_chain = chain
                break

            if link in visited:
                continue

            visited.add(link)

            for joint in self.children_map.get(link, []):
                queue.append((joint.child, chain + [joint]))

        if found_chain is None:
            raise RuntimeError(
                f'无法在 URDF 中找到从 {self.base_link} 到 {self.tip_link} 的运动链'
            )

        self.chain_joints = found_chain

        self.active_joint_names = [
            j.name for j in self.chain_joints
            if j.joint_type in ['revolute', 'continuous', 'prismatic']
        ]

    def joint_origin_transform(self, joint):
        return trans_matrix(joint.origin_xyz) @ rpy_matrix(joint.origin_rpy)

    def joint_motion_transform(self, joint, q):
        if joint.joint_type in ['revolute', 'continuous']:
            return axis_angle_matrix(joint.axis, q)

        elif joint.joint_type == 'prismatic':
            return prismatic_matrix(joint.axis, q)

        else:
            return np.eye(4, dtype=np.float64)

    def compute_fk(self, joint_positions):
        """
        joint_positions:
            dict: joint_name -> value

        约定：
            revolute / continuous: rad
            prismatic: m
        """
        T = np.eye(4, dtype=np.float64)

        for joint in self.chain_joints:
            q = joint_positions.get(joint.name, 0.0)

            T_origin = self.joint_origin_transform(joint)
            T_motion = self.joint_motion_transform(joint, q)

            T = T @ T_origin @ T_motion

        return T

class FKTestUINode(Node):
    def __init__(self):
        super().__init__('fk_test_ui_node')

        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool0')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('publish_rate', 30.0)

        self.base_link = self.get_parameter('base_link').value
        self.tip_link = self.get_parameter('tip_link').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.urdf_xml = None
        self.fk_model = None

        self.joint_positions = {}
        self.last_fk_T = None

        self.app = None

        self.joint_pub = self.create_publisher(
            JointState,
            self.joint_states_topic,
            10
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.robot_description_sub = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos
        )

        period = 1.0 / max(self.publish_rate, 1.0)
        self.publish_timer = self.create_timer(
            period,
            self.publish_joint_states
        )

        self.get_logger().info('FK 测试节点已启动')
        self.get_logger().info(f'base_link: {self.base_link}')
        self.get_logger().info(f'tip_link : {self.tip_link}')
        self.get_logger().info(f'joint_states_topic: {self.joint_states_topic}')
        self.get_logger().info('等待 /robot_description ...')

    def robot_description_callback(self, msg):
        if self.urdf_xml is not None:
            return

        self.urdf_xml = msg.data

        try:
            self.fk_model = URDFFKModel(
                self.urdf_xml,
                self.base_link,
                self.tip_link
            )

            for name in self.fk_model.active_joint_names:
                self.joint_positions[name] = 0.0

            self.get_logger().info('成功解析 URDF')
            self.get_logger().info('运动链关节如下:')

            for j in self.fk_model.chain_joints:
                self.get_logger().info(
                    f'  {j.parent} --[{j.name}, {j.joint_type}]--> {j.child}'
                )

            self.get_logger().info(
                f'主动关节: {self.fk_model.active_joint_names}'
            )

            if self.app is not None and not getattr(self.app, 'is_closing', False):
                self.app.after(0, self.app.build_joint_ui)

        except Exception as e:
            self.get_logger().error(f'解析 URDF 或建立运动链失败: {e}')

            if self.app is not None and not getattr(self.app, 'is_closing', False):
                self.app.after(
                    0,
                    lambda: messagebox.showerror('URDF 解析失败', str(e))
                )

    def publish_joint_states(self):
        if self.fk_model is None:
            return

        if not rclpy.ok():
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.fk_model.active_joint_names)
        msg.position = [
            float(self.joint_positions.get(name, 0.0))
            for name in msg.name
        ]

        self.joint_pub.publish(msg)

    def set_joint_position(self, joint_name, value_rad_or_meter):
        self.joint_positions[joint_name] = float(value_rad_or_meter)

    def compute_fk(self):
        if self.fk_model is None:
            return None

        T = self.fk_model.compute_fk(self.joint_positions)
        self.last_fk_T = T
        return T

class FKTestUI(tk.Tk):
    def __init__(self, node: FKTestUINode):
        super().__init__()

        self.node = node
        self.node.app = self

        self.is_closing = False
        self.node_destroyed = False

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind("<Control-c>", lambda event: self.on_close())

        self.title('TL URDF Forward Kinematics Test')
        self.geometry('900x620')

        self.joint_widgets = {}
        self.fk_text_var = tk.StringVar()

        self.create_layout()

        self.after(20, self.ros_spin_once)
        self.after(100, self.update_fk_display)

    def create_layout(self):
        title = ttk.Label(
            self,
            text='URDF 正解测试工具',
            font=('Arial', 18, 'bold')
        )
        title.pack(pady=10)

        info = ttk.Label(
            self,
            text=(
                f'base_link: {self.node.base_link}    '
                f'tip_link: {self.node.tip_link}    '
                f'发布: {self.node.joint_states_topic}'
            ),
            font=('Arial', 11)
        )
        info.pack(pady=5)

        warn = ttk.Label(
            self,
            text='注意：测试时请避免其它节点同时发布 /joint_states，否则 RViz 和 robot_state_publisher 会混乱。',
            foreground='red'
        )
        warn.pack(pady=5)

        self.main_frame = ttk.Frame(self)
        self.main_frame.pack(fill='both', expand=True, padx=10, pady=10)

        self.joint_frame = ttk.LabelFrame(
            self.main_frame,
            text='关节输入'
        )
        self.joint_frame.pack(fill='x', padx=5, pady=5)

        self.output_frame = ttk.LabelFrame(
            self.main_frame,
            text='URDF 正解结果'
        )
        self.output_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.fk_label = ttk.Label(
            self.output_frame,
            textvariable=self.fk_text_var,
            font=('Courier', 11),
            justify='left'
        )
        self.fk_label.pack(anchor='w', padx=10, pady=10)

        button_frame = ttk.Frame(self.main_frame)
        button_frame.pack(fill='x', pady=5)

        ttk.Button(
            button_frame,
            text='全部置零',
            command=self.reset_all_joints
        ).pack(side='left', padx=5)

        ttk.Button(
            button_frame,
            text='打印当前关节和正解',
            command=self.print_current_state
        ).pack(side='left', padx=5)

        self.waiting_label = ttk.Label(
            self.joint_frame,
            text='等待 /robot_description ...',
            foreground='blue'
        )
        self.waiting_label.pack(padx=10, pady=10)

    def build_joint_ui(self):
        if self.is_closing:
            return

        for child in self.joint_frame.winfo_children():
            child.destroy()

        if self.node.fk_model is None:
            ttk.Label(
                self.joint_frame,
                text='URDF 尚未加载',
                foreground='red'
            ).pack(padx=10, pady=10)
            return

        self.joint_widgets.clear()

        row = 0

        header = ttk.Frame(self.joint_frame)
        header.grid(
            row=row,
            column=0,
            columnspan=5,
            sticky='ew',
            padx=5,
            pady=5
        )

        ttk.Label(
            header,
            text='旋转关节输入单位：deg；内部发布 /joint_states 使用 rad'
        ).pack(anchor='w')

        row += 1

        for joint in self.node.fk_model.chain_joints:
            if joint.joint_type not in ['revolute', 'continuous', 'prismatic']:
                continue

            name = joint.name

            label = ttk.Label(
                self.joint_frame,
                text=name,
                width=25
            )
            label.grid(
                row=row,
                column=0,
                padx=5,
                pady=5,
                sticky='w'
            )

            if joint.joint_type == 'prismatic':
                min_val = joint.lower if joint.lower is not None else -0.5
                max_val = joint.upper if joint.upper is not None else 0.5
                unit = 'm'
                slider_scale = 1.0
                default_display = 0.0
            else:
                if joint.joint_type == 'continuous':
                    min_val = -180.0
                    max_val = 180.0
                else:
                    lower = joint.lower if joint.lower is not None else -math.pi
                    upper = joint.upper if joint.upper is not None else math.pi
                    min_val = math.degrees(lower)
                    max_val = math.degrees(upper)

                unit = 'deg'
                slider_scale = math.pi / 180.0
                default_display = 0.0

            var = tk.DoubleVar(value=default_display)

            slider = ttk.Scale(
                self.joint_frame,
                from_=min_val,
                to=max_val,
                variable=var,
                orient='horizontal',
                length=400
            )
            slider.grid(
                row=row,
                column=1,
                padx=5,
                pady=5,
                sticky='ew'
            )

            entry = ttk.Entry(
                self.joint_frame,
                width=12
            )
            entry.insert(0, f'{default_display:.3f}')
            entry.grid(
                row=row,
                column=2,
                padx=5,
                pady=5
            )

            unit_label = ttk.Label(
                self.joint_frame,
                text=unit,
                width=5
            )
            unit_label.grid(
                row=row,
                column=3,
                padx=5,
                pady=5
            )

            def make_slider_callback(joint_name, var_ref, entry_ref, scale):
                def callback(_event=None):
                    if self.is_closing:
                        return

                    display_value = float(var_ref.get())

                    entry_ref.delete(0, tk.END)
                    entry_ref.insert(0, f'{display_value:.3f}')

                    self.node.set_joint_position(
                        joint_name,
                        display_value * scale
                    )

                return callback

            def make_entry_callback(joint_name, var_ref, entry_ref, scale):
                def callback(_event=None):
                    if self.is_closing:
                        return

                    try:
                        display_value = float(entry_ref.get())

                        var_ref.set(display_value)

                        self.node.set_joint_position(
                            joint_name,
                            display_value * scale
                        )

                    except Exception:
                        if not self.is_closing:
                            messagebox.showwarning(
                                '输入错误',
                                f'{joint_name} 输入不是合法数字'
                            )

                return callback

            slider.configure(
                command=lambda _v, cb=make_slider_callback(
                    name,
                    var,
                    entry,
                    slider_scale
                ): cb()
            )

            entry.bind(
                '<Return>',
                make_entry_callback(
                    name,
                    var,
                    entry,
                    slider_scale
                )
            )

            ttk.Button(
                self.joint_frame,
                text='应用',
                command=make_entry_callback(
                    name,
                    var,
                    entry,
                    slider_scale
                )
            ).grid(
                row=row,
                column=4,
                padx=5,
                pady=5
            )

            self.joint_widgets[name] = {
                'var': var,
                'entry': entry,
                'scale': slider_scale,
                'joint_type': joint.joint_type
            }

            row += 1

        self.joint_frame.columnconfigure(1, weight=1)

    def reset_all_joints(self):
        if self.is_closing:
            return

        for name, widgets in self.joint_widgets.items():
            widgets['var'].set(0.0)

            widgets['entry'].delete(0, tk.END)
            widgets['entry'].insert(0, '0.000')

            self.node.set_joint_position(name, 0.0)

    def print_current_state(self):
        if self.is_closing:
            return

        T = self.node.compute_fk()

        if T is None:
            print('FK 尚未可用')
            return

        p = T[0:3, 3]
        rpy = matrix_to_rpy(T)
        rpy_deg = np.degrees(rpy)

        print('\n========== 当前关节 ==========')

        for name in self.node.fk_model.active_joint_names:
            q = self.node.joint_positions.get(name, 0.0)
            print(f'{name}: {q:.9f} rad, {math.degrees(q):.6f} deg')

        print('\n========== URDF FK XYZ + RPY ==========')
        print(f'x     = {p[0]: .9f} m')
        print(f'y     = {p[1]: .9f} m')
        print(f'z     = {p[2]: .9f} m')

        print('\nrpy [rad]:')
        print(f'roll  = {rpy[0]: .9f}')
        print(f'pitch = {rpy[1]: .9f}')
        print(f'yaw   = {rpy[2]: .9f}')

        print('\nrpy [deg]:')
        print(f'roll  = {rpy_deg[0]: .6f}')
        print(f'pitch = {rpy_deg[1]: .6f}')
        print(f'yaw   = {rpy_deg[2]: .6f}')

        print('=======================================\n')

    def update_fk_display(self):
        if self.is_closing:
            return

        try:
            T = self.node.compute_fk()

            if T is not None:
                p = T[0:3, 3]
                rpy = matrix_to_rpy(T)
                rpy_deg = np.degrees(rpy)

                fk_text = ''
                fk_text += 'URDF FK Result\n'
                fk_text += '==============================\n\n'

                fk_text += 'position xyz [m]:\n'
                fk_text += f'  x = {p[0]: .6f}\n'
                fk_text += f'  y = {p[1]: .6f}\n'
                fk_text += f'  z = {p[2]: .6f}\n\n'

                fk_text += 'rpy [rad]:\n'
                fk_text += f'  roll  = {rpy[0]: .6f}\n'
                fk_text += f'  pitch = {rpy[1]: .6f}\n'
                fk_text += f'  yaw   = {rpy[2]: .6f}\n\n'

                fk_text += 'rpy [deg]:\n'
                fk_text += f'  roll  = {rpy_deg[0]: .6f}\n'
                fk_text += f'  pitch = {rpy_deg[1]: .6f}\n'
                fk_text += f'  yaw   = {rpy_deg[2]: .6f}\n'

                self.fk_text_var.set(fk_text)

        except Exception as e:
            if not self.is_closing:
                print(f'update_fk_display 异常: {e}')

        if not self.is_closing:
            self.after(100, self.update_fk_display)

    def ros_spin_once(self):
        if self.is_closing:
            return

        try:
            if rclpy.ok() and self.node is not None:
                rclpy.spin_once(self.node, timeout_sec=0.0)

        except KeyboardInterrupt:
            self.on_close()
            return

        except Exception as e:
            if not self.is_closing:
                print(f'ros_spin_once 异常: {e}')

        if not self.is_closing:
            self.after(10, self.ros_spin_once)

    def on_close(self):
        """
        统一退出入口。

        支持：
            1. 点击 UI 右上角关闭按钮
            2. UI 内 Ctrl+C
            3. 终端 Ctrl+C

        这里必须避免：
            1. 重复 destroy_node()
            2. 重复 rclpy.shutdown()
            3. Tkinter 窗口销毁后 after 回调继续执行
        """
        if self.is_closing:
            return

        self.is_closing = True

        try:
            if self.node is not None and not self.node_destroyed:
                self.node.destroy_node()
                self.node_destroyed = True
        except Exception as e:
            print(f'关闭 ROS node 时出现异常: {e}')

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f'rclpy.shutdown() 异常: {e}')

        try:
            self.destroy()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)

    node = FKTestUINode()
    app = FKTestUI(node)

    def handle_sigint(sig, frame):
        try:
            if not app.is_closing:
                app.on_close()
        except Exception:
            pass

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        app.mainloop()

    except KeyboardInterrupt:
        app.on_close()

    finally:
        try:
            if not app.is_closing:
                app.on_close()
        except Exception as e:
            print(f'退出清理异常: {e}')

if __name__ == '__main__':
    main()