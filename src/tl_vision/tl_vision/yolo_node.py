#!/usr/bin/env python3
"""
YOLO Detection Demo Node for ROS2
订阅RGB图像，运行YOLO推理，使用固定深度值计算3D坐标并发布
支持实时OpenCV显示
"""

import os
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from tl_ros2_interface.msg import ObjectInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np


class YOLODemo(Node):
    
    def __init__(self):
        super().__init__('yolo_node')
        
        _default_model_path = os.path.join(
            get_package_share_directory('tl_vision'), 'model', 'yolov8n.pt'
        )
        
        self.declare_parameter('model_path', _default_model_path)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera.fx', 610.0)   # 焦距 x (像素单位)
        self.declare_parameter('camera.fy', 610.0)   # 焦距 y (像素单位)
        self.declare_parameter('camera.cx', 320.0)   # 光心 x
        self.declare_parameter('camera.cy', 240.0)   # 光心 y
        self.declare_parameter('show_display', True)       # 是否显示OpenCV窗口
        self.declare_parameter('display_window_name', 'YOLO Detection')  # 窗口名称
        # self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')  # 深度话题（备用）
        self.declare_parameter('fixed_depth', 0.419)       # 固定深度（用卷尺或深度图标定后填入）
        
        # 获取参数值
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('tl_vision'), 'model', 'yolov8n.pt'
            )
        elif not os.path.isabs(model_path):
            # 相对路径 → 在 package model 目录下查找
            model_path = os.path.join(
                get_package_share_directory('tl_vision'), 'model', model_path
            )
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value
        
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        
        self.fx = self.get_parameter('camera.fx').get_parameter_value().double_value
        self.fy = self.get_parameter('camera.fy').get_parameter_value().double_value
        self.cx = self.get_parameter('camera.cx').get_parameter_value().double_value
        self.cy = self.get_parameter('camera.cy').get_parameter_value().double_value
        self.show_display = self.get_parameter('show_display').get_parameter_value().bool_value
        self.window_name = self.get_parameter('display_window_name').get_parameter_value().string_value
        # self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.fixed_depth = self.get_parameter('fixed_depth').get_parameter_value().double_value
        
        # 加载YOLO模型
        self.get_logger().info(f'正在加载 YOLO 模型: {model_path}')
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'✅ YOLO 模型加载成功')
            self.get_logger().info(f'   置信度阈值: {self.conf_threshold}')
            self.get_logger().info(f'   推理设备: {device}')
        except Exception as e:
            self.get_logger().error(f'❌ 模型加载失败: {e}')
            raise e
        
        # 初始化
        self.bridge = CvBridge()
        self.rgb_timestamp = None          # 缓存RGB时间戳
        # self.latest_depth_image = None     # 最新深度图缓存（备用）
        # self.depth_image_ready = False     # 深度图是否已收到
        #
        # # 订阅深度图（取消注释即可启用真实深度）
        # self.subscription_depth = self.create_subscription(
        #     Image,
        #     self.depth_topic,
        #     self.depth_callback,
        #     10
        # )
        
        self.subscription_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        
        # 发布相机坐标系下的3D坐标（逐个）
        self.publisher_3d_pos_camera = self.create_publisher(
            ObjectInfo,
            '/tl_vision/object_3d_pos_camera',
            10
        )
        # 发布所有检测到的物体列表（批量，用于多块抓取）
        self.publisher_all_objects = self.create_publisher(
            Float64MultiArray,
            '/tl_vision/all_objects_camera',
            10
        )
        
        # 创建OpenCV显示窗口（如果启用）
        if self.show_display:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 640, 480)
            self.get_logger().info(f'OpenCV显示窗口已创建: {self.window_name}')
        
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.last_fps_update_time = time.time()
        self.fps_display = 0.0
        
        # 打印配置信息
        self.get_logger().info(f' 订阅RGB话题: {self.rgb_topic}')
        # self.get_logger().info(f' 订阅深度话题: {self.depth_topic}')
        self.get_logger().info(f' 固定深度: {self.fixed_depth}m（如需真实深度，取消depth_topic注释）')
        self.get_logger().info(f' 相机内参: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
        self.get_logger().info(f' 实时显示: {"开启" if self.show_display else "关闭"}')
        self.get_logger().info(' YOLO Demo 节点已启动，等待图像...')
    
    def rgb_callback(self, msg: Image):
        """
        RGB图像回调函数：主处理逻辑
        接收RGB图像，进行YOLO检测，计算物体3D坐标并发布
        """
        self.frame_count += 1
        self.rgb_timestamp = msg.header.stamp
        
        try:
            # RGB图像 -> OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # YOLO推理
            start_time = time.time()
            results = self.model(cv_image, conf=self.conf_threshold, imgsz=320, verbose=False)
            inference_time = (time.time() - start_time) * 1000
            self.total_inference_time += inference_time
            
            # 处理检测结果
            detections = self.process_detections(results, cv_image.shape, cv_image)
            
            # 打印检测结果
            self.print_detections(detections, inference_time)
            
            # 发布3D坐标（逐个）
            self.publish_object_3d_pos_camera(detections, msg.header.stamp)
            
            # 发布所有物体列表（批量）
            self.publish_all_objects_camera(detections, msg.header.stamp)
            
            # 实时显示
            if self.show_display:
                self.display_realtime(cv_image, detections, inference_time)
            
        except Exception as e:
            self.get_logger().error(f'图像处理出错: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def process_detections(self, results, image_shape, cv_image=None):
        """
        处理YOLO检测结果，提取检测信息并计算3D坐标
        
        :param results: YOLO推理结果
        :param image_shape: 图像尺寸 (height, width)
        :param cv_image: OpenCV图像（BGR），用于边缘角度检测
        :return: 检测结果列表，包含3D坐标信息
        """
        detections = []
        height, width = image_shape[:2]
        
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            for box in boxes:
                # 获取边界框坐标
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                
                # 计算中心点像素坐标
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                
                # 获取置信度和类别
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                cx = max(0, min(cx, width - 1))
                cy = max(0, min(cy, height - 1))
                
                # 提取深度值并计算3D坐标
                depth_m = self.get_depth_at_pixel(cx, cy)
                xyz = self.pixel_to_3d(cx, cy, depth_m) if depth_m > 0 else None
                
                # 检测物体边缘角度（异常不影响主流程）
                try:
                    angle = self.detect_edge_angle(cv_image, (x1, y1, x2, y2))
                except Exception:
                    angle = 0.0
                
                detection = {
                    'class_name': class_name,
                    'confidence': confidence,
                    'pixel_x': cx,
                    'pixel_y': cy,
                    'bbox': (x1, y1, x2, y2),
                    'depth': depth_m,
                    'point_3d': xyz,
                    'angle': angle
                }
                detections.append(detection)
        
        # 按像素X坐标左→右为主、Y坐标上→下为副排序（用于多块依次抓取）
        # 先按列（X），再按行（Y），实现逐列从上到下抓取
        detections.sort(key=lambda d: (d['pixel_x'], d['pixel_y']))
        
        return detections
    
    # ════════════════════════════════════════════
    # 深度图相关（取消注释即可启用RealSense真实深度）
    # ════════════════════════════════════════════
    # def depth_callback(self, msg: Image):
    #     try:
    #         self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
    #         if not self.depth_image_ready:
    #             self.depth_image_ready = True
    #             self.get_logger().info(f'✅ 深度图已接收，尺寸: {self.latest_depth_image.shape}')
    #     except Exception as e:
    #         self.get_logger().warn(f'深度图转换失败: {e}')

    def get_depth_at_pixel(self, u, v):
        """返回固定深度值（由 fixed_depth 参数设定）"""
        return self.fixed_depth
    
    def pixel_to_3d(self, u, v, depth):
        """
        使用针孔相机模型，将像素坐标+深度转换为相机坐标系下的3D点
        
        :param u: 像素坐标x (列)
        :param v: 像素坐标y (行)
        :param depth: 深度值（米）
        :return: (X, Y, Z) 相机坐标系下的3D坐标，无效时返回None
        """
        if depth <= 0:
            return None
        
        # 针孔相机模型公式
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        
        return (X, Y, Z)
    
    def detect_edge_angle(self, cv_image, bbox):
        """
        使用 OpenCV 边缘检测识别正方形物体的旋转角度
        
        :param cv_image: BGR 图像
        :param bbox: (x1, y1, x2, y2) 边界框像素坐标
        :return: 物体主方向角度（弧度），范围 [0, π/2)（正方形 90° 对称），检测失败返回 0.0
        """
        if cv_image is None:
            return 0.0
        
        x1, y1, x2, y2 = [int(v) for v in bbox]
        h, w = cv_image.shape[:2]
        
        # 适当扩大 ROI 区域，给边缘检测留余量
        pad = 20
        x1 = max(0, x1 - pad)
        y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad)
        y2 = min(h, y2 + pad)
        
        # 检查 ROI 尺寸是否有效
        if x2 - x1 < 10 or y2 - y1 < 10:
            return 0.0
        
        roi = cv_image[y1:y2, x1:x2]
        
        try:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        except Exception:
            return 0.0
        
        # 高斯模糊降噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 用 Otsu 自适应二值化分割物体（比 Canny 更适应光照变化）
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0
        
        # 取最大轮廓（目标物体）
        largest = max(contours, key=cv2.contourArea)
        
        # ── 用 Hough 直线检测找主方向（比图像矩更稳定） ──
        edges = cv2.Canny(blurred, 30, 100)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, minLineLength=20, maxLineGap=10)
        if lines is not None and len(lines) >= 2:
            angles = []
            for line in lines:
                x1l, y1l, x2l, y2l = line[0]
                dx = x2l - x1l
                dy = y2l - y1l
                if abs(dx) < 1 and abs(dy) < 1:
                    continue
                a = np.arctan2(dy, dx) % np.pi
                angles.append(a)
            if angles:
                hist, bins = np.histogram(angles, bins=90, range=(0, np.pi))  # 90格=2°分辨率
                best = np.argmax(hist)
                theta = (bins[best] + bins[best + 1]) / 2
                theta = theta % (np.pi / 2)       # [0, 90°) 正方形90°对称
                if theta > np.pi / 4:            # > 45° → 取补角（等效旋转，减少关节行程）
                    theta -= np.pi / 2            # [0, 45°) → (-45°, 0)，80° → -10°
                if abs(theta) < np.deg2rad(2.0):
                    theta = 0.0
                return theta
        
        # ── 备选：图像矩 ──
        m = cv2.moments(largest)
        if m['m00'] > 0:
            theta = 0.5 * np.arctan2(2 * m['mu11'], m['mu20'] - m['mu02'])
            theta = abs(theta) % (np.pi / 2)  # [0, 90°)
            if theta > np.pi / 4:
                theta -= np.pi / 2            # → (-45°, 0)
            if abs(theta) < np.deg2rad(1.0):
                theta = 0.0
            return theta
        
        return 0.0
    
    def publish_object_3d_pos_camera(self, detections, timestamp):
        """
        发布检测物体的3D坐标
        
        :param detections: 检测结果列表
        :param timestamp: 图像时间戳
        """
        for det in detections:
            if det['point_3d'] is not None:
                x, y, z = det['point_3d']
                
                # 创建PointStamped消息
                object_msg = ObjectInfo()
                object_msg.type = det['class_name']
                object_msg.pos.header.stamp = timestamp
                object_msg.pos.header.frame_id = 'camera_color_optical_frame'
                object_msg.pos.point.x = x
                object_msg.pos.point.y = y
                object_msg.pos.point.z = z
                object_msg.angle = det.get('angle', 0.0)
                
                # 发布3D坐标
                self.publisher_3d_pos_camera.publish(object_msg)

    def publish_all_objects_camera(self, detections, timestamp):
        """
        批量发布所有检测物体的3D坐标（左→右排序）
        Float64MultiArray 格式：[count, x1,y1,z1,angle1, x2,y2,z2,angle2, ...]
        """
        msg = Float64MultiArray()
        data = []
        for det in detections:
            if det['point_3d'] is not None:
                x, y, z = det['point_3d']
                data.extend([x, y, z, det.get('angle', 0.0)])

        count = len(data) // 4
        # 始终发布（包括 count=0），让 control_node 知道当前视野内方块数量已更新
        msg.data = [float(count)] + data
        self.publisher_all_objects.publish(msg)
    
    def display_realtime(self, cv_image, detections, inference_time):
        """
        实时显示检测结果（OpenCV窗口）
        
        :param cv_image: OpenCV图像
        :param detections: 检测结果列表
        :param inference_time: 推理耗时（毫秒）
        """
        # 复制图像避免修改原图
        display_img = cv_image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            
            # 根据置信度设置颜色
            if det['confidence'] > 0.7:
                color = (0, 255, 0)      # 高置信度：绿色
            elif det['confidence'] > 0.5:
                color = (0, 255, 255)    # 中置信度：黄色
            else:
                color = (0, 0, 255)      # 低置信度：红色
            
            # 绘制边界框
            cv2.rectangle(display_img, (x1, y1), (x2, y2), color, 2)
            
            # 生成标签
            angle_deg = np.rad2deg(det.get('angle', 0.0))
            label = f"{det['class_name']}: {det['confidence']:.2f} | {angle_deg:.1f}°"
            
            # 获取文本尺寸
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            
            # 绘制标签背景
            cv2.rectangle(display_img,
                         (x1, y1 - label_h - 5),
                         (x1 + label_w, y1),
                         color,
                         cv2.FILLED)
            
            # 绘制标签文字
            cv2.putText(display_img, label,
                       (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 0, 0), 2)
            
            # 绘制中心点
            cv2.circle(display_img, (det['pixel_x'], det['pixel_y']), 5, (0, 0, 255), -1)
            
            # 绘制角度方向线（从中心沿检测到的角度方向延伸）
            angle_rad = det.get('angle', 0.0)
            line_len = 40
            cx, cy = det['pixel_x'], det['pixel_y']
            ex = int(cx + line_len * np.cos(angle_rad))
            ey = int(cy + line_len * np.sin(angle_rad))
            cv2.line(display_img, (cx, cy), (ex, ey), (255, 0, 0), 2)
        
        # 计算平均耗时和FPS
        avg_time = self.total_inference_time / self.frame_count if self.frame_count > 0 else 0
        fps = 1000 / avg_time if avg_time > 0 else 0
        
        # 添加统计信息面板
        info_lines = [
            f"Frame: {self.frame_count}",
            f"Inference: {inference_time:.1f}ms",
            f"FPS: {fps:.1f}",
            f"Objects: {len(detections)}"
        ]
        
        # 绘制半透明背景
        panel_height = len(info_lines) * 25 + 10
        overlay = display_img.copy()
        cv2.rectangle(overlay, (5, 5), (230, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, display_img, 0.4, 0, display_img)
        
        # 绘制信息文字
        for i, line in enumerate(info_lines):
            y_pos = 30 + i * 25
            cv2.putText(display_img, line, (10, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 显示图像
        cv2.imshow(self.window_name, display_img)
        cv2.waitKey(1)  # 只需要1ms的等待时间来刷新窗口，不需要键盘事件
    
    def print_detections(self, detections: list, inference_time: float):
        """
        打印检测结果到终端（格式满足验收要求）
        """
        num_detections = len(detections)
        
        # 打印帧头信息
        self.get_logger().info(
            f'Frame #{self.frame_count} | 推理耗时: {inference_time:.1f}ms | '
            f'检测到 {num_detections} 个物体'
        )
        
        # 打印每个检测结果
        for i, det in enumerate(detections, 1):
            class_name = det['class_name']
            confidence = det['confidence']
            cx = det['pixel_x']
            cy = det['pixel_y']
            
            self.get_logger().info(
                f'   [{i}] Detected: {class_name} ({confidence:.2f}) at pixel ({cx}, {cy})'
            )
        
        if num_detections == 0:
            self.get_logger().info('   未检测到物体')
        
        # 每100帧打印平均性能
        if self.frame_count % 100 == 0:
            avg_time = self.total_inference_time / self.frame_count
            self.get_logger().info(
                f' 统计 | 平均推理: {avg_time:.1f}ms ({1000/avg_time:.1f} FPS)'
            )
    
    def __del__(self):
        """析构函数：关闭OpenCV窗口"""
        if self.show_display:
            cv2.destroyAllWindows()


def signal_handler(sig, frame):
    print("\n正在关闭节点...")
    cv2.destroyAllWindows()
    sys.exit(0)


def main(args=None):
    """主函数"""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.init(args=args)
    node = YOLODemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()