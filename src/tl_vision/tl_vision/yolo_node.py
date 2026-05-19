#!/usr/bin/env python3
"""
YOLO Detection Demo Node for ROS2 with Depth Integration
订阅RGB图像和深度图像，运行YOLO推理，计算物体3D坐标并发布
支持实时OpenCV显示
"""

import os
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from tl_ros2_interface.msg import ObjectInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np


class YOLODemo(Node):
    
    def __init__(self):
        super().__init__('yolo_demo')
        
        _default_model_path = os.path.join(
            get_package_share_directory('tl_vision'), 'model', 'yolov8n.pt'
        )
        
        self.declare_parameter('model_path', _default_model_path)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera.fx', 610.0)   # 焦距 x (像素单位)
        self.declare_parameter('camera.fy', 610.0)   # 焦距 y (像素单位)
        self.declare_parameter('camera.cx', 320.0)   # 光心 x
        self.declare_parameter('camera.cy', 240.0)   # 光心 y
        self.declare_parameter('depth_median_window', 10)  # 中位数滤波窗口大小（奇数）
        self.declare_parameter('max_valid_depth', 10.0)    # 最大有效深度（米）
        self.declare_parameter('min_valid_depth', 0.1)     # 最小有效深度（米）
        self.declare_parameter('show_display', True)       # 是否显示OpenCV窗口
        self.declare_parameter('display_window_name', 'YOLO Detection')  # 窗口名称
        
        # 获取参数值
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('tl_vision'), 'model', 'yolov8n.pt'
            )
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value
        
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        
        self.fx = self.get_parameter('camera.fx').get_parameter_value().double_value
        self.fy = self.get_parameter('camera.fy').get_parameter_value().double_value
        self.cx = self.get_parameter('camera.cx').get_parameter_value().double_value
        self.cy = self.get_parameter('camera.cy').get_parameter_value().double_value
        
        self.depth_median_window = self.get_parameter('depth_median_window').get_parameter_value().integer_value
        self.max_valid_depth = self.get_parameter('max_valid_depth').get_parameter_value().double_value
        self.min_valid_depth = self.get_parameter('min_valid_depth').get_parameter_value().double_value
        self.show_display = self.get_parameter('show_display').get_parameter_value().bool_value
        self.window_name = self.get_parameter('display_window_name').get_parameter_value().string_value
        
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
        self.latest_depth = None           # 缓存最新的深度图像
        self.rgb_timestamp = None          # 缓存RGB时间戳
        self.depth_timestamp = None        # 缓存深度时间戳
        
        # 配置QoS（深度图像通常使用传感器数据QoS）
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        
        self.subscription_depth = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            depth_qos
        )
        
        # 发布相机坐标系下的3D坐标
        self.publisher_3d_pos_camera = self.create_publisher(
            ObjectInfo,
            '/tl_vision/object_3d_pos_camera',
            10
        )
        
        # 创建OpenCV显示窗口（如果启用）
        if self.show_display:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)
            self.get_logger().info(f'🖼️ OpenCV显示窗口已创建: {self.window_name}')
        
        self.frame_count = 0
        self.valid_depth_count = 0
        self.total_inference_time = 0.0
        self.last_fps_update_time = time.time()
        self.fps_display = 0.0
        
        # 打印配置信息
        self.get_logger().info(f'📷 订阅RGB话题: {self.rgb_topic}')
        self.get_logger().info(f'📏 订阅深度话题: {self.depth_topic}')
        self.get_logger().info(f'📷 相机内参: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
        self.get_logger().info(f'🔧 深度中位数滤波窗口: {self.depth_median_window}x{self.depth_median_window}')
        self.get_logger().info(f'🖼️ 实时显示: {"开启" if self.show_display else "关闭"}')
        self.get_logger().info('🚀 YOLO Demo 节点已启动，等待图像...')
    
    def depth_callback(self, msg: Image):
        """深度图像回调函数：缓存最新的深度图"""
        try:
            # 将ROS深度图像转为OpenCV格式（16UC1，单位毫米）
            depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.latest_depth = depth_image
            self.depth_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'深度图像处理出错: {e}')
    
    def rgb_callback(self, msg: Image):
        """
        RGB图像回调函数：主处理逻辑
        接收RGB图像，进行YOLO检测，结合深度图计算3D坐标
        """
        self.frame_count += 1
        self.rgb_timestamp = msg.header.stamp
        
        try:
            # RGB图像 -> OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # YOLO推理
            start_time = time.time()
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
            inference_time = (time.time() - start_time) * 1000
            self.total_inference_time += inference_time
            
            # 处理检测结果
            detections = self.process_detections(results, cv_image.shape)
            
            # 打印检测结果
            self.print_detections(detections, inference_time)
            
            # 发布3D坐标
            self.publish_object_3d_pos_camera(detections, msg.header.stamp)
            
            # 实时显示
            if self.show_display:
                self.display_realtime(cv_image, detections, inference_time)
            
        except Exception as e:
            self.get_logger().error(f'图像处理出错: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def process_detections(self, results, image_shape):
        """
        处理YOLO检测结果，提取检测信息并计算3D坐标
        
        :param results: YOLO推理结果
        :param image_shape: 图像尺寸 (height, width)
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
                
                detection = {
                    'class_name': class_name,
                    'confidence': confidence,
                    'pixel_x': cx,
                    'pixel_y': cy,
                    'bbox': (x1, y1, x2, y2),
                    'depth': depth_m,
                    'point_3d': xyz
                }
                detections.append(detection)
        
        return detections
    
    def get_depth_at_pixel(self, u, v):
        """
        从深度图像中提取指定像素位置的深度值
        
        :param u: 像素坐标x (列)
        :param v: 像素坐标y (行)
        :return: 深度值（米），无效时返回0
        """
        if self.latest_depth is None:
            return 0.0
        
        try:
            height, width = self.latest_depth.shape
            
            if u < 0 or u >= width or v < 0 or v >= height:
                return 0.0
            
            # 简化实现：直接读取单点深度值
            depth_mm = self.latest_depth[v, u]
            
            # 过滤无效深度（0表示无效）
            if depth_mm == 0:
                return 0.0
            
            # 转换为米
            depth_m = depth_mm / 1000.0
            
            # 过滤超出有效范围的深度
            if depth_m < self.min_valid_depth or depth_m > self.max_valid_depth:
                return 0.0
            
            # 如果启用中位数滤波，使用周围区域的中位数替代单点值
            half_window = self.depth_median_window // 2
            if half_window > 0:
                # 定义ROI区域
                roi_top = max(0, v - half_window)
                roi_bottom = min(height, v + half_window + 1)
                roi_left = max(0, u - half_window)
                roi_right = min(width, u + half_window + 1)
                
                # 提取ROI区域的有效深度值
                roi = self.latest_depth[roi_top:roi_bottom, roi_left:roi_right]
                valid_depths = roi[roi > 0]  # 过滤无效值
                
                if len(valid_depths) > 0:
                    median_mm = np.median(valid_depths)
                    depth_m = median_mm / 1000.0
                    
                    # 再次过滤阈值
                    if depth_m < self.min_valid_depth or depth_m > self.max_valid_depth:
                        return 0.0
                else:
                    return 0.0
            
            self.valid_depth_count += 1
            return depth_m
            
        except Exception as e:
            self.get_logger().debug(f'深度值提取出错: {e}')
            return 0.0
    
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
                
                # 发布3D坐标
                self.publisher_3d_pos_camera.publish(object_msg)
    
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
            label = f"{det['class_name']}: {det['confidence']:.2f}"
            if det['depth'] > 0:
                label += f" | Z={det['depth']:.2f}m"
            
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
            f'📸 Frame #{self.frame_count} | 推理耗时: {inference_time:.1f}ms | '
            f'检测到 {num_detections} 个物体'
        )
        
        # 打印每个检测结果
        for i, det in enumerate(detections, 1):
            class_name = det['class_name']
            confidence = det['confidence']
            cx = det['pixel_x']
            cy = det['pixel_y']
            depth = det['depth']
            
            if depth > 0:
                xyz = det['point_3d']
                self.get_logger().info(
                    f'   [{i}] Detected: {class_name} ({confidence:.2f}) at pixel ({cx}, {cy}) '
                    f'| 深度: {depth:.3f}m | 3D: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})'
                )
            else:
                self.get_logger().info(
                    f'   [{i}] Detected: {class_name} ({confidence:.2f}) at pixel ({cx}, {cy}) '
                    f'| ⚠️ 无有效深度'
                )
        
        if num_detections == 0:
            self.get_logger().info('   ⚠️ 未检测到物体')
        
        # 每100帧打印平均性能
        if self.frame_count % 100 == 0:
            avg_time = self.total_inference_time / self.frame_count
            valid_rate = self.valid_depth_count / self.frame_count * 100
            self.get_logger().info(
                f'📊 统计 | 平均推理: {avg_time:.1f}ms ({1000/avg_time:.1f} FPS) | '
                f'有效深度率: {valid_rate:.1f}%'
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