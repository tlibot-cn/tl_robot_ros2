#!/usr/bin/env python3
"""
拍照脚本 — 用于采集 YOLO 训练数据集

按键:
  k  拍照保存
  q  退出

保存格式:
  model_my/dataset_YYYYMMDD_HHMMSS/
    ├── img_0001.jpg
    ├── img_0002.jpg
    └── ...

LabelImg 标注后会在同级生成对应的 img_xxxx.txt 文件。
"""

import cv2
import os
import sys
import time
import numpy as np
from datetime import datetime


def open_camera_realsense():
    """使用 pyrealsense2 打开 RealSense 相机。"""
    try:
        import pyrealsense2 as rs
    except ImportError:
        print('⚠  pyrealsense2 未安装')
        return None, None

    pipeline = rs.pipeline()
    config = rs.config()

    # 启用 RGB 流（1280x720）
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    try:
        profile = pipeline.start(config)
        print(f'✅ RealSense 相机已打开')
        print(f'   RGB 分辨率: 1280x720')
        return 'realsense', pipeline
    except Exception as e:
        print(f'⚠  pyrealsense2 打开失败: {e}')
        return None, None


def open_camera_opencv():
    """使用 OpenCV VideoCapture 回退方案（RealSense 也支持 UVC 协议）。"""
    for index in [4, 6, 8, 0, 2]:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if cap.isOpened():
            # 设置为 1280x720
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w >= 640:
                print(f'✅ OpenCV 相机已打开 (index={index})')
                print(f'   RGB 分辨率: {w}x{h}')
                return 'opencv', cap
            cap.release()
    return None, None


def open_camera():
    """自动选择方式打开相机：优先 pyrealsense2，回退 OpenCV。"""
    mode, handle = open_camera_realsense()
    if handle is not None:
        return mode, handle

    print('⚠  pyrealsense2 不可用，尝试 OpenCV 回退...')
    mode, handle = open_camera_opencv()
    if handle is not None:
        return mode, handle

    print('❌ 所有方式都无法打开相机')
    return None, None


def grab_frame(mode, handle):
    """从相机获取一帧 RGB 图像。"""
    if mode == 'realsense':
        frames = handle.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())
    else:
        ret, frame = handle.read()
        if not ret:
            return None
        return frame


def release_camera(mode, handle):
    """释放相机资源。"""
    if mode == 'realsense':
        handle.stop()
    else:
        handle.release()


def main():
    # 自动选择相机
    mode, handle = open_camera()
    if handle is None:
        print('💡 提示: 确保 RealSense 相机已连接，或检查 /dev/video* 设备')
        sys.exit(1)

    # 创建保存目录
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    save_dir = os.path.join(os.path.dirname(__file__), f'dataset_{timestamp}')
    os.makedirs(save_dir, exist_ok=True)
    print(f'📁 图片保存到: {save_dir}')

    count = 0
    window_name = 'Photo Capture (k:拍照  q:退出)'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    print('💡 按 k 拍照 | 按 q 退出')

    try:
        while True:
            # 获取一帧
            frame = grab_frame(mode, handle)
            if frame is None:
                continue

            # 在画面左上角显示拍照数量
            info = f'Count: {count}'
            cv2.putText(frame, info, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print('👋 退出')
                break

            elif key == ord('k'):
                count += 1
                filename = f'img_{count:04d}.jpg'
                filepath = os.path.join(save_dir, filename)
                cv2.imwrite(filepath, frame)
                print(f'  ✅ [{count:04d}] 已保存: {filename}')

    finally:
        release_camera(mode, handle)
        cv2.destroyAllWindows()

    print(f'\n📊 共拍摄 {count} 张图片')
    print(f'📁 路径: {save_dir}')
    print('➡  下一步: 用 LabelImg 打开该文件夹进行标注')
    save_dir = os.path.join(os.path.dirname(__file__), f'dataset_{timestamp}')
    os.makedirs(save_dir, exist_ok=True)
    print(f'📁 图片保存到: {save_dir}')

    count = 0
    window_name = 'Photo Capture (k:拍照  q:退出)'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print('💡 按 k 拍照 | 按 q 退出')

    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ 读取摄像头失败')
            break

        # 在画面左上角显示当前拍照数量
        info = f'Count: {count}'
        cv2.putText(frame, info, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print('👋 退出')
            break

        elif key == ord('k'):
            count += 1
            filename = f'img_{count:04d}.jpg'
            filepath = os.path.join(save_dir, filename)
            cv2.imwrite(filepath, frame)
            print(f'  ✅ [{count:04d}] 已保存: {filename}')

    cap.release()
    cv2.destroyAllWindows()
    print(f'\n📊 共拍摄 {count} 张图片')
    print(f'📁 路径: {save_dir}')
    print('➡  下一步: 用 LabelImg 打开该文件夹进行标注')


if __name__ == '__main__':
    main()
