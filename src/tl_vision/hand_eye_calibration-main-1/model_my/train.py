#!/usr/bin/env python3
"""
YOLO 训练脚本 — 在拍好的数据集上训练自定义模型

使用方法:
  1. 用 photo.py 拍照
  2. 用 LabelImg 标注（格式选 YOLO，class = "block"）
  3. 运行 prepare_dataset.py 划分数据集
  4. 运行本脚本训练

训练完成后:
  - 模型保存在 runs/detect/custom_yolo/weights/best.pt
  - 复制到 tl_vision/model/ 替换旧模型
"""

import sys
import torch
from ultralytics import YOLO

# ===== 修改这里指向你的新数据集 =====
DATA_YAML = 'dataset_20260622_091606/data.yaml'

# 自动检测设备
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f'🔧 使用设备: {device}')

# 加载预训练模型
model = YOLO('yolov8s.pt')

# 开始训练
model.train(
    data=DATA_YAML,
    epochs=100,                # 训练轮数
    imgsz=640,                 # 输入图片尺寸
    batch=8,                   # 批次大小（CPU 用小 batch）
    device=device,             # 自动选择 GPU/CPU
    workers=2,                 # 数据加载线程数
    patience=30,               # 早停，30轮无提升就停止
    name='block_20260622',     # 训练名称（注意：每个新训练改个名字避免覆盖）
    exist_ok=False,            # 不覆盖已有结果
    verbose=True,
)

print('\n✅ 训练完成！')
print('📁 模型保存在: runs/detect/block_20260622/weights/best.pt')
print('➡  复制到 tl_vision/model/ 目录下')
print('➡  修改 yolo_node.yaml 中的 model_path')
