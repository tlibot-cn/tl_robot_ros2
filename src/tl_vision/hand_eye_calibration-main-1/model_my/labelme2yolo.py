#!/usr/bin/env python3
"""
labelme JSON → YOLO .txt 格式转换脚本

用法:
  python3 labelme2yolo.py dataset_20260622_091606

说明:
  - 读取 labelme 输出的 JSON 标注文件
  - 转换为 YOLO 格式的 .txt 文件（class x_center y_center width height）
  - class "block" 映射为 id 0
"""

import os
import sys
import json
from pathlib import Path


def convert_labelme_to_yolo(dataset_dir: str):
    dataset_path = Path(dataset_dir).resolve()
    if not dataset_path.exists():
        print(f'❌ 目录不存在: {dataset_path}')
        sys.exit(1)

    # 类别映射（按需修改）
    class_map = {'block': 0}

    json_files = sorted(dataset_path.glob('*.json'))
    # 排除没有对应图片的 JSON（labelme 生成的 labelme.json 等）
    json_files = [f for f in json_files if f.stem != 'labelme']

    if not json_files:
        print(f'❌ 没有找到 .json 文件')
        sys.exit(1)

    converted = 0
    skipped = 0

    for json_path in json_files:
        with open(json_path, 'r') as f:
            data = json.load(f)

        img_w = data.get('imageWidth', 0)
        img_h = data.get('imageHeight', 0)
        if img_w == 0 or img_h == 0:
            print(f'⚠  跳过 {json_path.name}: 无图片尺寸')
            skipped += 1
            continue

        shapes = data.get('shapes', [])
        if not shapes:
            print(f'⚠  跳过 {json_path.name}: 无标注')
            skipped += 1
            continue

        yolo_lines = []
        for shape in shapes:
            label = shape.get('label', '')
            if label not in class_map:
                print(f'⚠  未知标签 "{label}"，跳过')
                continue

            class_id = class_map[label]
            points = shape.get('points', [])

            if shape['shape_type'] == 'rectangle' and len(points) >= 2:
                # rectangle: 两个点 (左上, 右下)
                x1, y1 = points[0]
                x2, y2 = points[1]
            elif len(points) >= 2:
                # polygon: 取 bounding box
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                x1, x2 = min(xs), max(xs)
                y1, y2 = min(ys), max(ys)
            else:
                continue

            # 转 YOLO 格式: class x_center y_center width height (归一化)
            x_center = ((x1 + x2) / 2) / img_w
            y_center = ((y1 + y2) / 2) / img_h
            width = abs(x2 - x1) / img_w
            height = abs(y2 - y1) / img_h

            yolo_lines.append(f'{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}')

        if yolo_lines:
            txt_path = json_path.with_suffix('.txt')
            with open(txt_path, 'w') as f:
                f.write('\n'.join(yolo_lines) + '\n')
            converted += 1

    print(f'\n✅ 转换完成!')
    print(f'   已转换: {converted} 个文件')
    print(f'   跳过: {skipped} 个文件')
    print(f'\n现在可以运行 prepare_dataset.py:')
    print(f'   python3 prepare_dataset.py {dataset_dir} --zip')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('用法: python3 labelme2yolo.py <dataset_dir>')
        sys.exit(1)
    convert_labelme_to_yolo(sys.argv[1])
