#!/usr/bin/env python3
"""
数据集准备脚本 — 标注后执行

使用方法:
  1. 用 photo.py 拍照 → 生成 dataset_YYYYMMDD_HHMMSS/
  2. 用 LabelImg 标注（YOLO 格式，class = "block"）
  3. 运行本脚本：
       python3 prepare_dataset.py dataset_20260622_091606

  脚本会:
    - 检查每张图片是否有对应的 .txt 标注
    - 按 7:3 随机划分训练集/验证集
    - 生成 data.yaml（class: block）
    - 可选打包为 dataset_switch.zip（用于 Colab/Kaggle 训练）

输出结构:
  dataset_20260622_091606/
  ├── images/
  │   ├── train/    ← 70% 图片
  │   └── val/      ← 30% 图片
  ├── labels/
  │   ├── train/    ← 对应的 .txt
  │   └── val/
  └── data.yaml     ← 数据集配置
"""

import os
import sys
import random
import shutil
import argparse
from pathlib import Path


def prepare_dataset(dataset_dir: str, val_ratio: float = 0.3, seed: int = 42, zip_output: bool = False):
    dataset_path = Path(dataset_dir).resolve()

    if not dataset_path.exists():
        print(f'❌ 数据集目录不存在: {dataset_path}')
        sys.exit(1)

    # 收集所有 jpg 文件
    images = sorted(dataset_path.glob('*.jpg'))
    print(f'📸 图片总数: {len(images)}')

    if len(images) == 0:
        print('❌ 没有找到 .jpg 图片')
        sys.exit(1)

    # 检查标注
    missing_labels = []
    labeled_images = []
    for img in images:
        txt = img.with_suffix('.txt')
        if not txt.exists():
            missing_labels.append(img.name)
        else:
            # 检查标注文件是否非空
            if txt.stat().st_size == 0:
                print(f'⚠  空标注文件: {txt.name}（跳过）')
                missing_labels.append(img.name)
            else:
                labeled_images.append(img)

    if missing_labels:
        print(f'⚠  有 {len(missing_labels)} 张图片缺少标注:')
        for name in missing_labels[:10]:
            print(f'   - {name}')
        if len(missing_labels) > 10:
            print(f'   ... 共 {len(missing_labels)} 张')
        print(f'\n❌ 请先用 LabelImg 标注所有图片后再运行本脚本')
        print(f'   启动标注: labelImg {dataset_path}')

        ans = input(f'\n仍要继续（跳过未标注图片）？[y/N]: ')
        if ans.lower() != 'y':
            sys.exit(1)

    print(f'✅ 已标注: {len(labeled_images)} 张')

    if len(labeled_images) < 5:
        print('❌ 标注图片太少，无法训练')
        sys.exit(1)

    # 创建目录结构
    (dataset_path / 'images' / 'train').mkdir(parents=True, exist_ok=True)
    (dataset_path / 'images' / 'val').mkdir(parents=True, exist_ok=True)
    (dataset_path / 'labels' / 'train').mkdir(parents=True, exist_ok=True)
    (dataset_path / 'labels' / 'val').mkdir(parents=True, exist_ok=True)

    # 随机打乱并划分
    random.seed(seed)
    random.shuffle(labeled_images)

    split_idx = int(len(labeled_images) * (1 - val_ratio))
    train_imgs = labeled_images[:split_idx]
    val_imgs = labeled_images[split_idx:]

    print(f'\n📊 划分结果:')
    print(f'   训练集: {len(train_imgs)} 张')
    print(f'   验证集: {len(val_imgs)} 张')

    # 复制文件
    def copy_files(img_list, subset):
        for img in img_list:
            # 复制图片
            shutil.copy2(str(img), str(dataset_path / 'images' / subset / img.name))
            # 复制标注
            txt_src = img.with_suffix('.txt')
            shutil.copy2(str(txt_src), str(dataset_path / 'labels' / subset / txt_src.name))

    copy_files(train_imgs, 'train')
    copy_files(val_imgs, 'val')
    print('✅ 文件复制完成')

    # 生成 data.yaml
    yaml_content = f"""train: ./images/train
val: ./images/val
nc: 1
names: ['block']
"""
    yaml_path = dataset_path / 'data.yaml'
    yaml_path.write_text(yaml_content)
    print(f'✅ data.yaml 已生成: {yaml_path}')
    print(f'   类别: block')

    # 可选打包
    if zip_output:
        import zipfile
        zip_path = dataset_path.parent / 'dataset_switch.zip'
        print(f'\n📦 正在打包为 {zip_path} ...')
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
            for root, _, files in os.walk(dataset_path):
                for f in files:
                    full_path = os.path.join(root, f)
                    arcname = os.path.relpath(full_path, dataset_path)
                    # 只打包 images/, labels/, data.yaml
                    if arcname.startswith(('images/', 'labels/', 'data.yaml')):
                        zf.write(full_path, arcname)
        print(f'✅ 打包完成: {zip_path}')
        print(f'   上传到 Google Drive 后可用 Colab 训练')

    print(f'\n🎉 数据集准备完成！')
    print(f'📂 路径: {dataset_path}')
    print(f'')
    print(f'💻 本地训练（CPU 较慢）:')
    print(f'   python3 train.py')
    print(f'')
    print(f'☁️  云端训练（推荐, GPU 加速）:')
    print(f'   1. 将 dataset_switch.zip 上传到 Google Drive')
    print(f'   2. 打开 train_colab.ipynb 运行')
    print(f'   3. 训练完成后将 best.pt 下载到 model/ 目录')
    print(f'')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='准备 YOLO 数据集')
    parser.add_argument('dataset', help='数据集目录名（如 dataset_20260622_091606）')
    parser.add_argument('--val-ratio', type=float, default=0.3, help='验证集比例 (默认: 0.3)')
    parser.add_argument('--seed', type=int, default=42, help='随机种子 (默认: 42)')
    parser.add_argument('--zip', action='store_true', help='打包为 dataset_switch.zip')
    args = parser.parse_args()

    prepare_dataset(args.dataset, args.val_ratio, args.seed, args.zip)
