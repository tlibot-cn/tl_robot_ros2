# coding=utf-8

"""
眼在手上 用采集到的图片信息和机械臂位姿信息计算 相机坐标系相对于机械臂末端坐标系的 旋转矩阵和平移向量
A2^{-1}*A1*X=X*B2*B1^{−1}
"""

import os
import logging
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib
# 设置中文字体支持
def print_font_installation_guide():
    """打印中文字体安装指南"""
    import platform
    system = platform.system()
    
    print("\n" + "="*60)
    print("中文字体安装指南")
    print("="*60)
    
    if system == "Windows":
        print("Windows系统:")
        print("1. 系统通常已包含中文字体，如SimHei、Microsoft YaHei")
        print("2. 如果仍无法显示，请检查字体是否已安装")
        print("3. 可通过控制面板 > 字体 查看已安装字体")
        
    elif system == "Linux":
        print("Linux系统:")
        print("1. Ubuntu/Debian: sudo apt-get install fonts-wqy-microhei")
        print("2. CentOS/RHEL: sudo yum install wqy-microhei-fonts")
        print("3. 或者安装Google Noto字体:")
        print("   sudo apt-get install fonts-noto-cjk")
        print("4. 安装后清除matplotlib字体缓存:")
        print("   python -c \"import matplotlib.font_manager; matplotlib.font_manager._rebuild()\"")
        
    elif system == "Darwin":  # macOS
        print("macOS系统:")
        print("1. 系统通常已包含中文字体，如PingFang SC、Hiragino Sans GB")
        print("2. 如果仍无法显示，可安装额外字体:")
        print("3. 下载并安装Adobe思源黑体或Google Noto字体")
        
    print("\n通用解决方案:")
    print("1. 重启Python程序")
    print("2. 清除matplotlib字体缓存:")
    print("   python -c \"import matplotlib.font_manager; matplotlib.font_manager._rebuild()\"")
    print("3. 如果问题持续，程序将使用英文标签")
    print("="*60 + "\n")

def setup_chinese_font():
    # """设置matplotlib中文字体支持"""
    # try:
    #     # 尝试不同的中文字体
    #     chinese_fonts = [
    #         'SimHei',           # Windows 黑体
    #         'Microsoft YaHei',  # Windows 微软雅黑
    #         'WenQuanYi Micro Hei',  # Linux 文泉驿微米黑
    #         'WenQuanYi Zen Hei',    # Linux 文泉驿正黑
    #         'Noto Sans CJK SC',     # Google Noto字体
    #         'Source Han Sans SC',   # Adobe思源黑体
    #         'PingFang SC',          # macOS 苹方
    #         'Hiragino Sans GB',     # macOS 冬青黑体
    #         'STHeiti',              # macOS 华文黑体
    #         'Arial Unicode MS'      # 备用字体
    #     ]
        
    #     # 获取系统可用字体
    #     from matplotlib.font_manager import FontProperties, findSystemFonts
    #     available_fonts = [f.name for f in matplotlib.font_manager.fontManager.ttflist]
        
    #     # 找到第一个可用的中文字体
    #     selected_font = None
    #     for font in chinese_fonts:
    #         if font in available_fonts:
    #             selected_font = font
    #             break
        
    #     if selected_font:
    #         matplotlib.rcParams['font.sans-serif'] = [selected_font] + matplotlib.rcParams['font.sans-serif']
    #         print(f"已设置中文字体: {selected_font}")
    #     else:
    #         print("警告: 未找到合适的中文字体，图表中的中文可能显示为方块")
    #         print_font_installation_guide()
    #         # 使用默认字体列表
    #         matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
        
    #     matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
        
    # except Exception as e:
    #     print(f"设置中文字体时出错: {e}")
    #     # 使用基本设置
    #     matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
    #     matplotlib.rcParams['axes.unicode_minus'] = False
    """论文级：中文宋体 + 英文 Times New Roman"""

    import matplotlib
    import matplotlib.font_manager as fm

    # ===== 字体候选 =====
    chinese_fonts = [
        "SimSun",              # 宋体（Windows最标准）
        "Songti SC",           # macOS宋体
        "Noto Serif CJK SC",   # Linux推荐（更像宋体）
    ]

    english_fonts = [
        "Times New Roman",
        "Times",
    ]

    available_fonts = [f.name for f in fm.fontManager.ttflist]

    selected_cn = None
    selected_en = None

    # 找中文字体
    for f in chinese_fonts:
        if f in available_fonts:
            selected_cn = f
            break

    # 找英文字体
    for f in english_fonts:
        if f in available_fonts:
            selected_en = f
            break

    # ===== 设置策略（关键）=====
    font_list = []

    if selected_en:
        font_list.append(selected_en)

    if selected_cn:
        font_list.append(selected_cn)

    # fallback
    font_list += ["DejaVu Sans"]

    matplotlib.rcParams["font.family"] = "serif"
    matplotlib.rcParams["font.serif"] = font_list
    matplotlib.rcParams["axes.unicode_minus"] = False

    print(f"英文字体: {selected_en}")
    print(f"中文字体: {selected_cn}")

# 初始化中文字体
setup_chinese_font()

import  yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog

from save_poses import poses_main

np.set_printoptions(precision=8,suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


current_path = os.path.dirname(os.path.abspath(__file__))

images_path = os.path.join(current_path, "eye_hand_data", "data2025092601")

file_path = os.path.join(images_path, "poses.txt")

print("images_path:", images_path)
print("file_path:", file_path)


with open("config.yaml", 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)

XX = data.get("checkerboard_args").get("XX") #标定板的中长度对应的角点的个数
YY = data.get("checkerboard_args").get("YY") #标定板的中宽度对应的角点的个数
L = data.get("checkerboard_args").get("L")   #标定板一格的长度  单位为米


def calculate_reprojection_error(obj_points, img_points, rvecs, tvecs, camera_matrix, dist_coeffs):
    """
    计算重投影误差
    Args:
        obj_points: 3D点
        img_points: 2D点
        rvecs: 旋转向量
        tvecs: 平移向量
        camera_matrix: 相机内参矩阵
        dist_coeffs: 畸变系数
    Returns:
        errors: 每张图片的重投影误差列表
        mean_error: 平均重投影误差
    """
    errors = []
    for i in range(len(obj_points)):
        # 将3D点投影到2D
        projected_points, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        
        # 计算重投影误差
        error = cv2.norm(img_points[i], projected_points, cv2.NORM_L2) / len(projected_points)
        errors.append(error)
    
    mean_error = np.mean(errors)
    return errors, mean_error


def select_images_interactive(images_path, XX, YY, L):
    """
    交互式选择标定图片
    Args:
        images_path: 图片路径
        XX, YY: 棋盘格尺寸
        L: 格子大小
    Returns:
        selected_indices: 选中的图片索引列表
    """
    print("开始交互式图片选择...")
    print("将显示每张图片，请按以下键选择：")
    print("  'y' 或 'Enter': 保留此图片")
    print("  'n' 或 'Space': 删除此图片")
    print("  'q': 退出选择")
    print("  'a': 全选")
    print("  'd': 全删")
    
    # 获取所有图片文件
    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]
    images_num.sort(key=lambda x: int(x.split('.')[0]))  # 按数字排序
    
    selected_indices = []
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    
    # 准备棋盘格角点
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    objp = L * objp
    
    for i, image_file in enumerate(images_num):
        image_path = os.path.join(images_path, image_file)
        img = cv2.imread(image_path)
        
        if img is None:
            print(f"无法读取图片: {image_file}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
        
        # 创建显示图片
        display_img = img.copy()
        
        if ret:
            # 找到角点，绘制
            corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
            cv2.drawChessboardCorners(display_img, (XX, YY), corners2, ret)
            
            # 添加状态信息
            cv2.putText(display_img, f"Image {i+1}/{len(images_num)}: {image_file}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_img, "Chessboard Detected - Press 'y' to keep, 'n' to delete", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # 未找到角点
            cv2.putText(display_img, f"Image {i+1}/{len(images_num)}: {image_file}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_img, "No Chessboard Detected - Press 'n' to delete", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 显示图片
        cv2.imshow("Image Selection", display_img)
        
        while True:
            key = cv2.waitKey(0) & 0xFF
            
            if key == ord('q'):
                print("退出图片选择")
                cv2.destroyAllWindows()
                return selected_indices
            elif key == ord('a'):
                # 全选
                selected_indices = list(range(len(images_num)))
                print("已全选所有图片")
                break
            elif key == ord('d'):
                # 全删
                selected_indices = []
                print("已删除所有图片")
                break
            elif key == ord('y') or key == 13:  # Enter键
                if i not in selected_indices:
                    selected_indices.append(i)
                print(f"保留图片: {image_file}")
                break
            elif key == ord('n') or key == 32:  # Space键
                if i in selected_indices:
                    selected_indices.remove(i)
                print(f"删除图片: {image_file}")
                break
            else:
                print("无效按键，请按 'y'(保留), 'n'(删除), 'q'(退出), 'a'(全选), 'd'(全删)")
    
    cv2.destroyAllWindows()
    
    # 排序选中的索引
    selected_indices.sort()
    print(f"\n图片选择完成！共选中 {len(selected_indices)} 张图片")
    print("选中的图片索引:", selected_indices)
    
    return selected_indices

def plot_reprojection_errors(errors, valid_images,
                            save_path1="reprojection_error_bar.png",
                            save_path2="reprojection_error_hist.png"):
    """
    绘制重投影误差图（分成两张图）
    """

    try:
        setup_chinese_font()

        errors = np.array(errors)

        # ===============================
        # 图1：柱状图（每张图误差）
        # ===============================
        plt.figure(figsize=(8, 5))

        x_pos = range(len(errors))
        bars = plt.bar(x_pos, errors, alpha=0.7)

        # 均值 & 阈值
        mean_error = np.mean(errors)
        std_error = np.std(errors)
        high_threshold = mean_error + 2 * std_error

        plt.axhline(y=mean_error, linestyle='--', linewidth=2,
                    label=f'Mean Error: {mean_error:.4f}px')

        plt.axhline(y=high_threshold, linestyle=':', linewidth=2,
                    label=f'High Error Threshold: {high_threshold:.4f}px')

        # 标记异常点
        for bar, error in zip(bars, errors):
            if error > high_threshold:
                bar.set_alpha(0.9)

        plt.xlabel('Image Index')
        plt.ylabel('Reprojection Error(pixels)')
        plt.title('Reprojection Error per Image')

        plt.grid(True, alpha=0.3)
        plt.legend()

        plt.tight_layout()
        plt.savefig(save_path1, dpi=600)
        plt.close()

        # ===============================
        # 图2：误差直方图
        # ===============================
        plt.figure(figsize=(8, 5))

        plt.hist(errors, bins=min(10, len(errors)), alpha=0.7, edgecolor='black')

        plt.axvline(x=mean_error, linestyle='--', linewidth=2,
                    label=f'Mean Error: {mean_error:.4f}px')

        plt.xlabel('Reprojection Error(pixels)')
        plt.ylabel('Frequency')
        plt.title('Reprojection Error Distribution Histogram')

        plt.grid(True, alpha=0.3)
        plt.legend()

        plt.tight_layout()
        plt.savefig(save_path2, dpi=600)
        plt.close()

        print(f"图1已保存: {save_path1}")
        print(f"图2已保存: {save_path2}")

    except Exception as e:
        print(f"绘图失败: {e}")

def func():

    path = os.path.dirname(__file__)

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = L*objp

    # 第一步：交互式选择图片
    print("=" * 60)
    print("第一步：交互式选择标定图片")
    print("=" * 60)
    
    selected_indices = select_images_interactive(images_path, XX, YY, L)
    
    if len(selected_indices) < 3:
        print("错误：至少需要3张图片进行标定！")
        return None, None
    
    print(f"已选择 {len(selected_indices)} 张图片进行标定")

    # 第二步：处理选中的图片
    print("\n" + "=" * 60)
    print("第二步：处理选中的图片")
    print("=" * 60)
    
    obj_points = []     # 存储3D点
    img_points = []     # 存储2D点
    valid_images = []   # 存储有效图片信息

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]
    images_num.sort(key=lambda x: int(x.split('.')[0]))  # 按数字排序

    for idx in selected_indices:
        if idx < len(images_num):
            image_file = images_num[idx]
            image_path = os.path.join(images_path, image_file)

            logger_.info(f'处理图片: {image_file}')

            img = cv2.imread(image_path)
            if img is None:
                print(f"警告：无法读取图片 {image_file}")
                continue
                
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:
                obj_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                if corners2 is not None:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)
                valid_images.append(image_file)
                print(f"  ✓ 成功检测到棋盘格: {image_file}")
            else:
                print(f"  ✗ 未检测到棋盘格: {image_file}")

    N = len(img_points)
    print(f"\n有效图片数量: {N}")

    if N < 3:
        print("错误：有效图片数量不足，至少需要3张图片！")
        return None, None

    # 第三步：相机标定
    print("\n" + "=" * 60)
    print("第三步：相机标定")
    print("=" * 60)
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    if not ret:
        print("错误：相机标定失败！")
        return None, None

    print("相机标定成功！")
    print(f"内参矩阵:\n{mtx}")
    print(f"畸变系数:\n{dist}")

    # 第四步：计算重投影误差
    print("\n" + "=" * 60)
    print("第四步：计算重投影误差")
    print("=" * 60)
    
    errors, mean_error = calculate_reprojection_error(obj_points, img_points, rvecs, tvecs, mtx, dist)
    
    print("每张图片的重投影误差:")
    for i, (error, img_name) in enumerate(zip(errors, valid_images)):
        print(f"  {img_name}: {error:.4f} 像素")
    
    print(f"\n平均重投影误差: {mean_error:.4f} 像素")
    
    # 显示误差统计
    max_error = np.max(errors)
    min_error = np.min(errors)
    std_error = np.std(errors)
    
    print(f"最大误差: {max_error:.4f} 像素")
    print(f"最小误差: {min_error:.4f} 像素")
    print(f"标准差: {std_error:.4f} 像素")
    
    # 标记高误差图片
    high_error_threshold = mean_error + 2 * std_error
    high_error_images = []
    for i, (error, img_name) in enumerate(zip(errors, valid_images)):
        if error > high_error_threshold:
            high_error_images.append((img_name, error))
    
    if high_error_images:
        print(f"\n警告：以下图片误差较高（>{high_error_threshold:.4f}像素）:")
        for img_name, error in high_error_images:
            print(f"  {img_name}: {error:.4f} 像素")
        print("建议重新拍摄这些图片或从标定中排除")

    # 第五步：手眼标定
    print("\n" + "=" * 60)
    print("第五步：手眼标定")
    print("=" * 60)

    poses_main(file_path)
    # 机器人末端在基座标系下的位姿

    csv_file = os.path.join(path,"RobotToolPose.csv")
    tool_pose = np.loadtxt(csv_file,delimiter=',')

    R_tool = []
    t_tool = []

    for i in range(int(N)):
        R_tool.append(tool_pose[0:3,4*i:4*i+3])
        t_tool.append(tool_pose[0:3,4*i+3])

    R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)

    print("手眼标定完成！")
    
    return R, t, errors, mean_error, valid_images

if __name__ == '__main__':

    print("手眼标定程序启动")
    print("=" * 60)
    
    # 执行标定
    result = func()
    
    if result is None or len(result) < 2:
        print("标定失败！")
        exit(1)
    
    if len(result) == 5:
        # 新版本返回更多信息
        rotation_matrix, translation_vector, errors, mean_error, valid_images = result
    else:
        # 兼容旧版本
        rotation_matrix, translation_vector = result
        errors, mean_error, valid_images = None, None, None

    # 将旋转矩阵转换为四元数
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    x, y, z = translation_vector.flatten()

    print("\n" + "=" * 60)
    print("标定结果")
    print("=" * 60)
    
    logger_.info(f"旋转矩阵是:\n{rotation_matrix}")
    logger_.info(f"平移向量是:\n{translation_vector}")
    logger_.info(f"四元数是：\n{quaternion}")
    
    print(f"旋转矩阵:\n{rotation_matrix}")
    print(f"平移向量: [{x:.6f}, {y:.6f}, {z:.6f}]")
    print(f"四元数: [{quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}]")
    
    if errors is not None:
        print(f"\n重投影误差统计:")
        print(f"平均误差: {mean_error:.4f} 像素")
        print(f"最大误差: {np.max(errors):.4f} 像素")
        print(f"最小误差: {np.min(errors):.4f} 像素")
        print(f"标准差: {np.std(errors):.4f} 像素")
        
        # 保存误差报告到标定图片目录
        error_report_file = os.path.join(images_path, "reprojection_error_report.txt")
        with open(error_report_file, 'w', encoding='utf-8') as f:
            f.write("手眼标定重投影误差报告\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"标定图片数量: {len(valid_images)}\n")
            f.write(f"平均重投影误差: {mean_error:.4f} 像素\n")
            f.write(f"最大误差: {np.max(errors):.4f} 像素\n")
            f.write(f"最小误差: {np.min(errors):.4f} 像素\n")
            f.write(f"标准差: {np.std(errors):.4f} 像素\n\n")
            f.write("每张图片的详细误差:\n")
            f.write("-" * 30 + "\n")
            for img_name, error in zip(valid_images, errors):
                f.write(f"{img_name}: {error:.4f} 像素\n")
        
        print(f"\n误差报告已保存到: {error_report_file}")
        print(f"标定数据目录: {images_path}")
        
        # 询问是否绘制误差图表
        try:
            plot_choice = input("\n是否绘制误差图表？(y/n): ").lower().strip()
            if plot_choice == 'y':
                # 将图表保存到标定图片目录
                plot_save_path1 = os.path.join(images_path, "reprojection_errors_bar.png")
                plot_save_path2 = os.path.join(images_path, "reprojection_errors_hist.png")
                plot_reprojection_errors(errors, valid_images, save_path1=plot_save_path1, save_path2=plot_save_path2)
        except:
            pass  # 如果无法获取用户输入，跳过绘图
    
    print("\n标定完成！")

