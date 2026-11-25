#!/usr/bin/env python3
"""
夹爪抓取检测测试 - 对比空载和夹物体的**开口度**

用法:
    python3 test_gripper_feedback.py
    1. 空载闭合 → 看开口度
    2. 放物体 → 夹物体闭合 → 看开口度
    3. 对比：开口度差异 = 是否抓到物体
"""

import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from zhua import EG2Gripper


def close_and_read(gripper, label, opening=0):
    """闭合夹爪并读取开口度"""
    print(f'\n▶ {label}')
    gripper.set_opening(opening)
    time.sleep(1.5)
    pos = gripper.read_position()
    print(f'  实际开口度: {pos}  (0=完全闭合)')
    return pos


def main():
    PORT = '/dev/ttyUSB0'
    FORCE = 500

    gripper = EG2Gripper(port=PORT, slave_id=1, baudrate=115200)
    gripper.set_speed(500)
    gripper.set_force(FORCE)

    try:
        print('=' * 50)
        print(f'  夹爪抓取检测测试 (夹持力={FORCE}g)')
        print('=' * 50)

        # 1. 张开
        print('\n▷ 张开夹爪...')
        gripper.open_gripper(opening=1000)
        time.sleep(1.0)

        # 2. 空载闭合
        empty_pos = close_and_read(gripper, '空载闭合', opening=0)

        # 3. 张开，等放物体
        print('\n▷ 张开夹爪，请放入物体...')
        gripper.open_gripper(opening=1000)
        input('   放好物体后按 Enter 继续...')

        # 4. 夹物体闭合
        load_pos = close_and_read(gripper, '夹物体闭合', opening=0)

        # 5. 张开收尾
        print('\n▷ 张开夹爪...')
        gripper.open_gripper(opening=1000)
        time.sleep(0.5)

        # 结果
        print('\n' + '=' * 50)
        print('  结果对比')
        print('=' * 50)
        print(f'  空载开口度:  {empty_pos}')
        print(f'  夹物开口度:  {load_pos}')
        print(f'  开口度差异:  {load_pos - empty_pos}')

        if load_pos > empty_pos + 20:
            print(f'\n  ✅ 可以通过开口度判断抓取！')
            print(f'  建议阈值: 开口度 > {load_pos // 2} 即认为抓到物体')
        else:
            print(f'\n  ⚠ 开口度差异不大，试试调小夹持力')

    except KeyboardInterrupt:
        print('\n中断')
    finally:
        gripper.open_gripper(opening=1000)
        time.sleep(0.5)
        gripper.close()
        print('夹爪已断开')


if __name__ == '__main__':
    main()
