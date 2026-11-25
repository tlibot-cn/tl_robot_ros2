#!/usr/bin/env python3
"""夹爪控制 - 1张开  2闭合  其他键显示速度/夹持力"""

from zhua import EG2Gripper

g = EG2Gripper(port='/dev/ttyUSB0')

speed = 500      # 速度
force = 500      # 夹持力(g)

print('1-张开  2-闭合  3-显示速度  4-显示力度  q-退出')

while True:
    key = input('\n输入: ').strip()
    if key == 'q':
        break
    elif key == '1':
        g.open_gripper(1000)
        print('张开')
    elif key == '2':
        g.close_gripper(0)
        print('闭合')
    elif key == '3':
        print(f'速度: {speed}')
    elif key == '4':
        print(f'夹持力: {force}g')

g.close()
