# 直角坐标系位姿
# 单位约定：position 为 mm（本仓库笛卡尔位置采用 mm，非 ROS 惯例的 m）；rpy / arm_angle 为 rad
std_msgs/Header header
geometry_msgs/Point position   # 单位: mm（x/y/z 均为 mm，SDK coord=1 原值透传）
geometry_msgs/Vector3 rpy      # 单位: rad（绕 X/Y/Z 轴姿态角）
float64 arm_angle              # 单位: rad（冗余臂角）