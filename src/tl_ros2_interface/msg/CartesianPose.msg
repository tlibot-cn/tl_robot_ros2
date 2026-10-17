# 直角坐标系位姿
# 用途：话题载荷（/tcp_pose 发布）；同时作为 srv 嵌套参数（SetUserCoord.pos）。
# 单位遵循 REP 103：位置 m、姿态 rad。
std_msgs/Header header            # 时间戳与参考坐标系（/tcp_pose 为 base_link）
geometry_msgs/Point position      # 位置，单位：m
geometry_msgs/Vector3 rpy         # 姿态欧拉角，XYZ 内旋，单位：rad
float64 arm_angle                 # 冗余臂角，单位：rad（6 轴为 0；SetUserCoord 未使用）
