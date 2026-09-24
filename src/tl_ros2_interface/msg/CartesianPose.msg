# 直角坐标系位姿
# 用途：话题载荷（/tcp_pose 发布）；同时作为 srv 嵌套参数（SetUserCoord.pos）。
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 rpy
float64 arm_angle 