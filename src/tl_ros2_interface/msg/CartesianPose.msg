# 直角坐标系位姿
# 用途：话题载荷（/tcp_pose 发布）；同时作为 srv 嵌套参数（SetUserCoord.pos）。
# 单位注意：同一类型在两个使用场景口径不同（本仓库现状）：
#   /tcp_pose 发布：position 为 m、rpy 为 rad、arm_angle 为 rad；
#   SetUserCoord.pos 输入：position 为 mm、rpy 为 rad（控制器 mm 体系），arm_angle 未使用。
std_msgs/Header header            # 时间戳与参考坐标系（/tcp_pose 为 base_link）
geometry_msgs/Point position      # 位置：/tcp_pose 为 m；SetUserCoord 为 mm（见文件头）
geometry_msgs/Vector3 rpy         # 姿态欧拉角，XYZ 内旋，单位 rad
float64 arm_angle                 # 冗余臂角，单位 rad（6 轴为 0；SetUserCoord 当前未使用）
