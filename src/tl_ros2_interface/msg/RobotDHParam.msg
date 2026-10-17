# 机器人DH参数（新SDK: 标准DH参数）
# 用途：仅作 srv 嵌套参数（GetDHParam.param、SetDHParam.param），不发布话题。
# 对应 SDK tl::RobotDHParam：alpha[6]/a[6]/theta[6]/d[6]/eulerAngle/mountingAngle
# 单位遵循 REP 103：角度 rad、长度 m；驱动在边界换算到 SDK 的 deg / mm。
float64[] alpha           # 1-6轴DH参数alpha，数组长度 6，单位：rad
float64[] a               # 1-6轴DH参数a，数组长度 6，单位：m
float64[] theta           # 1-6轴DH参数theta，数组长度 6，单位：rad
float64[] d               # 1-6轴DH参数d，数组长度 6，单位：m
int32 euler_angle         # 欧拉角模式
float64 mounting_angle    # 机器人安装角度，单位：rad
