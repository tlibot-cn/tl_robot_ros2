# 机器人DH参数（新SDK: 标准DH参数）
# 对应 SDK tl::RobotDHParam：alpha[6]/a[6]/theta[6]/d[6]/eulerAngle/mountingAngle
float64[] alpha           # 1-6轴DH参数alpha，单位：deg
float64[] a               # 1-6轴DH参数a，单位：mm
float64[] theta           # 1-6轴DH参数theta，单位：deg
float64[] d               # 1-6轴DH参数d，单位：mm
int32 euler_angle         # 欧拉角模式
float64 mounting_angle    # 机器人安装角度，单位：deg

