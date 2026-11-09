# 运动指令
# 用途：Move action goal 载荷；同时作为 srv 嵌套参数（QueueMotionMoveJ.cmd）。
# 单位遵循 REP 103：关节角 rad、笛卡尔位置 m、姿态 rad；驱动在边界换算到控制器单位。
# target_pos_value 定长 14 维（与 SDK tl::MoveCmd::targetPosValue 一致：前 7 位本体、后 7 位外部轴）：
#   coord = 0（关节）：[0..6] 为目标关节角（rad），按轴数填写、其余置 0；
#   coord = 1/2/3（直角/工具/用户）：[0]X(m) [1]Y(m) [2]Z(m) [3]RX(rad) [4]RY(rad) [5]RZ(rad) [6]冗余臂角(rad)；
#   [7..13] 为外部轴点位，单位随外部轴类型，按 SDK 原值透传，几轴填几位、其余置 0。
float64[14] target_pos_value  # 目标点位：布局与单位见文件头
int32 coord                  # 坐标系：0=关节（rad）1=直角 2=工具 3=用户（1/2/3 时位置 m、姿态 rad）
float64 velocity             # 速度：MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）
float64 velocity_sync        # 外部轴同步速度：与 velocity 同口径（MOVJ %；MOVL m/s）
float64 acc                  # 加速度百分比，范围 (0,100]
float64 dec                  # 减速度百分比，范围 (0,100]
int32 pl                     # 平滑过渡等级：0=精确到达，范围 [0,5]
int32 tool_num               # 工具坐标系编号
int32 user_num               # 用户坐标系编号
bool para_sync               # 外部轴是否同步
