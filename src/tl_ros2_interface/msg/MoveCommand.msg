# 运动指令
# 用途：话题载荷（/tl_driver/moveJ、/tl_driver/moveL 订阅）；同时作为 srv 嵌套参数（JobInsertMove.cmd、QueueMotionMoveJ.cmd）。
# 单位遵循 REP 103：关节角 rad、笛卡尔位置 m、姿态 rad；驱动在边界换算到控制器单位。
# target_pos_value 为 14 维数组（与 SDK tl::MoveCmd::targetPosValue 一致：前 7 位本体、后 7 位外部轴）：
#   coord = 0（关节）：[0..6] 为目标关节角（rad），按轴数填写、其余置 0；
#   coord = 1/2/3（直角/工具/用户）：[0]X(m) [1]Y(m) [2]Z(m) [3]RX(rad) [4]RY(rad) [5]RZ(rad) [6]冗余臂角(rad)；
#   [7..13] 为外部轴点位，单位随外部轴类型，按 SDK 原值透传，几轴填几位、其余置 0。
float64[] target_pos_value   # 14 维目标点位：布局与单位见文件头
string target_pos_name       # 内置点位名（如 "GP0001"，target_pos_type=5/6 时使用）
int32 target_pos_type        # 点位类型 PosType：0=自定义数组 1=P 2=E 3=RP 4=AP 5=系统内置GP 6=含外部轴的GE
int32 coord                  # 坐标系：0=关节（rad）1=直角 2=工具 3=用户（1/2/3 时位置 m、姿态 rad）
float64 velocity             # 速度：MOVJ 为 %（0<v≤100）；MOVL 为 m/s（0<v≤1）
float64 velocity_sync        # 外部轴同步速度：与 velocity 同口径（MOVJ %；MOVL m/s）
float64 acc                  # 加速度百分比，范围 (0,100]
float64 dec                  # 减速度百分比，范围 (0,100]
int32 pl                     # 平滑过渡等级：0=精确到达，范围 [0,5]
int32 time                   # 提前执行时间（SDK 头未标注单位）
int32 tool_num               # 工具坐标系编号
int32 user_num               # 用户坐标系编号
int32 posidtype              # 变量类型：0=一级(P/GP) 1=二级(P[I001]/GP[I001])
int32 configuration          # 形态
int32 spin                   # MOVCA 指令使用：0=姿态不变 1=六轴不转 2=六轴旋转
bool para_sync               # 外部轴是否同步
