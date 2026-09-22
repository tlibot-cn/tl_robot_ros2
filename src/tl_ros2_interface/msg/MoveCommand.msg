# 运动指令
# 单位约定（本仓库）：笛卡尔位置一律为 mm —— 由 SDK coord=1 的原始值透传，非 ROS 惯例的 m；
# 姿态为 rad；关节角为 度。各字段单位见下方行尾注释。
#
# target_pos_value 为 14 维数组，两种布局由 coord 决定：
#   coord = 0（关节运动）：整组为目标关节角，单位 度。
#   coord = 1/2/3（直角/工具/用户运动）：前 7 位为本体位姿
#     [0]X(mm) [1]Y(mm) [2]Z(mm) [3]RX(rad) [4]RY(rad) [5]RZ(rad) [6]冗余臂角(rad)；
#     后 7 位为外部轴点位（单位随外部轴类型，本仓库未做换算，按 SDK 原值透传），几轴填几位，其余置 0。
#   与 lib/include/cpp/parameter/tl_define.h 中 targetPosValue 的注释（前 7 位本体值、后 7 位外部轴）一致。
float64[] target_pos_value   # 14 维目标点位：布局与单位见文件头说明
string target_pos_name       # 内置点位名（target_pos_type 为内置点位时使用），如 "GP0001"
int32 target_pos_type        # 点位类型 PosType：0=自定义数组 1=P 5=系统内置GP点位 6=含外部轴的GE点位
int32 coord                  # 坐标系：0=关节（点位为关节角，单位: 度）1=直角 2=工具 3=用户（1/2/3 时位置 mm、姿态 rad）
float64 velocity             # 速度：关节运动(MOVJ) 单位: %（0<velocity≤100）；直角运动(MOVL) 单位: mm/s（0<velocity≤1000）
float64 velocity_sync        # 外部轴同步速度：与 velocity 同口径（关节运动(MOVJ) 单位: %；直角运动(MOVL) 单位: mm/s）；本仓库不对外部轴做换算，按 SDK 原值透传
float64 acc                  # 加速度 单位: %
float64 dec                  # 减速度 单位: %
int32 pl                     # 平滑过渡(PL) 单位: %
int32 time                   # 提前执行时间（SDK 头未标注单位）
int32 tool_num               # 工具手坐标序号
int32 user_num               # 用户坐标序号
int32 posidtype              # 变量类型：0=一级(P/GP) 1=二级(P[I001]/GP[I001])
int32 configuration          # 形态
int32 spin                   # MOVCA 指令使用：0=姿态不变 1=六轴不转 2=六轴旋转
bool para_sync               # 外部轴是否同步