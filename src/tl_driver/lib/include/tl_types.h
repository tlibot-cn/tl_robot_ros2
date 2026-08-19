#pragma once
#include <array>
#include <string>
#include <vector>

// ==================== 跨平台导出宏 ====================
// Windows 下 __declspec(dllexport) 仅导出标记的接口符号，
// 与 Linux version-script（tl_exports.map）收敛导出集行为对齐；
// Linux/macOS 下宏为空，不影响符号表。
// @attention 仅用于对外接口声明（namespace tl 内函数 / 需导出的类），
//            内部实现细节不得标记，避免符号泄漏。
#if defined(_WIN32) || defined(WIN32)
#define TL_API __declspec(dllexport)
#else
#define TL_API
#endif

// 接口废弃标记：调用处产生编译警告（C++14 标准属性 [[deprecated]]，
// MSVC VS2015+ / GCC / Clang 均支持，含消息形式）
// 用法: TL_API TL_DEPRECATED("use new_api instead") Result old_api(...);
// 配套在 Doxygen 注释中写 @deprecated 说明（随存根生成进入 pyi docstring）。
#define TL_DEPRECATED(msg) [[deprecated(msg)]]

namespace tl
{

using SOCKETFD = int;

enum Result
{
  TIMEOUT = -6,
  EXCEPTION = -5,
  OPERATION_NOT_ALLOWED = -4,
  PARAM_ERR = -3,
  DISCONNECT = -2,
  RECEIVE_FAILED = -1,
  SUCCESS = 0
};

enum class PosType
{
  data = 0, // 自定义数组
  PType = 1,
  E_TYPE = 2,
  RP_TYPE = 3,
  AP_TYPE = 4,
  GPType = 5, // 系统内置的全局点位 GP001 只含机器人本体的点位
  GEType = 6  // 系统内置的全局点位 GE001 含外部轴的机器人点位
};

// 坐标系
enum class Coord
{
  JOINT = 0, // 关节坐标系
  BASE = 1,  // 基坐标系（直角坐标系）
  TOOL = 2,  // 工具坐标系
  USER = 3   // 用户坐标系
};

// 机器人类型（对应 get_robot_type 返回值）
// VERSION_DEV 枚举说明（5 起始，旧版 1–14 已废弃）：
enum class RobotType
{
  NONE                   = 5,   // 未使用
  SIX_AXIS_SERIAL        = 6,   // 六轴串联多关节
  FOUR_AXIS_SCARA        = 7,   // 四轴 SCARA
  FOUR_AXIS_PALLETIZING  = 8,   // 四轴码垛
  FOUR_AXIS_SERIAL       = 9,   // 四轴机器人
  SINGLE_AXIS            = 10,  // 一轴机器人
  FIVE_AXIS_SERIAL       = 11,  // 五轴机器人
  SIX_AXIS_COLLABORATIVE = 12,  // 六轴协作
  TWO_AXIS_SCARA         = 13,  // 二轴 SCARA
  THREE_AXIS_SCARA       = 14,  // 三轴 SCARA
  THREE_AXIS_CARTESIAN   = 15,  // 三轴直角机器人
  THREE_AXIS_SPECIAL     = 16,  // 三轴直角异形一机器人
  SEVEN_AXIS_SERIAL      = 17,  // 七轴串联多关节机器人
  SCARA_SPECIAL_1        = 18,  // 四轴 SCARA 异型一机器人
  FOUR_AXIS_PALLETIZING_LEAD = 19, // 四轴码垛丝杆机器人
  SIX_AXIS_SPRAY         = 20,  // 六轴喷涂机器人
  FOUR_AXIS_POLAR        = 21,  // 四轴极坐标异形机器人
  SIX_AXIS_SPECIAL_2     = 22,  // 六轴异型二
  DELTA                  = 23,  // delta（四轴并联机器人）
  WINE_TANK              = 24,  // 酒槽机型
  FOUR_AXIS_CARTESIAN_1  = 25,  // 四轴直角异型一机器人
  FIVE_AXIS_HYBRID       = 26,  // 五轴混动机器人
  FOUR_AXIS_SCARA_2      = 27,  // 四轴 SCARA 异型 2
  SIX_AXIS_SPECIAL_3     = 28,  // 六轴异型三
  BX_SCARA_SPECIAL       = 29,  // 宝信:三轴 SCARA 异型 1
  DELTA_2D               = 30,  // delta 2D 并联机器人模型
  THREE_AXIS_SERIAL_1    = 31,  // 三轴串联异形一
  FIVE_AXIS_COLLAB       = 32,  // 五轴协作机器人
  FOUR_AXIS_SCARA_3      = 33,  // 四轴 SCARA 异型三机器人
  CBBARA                 = 34,  // 六轴串联-CBBARA
  FOUR_AXIS_ROTARY_COL   = 35,  // 高椅立柱旋转四轴
  STEWART                = 36,  // 六自由度上平台 Stewart 并联机器人
  FOUR_AXIS_YZCC         = 37,  // 四轴 YZCC 机型
  SIX_AXIS_ZCCABC        = 38,  // 六轴 ZCCABC 机型
  GANTRY_WELD            = 39,  // 龙门焊接机型
};

// 移动指令参数  单独的Movj只能使用数值   作业文件运行模式只支持使用变量
struct MoveCmd
{
  PosType targetPosType{PosType::data};
  std::vector<double>
      targetPosValue; // 如果posType=PosType::data 为自定义数组,需要设置该向量值,前7位为本体值，后7位为外部轴
  std::string targetPosName{""}; // 如果posType为内置点位,需要设置该值,如 posType=PosType::GPType;posName=“GP0001”;
  Coord coord{Coord::JOINT};  // 坐标系（枚举）
  double velocity{50};
  double velocitySync{0};
  double acc{50};
  double dec{50};
  int pl{0};
  int time{0}; // 提前执行时间
  int toolNum{0};
  int userNum{0};
  int posidtype{0}; // 0:P GP 一级变量类型; 1:P[I001] GP[I001] 二级变量类型  需要使用二级类型时,实例如: targetPosType =
                    // PosType::GPType; posName=“GP￥I001”; posidtype=1; 插入到作业文件中的变量即为GP[I001]
  int configuration{0}; // 形态
  int spin{0};          // MOVCA指令使用 0姿态不变 1六轴不转 2六轴旋转
  bool parasync{false}; // 外部轴是否同步
  MoveCmd() : targetPosValue(14) {};
};

struct ToolParam
{
  double X{0};                   // X轴偏移，单位：mm
  double Y{0};                   // Y轴偏移，单位：mm
  double Z{0};                   // Z轴偏移，单位：mm
  double A{0};                   // 绕A轴旋转，单位：度
  double B{0};                   // 绕B轴旋转，单位：度
  double C{0};                   // 绕C轴旋转，单位：度
  double payloadMass{0};         // 负载质量，单位：kg
  double payloadInertia{0};      // 负载惯量
  double payloadMassCenter_X{0}; // 负载质心X，单位：mm
  double payloadMassCenter_Y{0}; // 负载质心Y，单位：mm
  double payloadMassCenter_Z{0}; // 负载质心Z，单位：mm
  std::string note;              // 注释
  double maxToolRangeX{0};       // 工具干涉区最大X，单位：mm
  double minToolRangeX{0};       // 工具干涉区最小X，单位：mm
  double maxToolRangeY{0};       // 工具干涉区最大Y，单位：mm
  double minToolRangeY{0};       // 工具干涉区最小Y，单位：mm
  double maxToolRangeZ{0};       // 工具干涉区最大Z，单位：mm
  double minToolRangeZ{0};       // 工具干涉区最小Z，单位：mm
  int toolHandMethod{0};         // 工具手方式 0:控制点 1:立方体
};

struct RobotDHParam
{
  double alpha[6]{0};          // 1-6轴DH参数alpha，单位：deg
  double a[6]{0};              // 1-6轴DH参数a，单位：mm
  double theta[6]{0};          // 1-6轴DH参数theta，单位：deg
  double d[6]{0};              // 1-6轴DH参数d，单位：mm
  int eulerAngle{0};           // 欧拉角模式
  double mountingAngle{0};     // 机器人安装角度，单位：deg
};

struct RobotState
{
  int channel = 1;               // 查询的通道，最多9个通道
  bool stop = false;             // 为true时停止持续发送
  int mode = 0;                  // 0-查询只回复一次  1-查询持续回复
  int interval = 10;             // 仅mode = 1时有效，回复时间范围 [10,60000] ms
  bool ioState = false;          // 查询IO
  int position = -1;             // 0-关节坐标  1-基坐标
  bool dataildmotionpos = false; // 机械臂的运动点位
  bool programRunStatus = false; // 运行状态
  bool servoStatus = false;      // 伺服状态
  bool operationMode = false;    // 操作模式，
  bool globalSpeed = false;      // 全局速度，
  bool syncPosition = false;     // 外部轴坐标
  int posSum = 1;                // 当查询机械臂运动点位时，posNum为每帧数据回复的点位数目
  std::vector<std::string>
      ioPort; // IO端口，可查询的最大数量不可大于IO实际个数 例子:[ “DI1”, “DI16”, “DO1”, “DO3”, “DO17”]
  std::vector<std::string>
      optional; // 查询运动点位返回的坐标类型  "ACS"-关节参数 "MCS"-基坐标参数 "time"-时间戳 "reset"-重置点位记录
};

struct CollisionPara
{
  std::vector<double>
      collisionDetection_run; ///< 数组，碰撞检测阈值（指令），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_run≤10000
  std::vector<double>
      collisionDetection_teach; ///< 数组，碰撞检测阈值（点动），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_teach≤10000
  double position_delay_time_ms_value{0.0}; ///<//指令位置响应时间，参数范围：0<position_delay_time_ms_value≤99
  double error_enable_time_ms_value{0.0};   ///< 误差允许时间，参数范围：0≤error_enable_time_ms_value≤99
  unsigned int axisum{6};                   ///< 机器人轴数，默认为六轴机器人
};

// 碰撞安全参数（对应厂商 CollisionSafeParam，24.03+ 固件）
struct CollisionSafeParam
{
  std::vector<double> safe_coeff;            // 碰撞检测系数，第几位为第几轴
  std::vector<double> servo_execution_delay; // 伺服执行响应时间，第几位为第几轴，单位 ms
};

// 关节参数（对应厂商 RobotJointParam）
struct RobotJointParam
{
  double angleToDistance{0};        // 关节角度转距离转换比
  int angleToDistanceUnit{0};       // 转换比单位枚举值
  int axisDirection{1};             // 关节轴正方向
  std::string encodeResolutionUnit; // 编码器分辨率单位文本
  int encoderResolution{0};         // 编码器分辨率数值
  double maxAcc{0};                 // 最大加速度
  double maxAccJerk{0};             // 最大加加速度
  double maxDec{0};                 // 最大减速度
  double maxDecJerk{0};             // 最大减加速度
  double maxPos{0};                 // 正向软限位
  double maxRotSpeed{0};            // 最大转速
  double minPos{0};                 // 负向软限位
  int motorDirection{1};            // 电机方向
  double ratedRotSpeed{0};          // 电机额定转速
  double reduceRatio{0};            // 减速比
  bool reduceRatioEnable{false};    // 是否启用减速比
  double reverseClearance{0};       // 反向间隙
};

// 笛卡尔参数（对应厂商 CartesianParam）
struct CartesianParam
{
  double maxSpeed{1000};      // 最大线速度 (mm/s)
  double maxAcc{3};           // 最大线加速度 (mm/s^2)
  double maxDec{-3};          // 最大线减速度 (mm/s^2)
  double maxAttitudeVel{500}; // 最大姿态角速度 (deg/s)
  int speedLimitMode{0};      // 速度限制方式 0:位姿，1:位置
  double posResolution{0.01}; // 绝对位置分辨率，范围[0.0001, 0.1]
  double maxJerk{10000};      // 最大线加加速度 (mm/s^3)
  int interpType{0};          // 插补方式 0:S型，1:梯形
  double minAccTime{100};     // 最小加速时间 (ms)
  double minDecTime{100};     // 最小减速时间 (ms)
};

struct ToolCoordinateRange
{
  double max_range_x;   ///< X轴最大范围
  double min_range_x;   ///< X轴最小范围
  double max_range_y;   ///< Y轴最大范围
  double min_range_y;   ///< Y轴最小范围
  double max_range_z;   ///< Z轴最大范围
  double min_range_z;   ///< Z轴最小范围
  int tool_hand_method; ///< 工具握持方式
};

struct SixDimensionalForceCommunicationParams
{
  bool sensorDragEnable = true;          // 传感器拖拽使能// 默认值通常为 true
  int originDataInitialPara = 0;         // 原始数据初始参数
  int sensorCommunicationType = 0;       // 通讯类型 (0: EtherCAT, 1: Modbus RTU, 2: Modbus TCP)
  bool startupAutoConnectSensor = false; // 启动自动连接传感器
  int YDirection = 1;                    // Y 方向
  int ZDirection = 1;                    // Z 方向
  int etherCat_mapNum = 0;               // EtherCAT 参数
  // Modbus RTU 参数
  int modbus_rtu_slaveID = 1;
  int modbus_rtu_port = 1;
  int modbus_rtu_baudRate = 115200;
  int modbus_rtu_addressType = 0;
  int modbus_rtu_firstAddress = 1;
  int modbus_rtu_addressNum = 1;
  int modbus_rtu_endian = 1;
  std::string modbus_rtu_checkBit = "N";
  int modbus_rtu_dataBit = 8;
  int modbus_rtu_stopBit = 1;
  // Modbus TCP 参数
  std::string modbus_tcp_IP = "192.168.1.14";
  int modbus_tcp_port = 503;
  int modbus_tcp_addressType = 0;
  int modbus_tcp_firstAddress = 1;
  int modbus_tcp_addressNum = 1;
  int modbus_tcp_endian = 1;
};

struct Sensor6DData
{
  bool sensorConnected = false;
  // 传感器原始数据 (Fx, Fy, Fz, Mx, My, Mz)
  double fxData = 0.0;
  double fyData = 0.0;
  double fzData = 0.0;
  double mxData = 0.0;
  double myData = 0.0;
  double mzData = 0.0;
  // 去皮数据 (Fx, Fy, Fz, Mx, My, Mz)
  double fxDataSubBase = 0.0;
  double fyDataSubBase = 0.0;
  double fzDataSubBase = 0.0;
  double mxDataSubBase = 0.0;
  double myDataSubBase = 0.0;
  double mzDataSubBase = 0.0;
  std::vector<double> torqueConvertData; // 扭矩转换数据
  Sensor6DData() : torqueConvertData(6, 0.0) {}
};

struct SensorBaseParam
{
  double sensorMass = 0.0;        // 质量
  double sensorMassCenterX = 0.0; // X方向质心
  double sensorMassCenterY = 0.0; // Y方向质心
  double sensorMassCenterZ = 0.0; // Z方向质心
  bool saveZero = false;          // 是否已标零
  SensorBaseParam(double mass = 0.0, double cx = 0.0, double cy = 0.0, double cz = 0.0, bool zeroed = false)
      : sensorMass(mass), sensorMassCenterX(cx), sensorMassCenterY(cy), sensorMassCenterZ(cz), saveZero(zeroed)
  {
  }
};

struct RemoteProgram
{
  int port;  // 远程程序端口绑定
  int value; // 使用远程IO功能时有效参数(0/1),使用远程状态提示功能时有效参数(0/1/2)
};

struct RemoteControl
{
  int clearStashPort; // 清除断电保持数据绑定端口
  int faultResetPort; // 清除报警端口
  int pausePort;      // 暂停端口
  int startPort;      // 启动端口
  int stopPort;       // 停止端口

  int clearStashValue; // 清除断电保持数据端口的有效参数(0/1),与clearStashPort相对应
  int faultResetValue; // 清除报警端口的有效参数(0/1),与faultResetPort相对应
  int pauseValue;      // 暂停端口的有效参数(0/1),与pausePort相对应
  int startValue;      // 启动端口的有效参数(0/1),与startPort相对应
  int stopValue;       // 停止端口的有效参数(0/1),与stopPort相对应

  std::vector<RemoteProgram> program; // 远程程序端口设置,详见 RemoteProgram
};

struct IOtype
{
  int num;                       // IO板数量
  std::vector<std::string> type; // IO型号
  std::vector<std::vector<int>>
      io_port_sum; // IO端口数量, 一维数组: [数字输入端口数量,数字输出端口数量,模拟输入端口数量,模拟输出端口数量]
};

// TCP参数
struct ModbusTCPParameter
{
  std::string IP{"192.168.1.13"};
  int port{503};
  int endian_type{1}; // 浮点数字节序类型，默认 1
};

// RTU参数
struct ModbusRTUParameter
{
  int slaveId;                  // 从站号
  int port{1};                  // 端口
  int baudrate{115200};         // 波特率
  std::string checkBit{"None"}; // 奇偶校验位"None","Even","Odd"
  int dataBit{8};               // 数据位,5,6,7,8
  int stopBit{1};               // 停止位,1,2
};

struct ModbusMasterParameter
{
  std::string type{"TCP"};  // 主站类型 "TCP","RTU"
  bool startAddress{false}; // false:起始地址为1；true:起始地址为0
  // TCP参数
  ModbusTCPParameter TCP;
  // RTU参数
  ModbusRTUParameter RTU;
};

// ==================== VERSION_DEV 新增结构体 ====================

// 用户坐标参数（对应厂商 UserCoordParam）
struct UserCoordParam
{
  int location_type{0};           // 用户坐标类型 0:静态 1:联动 2:动态
  std::string name;               // 用户坐标名称
  double position[6]{0};          // 用户坐标参数[X,Y,Z,A,B,C]
  int related_type{0};            // 联动对象类型 0:机器人 1:外部轴
  int related_num{0};             // 联动对象编号
  int related_coord_type{0};      // 联动坐标类型 0:直角 1:工具 2:用户
  int related_coord_num{0};       // 联动坐标编号
};

// 伺服运动参数（对应厂商 ServoMovePara）
struct ServoMovePara
{
  bool clearBuffer{false};              // 是否清除之前未开始插补的点位
  int targetMode{0};                    // 0:独立点 1:连续轨迹
  int sendMode{0};                      // 0:一次传输全部 1:一次传输部分
  int runMode{0};                       // 0:接收完再运动 1:边接边运动
  int sum{0};                           // 总传输次数
  int count{0};                         // 当前是第几次
  int coord{0};                         // 0:关节 1:直角
  int extMove{0};                       // 外部轴运动标志
  int size{0};                          // 本次传输点位数
  std::vector<std::vector<double>> pos;      // 点位，二维[本次点数][7维]
  std::vector<std::vector<double>> axisvel;  // 轴速度，二维[本次点数][7维]
  std::vector<std::vector<double>> axisacc;  // 轴加速度，二维[本次点数][7维]
  std::vector<double> timeStamp;             // 到达时间戳 (ms)
};

// 伺服点位运动参数（对应厂商 ServoPointMovePara）
struct ServoPointMovePara
{
  bool end{false};                          // 是否清除之前未开始的点位
  int sum{0};                               // 总帧数
  int count{0};                             // 当前帧
  std::vector<std::vector<double>> pos;     // 点位，二维[帧数][12维: 本体7+外部轴5]
};

// 机器人速度参数（对应厂商 RobotSpeedParam）
struct RobotSpeedParam
{
  bool handwheel_control{false};          // 是否为手轮模式
  double micro_dot_speed_ACS{0};          // 步进距离(ACS)
  double micro_dot_speed_MCS{0};          // 手轮距离(MCS)
  std::string name;                       // 当前速度名称
  int segment{1};                         // 段号
  int speed{0};                           // 当前速度档位值
  std::vector<std::string> speed_segment_name; // 速度档名称列表
  std::vector<int> speed_segment;         // 速度档数值列表
  int type{0};                            // 速度类型 0:档位 1:ACS步进 2:MCS步进
};

// 机器人速度段参数（对应厂商 RobotSpeedSegmentParam）
struct RobotSpeedSegmentParam
{
  std::vector<std::string> speed_segment_name; // 速度段名称列表，最大20
  std::vector<int> speed_segment;              // 速度段速度列表
};

// 拖动示教参数（对应厂商 DragParam）
struct DragParam
{
  int drag_mode{0};              // 拖动模式 0:自由 1:位置 2:姿态
  double start_threshold_F{0};   // 启动阈值 F
  double start_threshold_M{0};   // 启动阈值 M
  double dragInPosMaxVel{0};     // 笛卡尔空间线速度限制
  double dragInPosMaxAngleVel{0}; // 关节空间速度限制
  double drag_change_rate[6]{0}; // 变化率阈值[X,Y,Z,A,B,C]
  double drag_damper[6]{0};      // 阻尼系数[X,Y,Z,A,B,C]
  double drag_mass[6]{0};        // 质量系数[X,Y,Z,A,B,C]
};

// VFD 运行参数（对应厂商 VFDRunParam）
struct VFDRunParam
{
  int dir{0};               // 方向
  int vel{0};               // 速度
  int acceleration{0};      // 加速度
  int deceleration{0};      // 减速度
  bool independent_axis{true}; // 是否为主轴
};

// VFD 状态（对应厂商 VFDState）
struct VFDState
{
  int motor_vel{0};       // 电机转速
  int motor_current{0};   // 电机电流
  double spindle_angle{0}; // 主轴角度
};

// 独立轴参数（对应厂商 IndependentAxisParam）
struct IndependentAxisParam
{
  int axis_num{0};                             // 轴编号
  double angular_distance_conversion_ratio{0}; // 角度距离转换比
  bool is_track{false};                        // 是否是地轨
  double encoder_bits{0};                      // 编码器位数
  double inverse_limit{0};                     // 反限位
  double max_positive_speed{0};                // 最大正转速（倍率）
  double max_reverse_speed{0};                 // 最大反转速（倍率）
  int motor_dir{1};                            // 电机方向 1:正 -1:负
  double positive_limit{0};                    // 正限位
  double rated_positive_speed{0};              // 额定正转速 (rpm)
  double rated_reverse_speed{0};               // 额定反转速 (rpm)
  double reduction_ratio{0};                   // 减速比
  double max_acc{0};                           // 最大加速度（倍率）
  double max_dec{0};                           // 最大减速度（倍率）
  int select_bind_servo{0};                    // 选择绑定的伺服编号
};

// 独立轴运行（对应厂商 IndependentAxisRun）
struct IndependentAxisRun
{
  int axis_num{0};   // 轴编号
  int vel{0};        // 速度
  int dir{1};        // 方向 1:正 -1:负
  double acc{0};     // 加速度
  double dec{0};     // 减速度
};

// 传感器负载参数（对应厂商 PayloadParamBySensor）
struct PayloadParamBySensor
{
  double payloadMass{0};         // 负载质量 (kg)
  double payloadMassCenterX{0};  // 负载质心X (mm)
  double payloadMassCenterY{0};  // 负载质心Y (mm)
  double payloadMassCenterZ{0};  // 负载质心Z (mm)
};

// 逆运动学参数（对应厂商 InverseKinParameter）
struct InverseKinParameter
{
  int configuration{0};  // 1:左手系 2:右手系 0:自适应
  int toolCoord{0};
  int userCoord{0};
  int standbyThree{0};
  int standbyFour{0};
  double vel_limit_perc{1.0};
  double acc_limit_perc{1.0};
  double dec_limit_perc{1.0};
};

} // namespace tl
