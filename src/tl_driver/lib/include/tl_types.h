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
#  define TL_API __declspec(dllexport)
#else
#  define TL_API
#endif

// 接口废弃标记：调用处产生编译警告（C++14 标准属性 [[deprecated]]，
// MSVC VS2015+ / GCC / Clang 均支持，含消息形式）
// 用法: TL_API TL_DEPRECATED("使用 new_api 替代") Result old_api(...);
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
enum class RobotType
{
  SIX_AXIS_SERIAL = 1,             // 六轴串联多关节
  FOUR_AXIS_SCARA = 2,             // 四轴 SCARA
  FOUR_AXIS_PALLETIZING = 3,       // 四轴码垛
  FOUR_AXIS_SERIAL = 4,            // 四轴串联多关节
  SINGLE_AXIS = 5,                 // 单轴
  FIVE_AXIS_SERIAL = 6,            // 五轴串联多关节
  SIX_AXIS_COLLABORATIVE = 7,      // 六轴协作
  TWO_AXIS_SCARA = 8,              // 二轴 SCARA
  THREE_AXIS_SCARA = 9,            // 三轴 SCARA
  THREE_AXIS_CARTESIAN = 10,       // 三轴直角
  THREE_AXIS_SPECIAL = 11,         // 三轴异形
  SEVEN_AXIS_SERIAL = 12,          // 七轴串联多关节
  SCARA_SPECIAL = 13,              // SCARA 异形一
  FOUR_AXIS_PALLETIZING_LEAD = 14, // 四轴码垛丝杆
};

// 移动指令参数  单独的Movj只能使用数值   作业文件运行模式只支持使用变量
struct MoveCmd
{
  PosType targetPosType{PosType::data};
  std::vector<double> targetPosValue; // 如果posType=PosType::data
                                      // 为自定义数组,需要设置该向量值,前7位为本体值，后7位为外部轴
  std::string targetPosName{
      ""}; // 如果posType为内置点位,需要设置该值,如 posType=PosType::GPType;posName=“GP0001”;
  Coord coord{Coord::JOINT}; // 坐标系（枚举）
  double velocity{50};
  double velocitySync{0};
  double acc{50};
  double dec{50};
  int pl{0};
  int time{0}; // 提前执行时间
  int toolNum{0};
  int userNum{0};
  int posidtype{0}; // 0:P GP 一级变量类型; 1:P[I001] GP[I001] 二级变量类型
                    // 需要使用二级类型时,实例如: targetPosType = PosType::GPType;
                    // posName=“GP￥I001”; posidtype=1; 插入到作业文件中的变量即为GP[I001]
  int configuration{0}; // 形态
  int spin{0};          // MOVCA指令使用 0姿态不变 1六轴不转 2六轴旋转
  bool parasync{false}; // 外部轴是否同步
  MoveCmd() : targetPosValue(14){};
};

struct HanYu
{
  double PC = 0;
  double SP[3] = {0}; // X,Y,Z
  double TL[3] = {0}; // X,Y,Z
};

struct Alpha
{
  double alpha1 = 90.0; // 范围[-180°, 180°]
  double alpha2 = 0.0;
  double alpha3 = 90.0;
  double alpha4 = 90.0;
  double alpha5 = -90.0;
  double alpha6 = 0.0;
};

struct ToolParam
{
  double X;                   // X轴偏移方向
  double Y;                   // Y轴偏移方向
  double Z;                   // Z轴偏移方向
  double A;                   // 绕A轴旋转
  double B;                   // 绕B轴旋转
  double C;                   // 绕C轴旋转
  double payloadMass;         // 负载质量
  double payloadInertia;      // 负载惯性
  double payloadMassCenter_X; // 负载质心X
  double payloadMassCenter_Y; // 负载质心Y
  double payloadMassCenter_Z; // 负载质心Z
};

struct RobotDHParam
{
  double L1{0};
  double L2{0};
  double L3{0};
  double L4{0};
  double L5{0};
  double L6{0};
  double L7{0};
  double L8{0};
  double L9{0};
  double L10{0};
  double L11{0};
  double L12{0};
  double L13{0};
  double L14{0};
  double L15{0};
  double L16{0};
  double L17{0};
  double L18{0};
  double L19{0};
  double L20{0};

  double Couple_Coe_1_2;
  double Couple_Coe_2_3;
  double Couple_Coe_3_2;
  double Couple_Coe_3_4;
  double Couple_Coe_4_5;
  double Couple_Coe_4_6;
  double Couple_Coe_5_6;

  double dynamicLimit_max{0};
  double dynamicLimit_min{0};

  double pitch{0};              // 螺距
  double sliding_lead_value{0}; // 滑动电动缸导程,酒槽机型用
  double uplift_lead_value{0};  // 顶升电动缸导程,酒槽机型用
  double spray_distance{0};     // 喷料距离,酒槽机型用

  double threeAxisDirection{0}; // 3轴方向
  double fiveAxisDirection{0};  // 五轴方向

  double twoAxisConversionRatio{0};
  double threeAxisConversionRatio{0};
  double amplificationRatio{0};

  double conversionratio_x{0};
  double conversionratio_y{0};
  double conversionratio_z{0};

  double conversionratio_J1{0}; // 1轴转换比 五轴混动
  double conversionratio_J2{0};
  double conversionratio_J3{0};

  int upsideDown{0};

  HanYu hanyu;
  Alpha alpha;
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
  int posSum = 1; // 当查询机械臂运动点位时，posNum为每帧数据回复的点位数目
  std::vector<std::string> ioPort; // IO端口，可查询的最大数量不可大于IO实际个数 例子:[ “DI1”,
                                   // “DI16”, “DO1”, “DO3”, “DO17”]
  std::vector<std::string> optional; // 查询运动点位返回的坐标类型  "ACS"-关节参数 "MCS"-基坐标参数
                                     // "time"-时间戳 "reset"-重置点位记录
};

struct CollisionPara
{
  std::vector<double>
      collisionDetection_run; ///< 数组，碰撞检测阈值（指令），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_run≤10000
  std::vector<double>
      collisionDetection_teach; ///< 数组，碰撞检测阈值（点动），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_teach≤10000
  double position_delay_time_ms_value{
      0.0}; ///<//指令位置响应时间，参数范围：0<position_delay_time_ms_value≤99
  double error_enable_time_ms_value{
      0.0}; ///< 误差允许时间，参数范围：0≤error_enable_time_ms_value≤99
  unsigned int axisum{6}; ///< 机器人轴数，默认为六轴机器人
};

// 碰撞安全参数（对应协议 CollisionSafeParam，24.03+ 固件）
struct CollisionSafeParam
{
  std::vector<double> safe_coeff;            // 碰撞检测系数，第几位为第几轴
  std::vector<double> servo_execution_delay; // 伺服执行响应时间，第几位为第几轴，单位 ms
};

struct RobotJointParam
{
  double reducRatio{0};      // 减速比
  int encoderResolution{0};  // 编码器位数
  double posSWLimit{0};      // 轴正限位
  double negSWLimit{0};      // 轴反限位
  double ratedRotSpeed{0};   // 电机额定正转速
  double ratedDeRotSpeed{0}; // 电机额定反转速
  double maxRotSpeed{0};     // 电机最大正转速
  double maxDeRotSpeed{0};   // 电机最大反转速
  double ratedVel{0};        // 额定正速度
  double deRatedVel{0};      // 额定反速度
  double maxAcc{0};          // 最大加速度
  double maxDecel{0};        // 最大减速度
  int direction{1};          // 模型方向，1：正向，-1：反向
};

// 拖拽参数（对应协议 DragParam）
struct DragParam
{
  int dragMode{0};                // 拖动模式 0-自由拖动 1-位置拖动 2-姿态拖动
  double startThresholdF{0};      // 启动阈值 F
  double startThresholdM{0};      // 启动阈值 M
  double dragInPosMaxVel{0};      // 笛卡尔空间线速度限制
  double dragInPosMaxAngleVel{0}; // 关节空间速度限制
  std::array<double, 6> dragChangeRate{}; // 变化率阈值 [X,Y,Z,A,B,C]
  std::array<double, 6> dragDamper{};     // 阻尼系数 [X,Y,Z,A,B,C]
  std::array<double, 6> dragMass{};       // 质量系数 [X,Y,Z,A,B,C]
};

// 拖拽力矩参数（对应协议 DragTorqueParam）
struct DragTorqueParam
{
  std::vector<double> deviationCoeff;  // 模型偏差阈值，第几位为第几轴
  double startThresholdF{0};           // 六维力启动阈值 F
  double startThresholdM{0};           // 六维力启动阈值 M
  std::vector<double> frictionOffset;  // 摩擦力补偿系数，第几位为第几轴
  int jointVelLimit{0};                // 关节速度限制，单位 °/s
  std::vector<double> resistanceCoeff; // 超限阻力系数，第几位为第几轴
  std::vector<double> sensorCoeff;     // 关节传感器灵敏系数，第几位为第几轴
  std::vector<double> targetTorqCoeff; // 目标扭矩矫正系数，第几位为第几轴
  int waitCycle{0};                    // 等待周期
};

// 笛卡尔参数（对应协议 CartesianParam）
struct CartesianParam
{
  double maxSpeed{0};       // 最大线速度 (mm/s)
  double maxAcc{0};         // 最大线加速度 (mm/s^2)
  double maxDec{0};         // 最大线减速度 (mm/s^2)
  double maxAttitudeVel{0}; // 最大姿态角速度 (deg/s)
  int speedLimitMode{0};    // 速度限制方式 0:位姿，1:位置
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

// 传感器负载参数（对应协议 PayloadParamBySensor）
struct PayloadParamBySensor
{
  double payloadMass{0};        // 计算出的负载质量
  double payloadMassCenterX{0}; // X方向负载质心
  double payloadMassCenterY{0}; // Y方向负载质心
  double payloadMassCenterZ{0}; // Z方向负载质心
  PayloadParamBySensor(double mass = 0.0, double cx = 0.0, double cy = 0.0, double cz = 0.0)
      : payloadMass(mass), payloadMassCenterX(cx), payloadMassCenterY(cy), payloadMassCenterZ(cz)
  {
  }
};

struct SensorBaseParam
{
  double sensorMass = 0.0;        // 质量
  double sensorMassCenterX = 0.0; // X方向质心
  double sensorMassCenterY = 0.0; // Y方向质心
  double sensorMassCenterZ = 0.0; // Z方向质心
  bool saveZero = false;          // 是否已标零
  SensorBaseParam(double mass = 0.0, double cx = 0.0, double cy = 0.0, double cz = 0.0,
                  bool zeroed = false)
      : sensorMass(mass), sensorMassCenterX(cx), sensorMassCenterY(cy), sensorMassCenterZ(cz),
        saveZero(zeroed)
  {
  }
};

struct RemoteProgram
{
  int port; // 远程程序端口绑定
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

// IO 型号信息（数字/模拟输入输出端口总数，对应控制器 IO 板信息）
struct IOtype
{
  int num{0};                    // IO 板数量
  std::vector<std::string> type; // 各 IO 板型号
  std::vector<std::vector<int>>
      io_port_sum; // 端口总数，一维：[数字输入, 数字输出, 模拟输入, 模拟输出]（每行一块 IO 板）
};

// IO 报警信息设置（数字输入/输出端口报警功能）
struct AlarmdIO
{
  int msgType; // 消息类型 0：普通消息，1：警告消息，2：错误消息（早期版本无此参数，全部设为 0
               // 即可）
  int value;   // IO 有效参数设置(0/1)
  int enable;  // 使能设置(0/1)
  std::string msg; // 消息内容
};

// 远程程序选择参数（用于远程IO程序选择/状态提示）
struct RemoteProgramSetting
{
  std::string job; // 远程程序选择
  int times;       // 远程程序运行次数
};

// IO 安全设置参数（紧急停止/安全光幕端口绑定）
struct SafeIO
{
  int quickStopPort1;     // 紧急停止端口1
  int quickStopPort2;     // 紧急停止端口2
  int quickStopValue1;    // 紧急停止参数1(0/1)
  int quickStopValue2;    // 紧急停止参数2(0/1)
  bool quickStopEnable;   // 紧急停止使能
  bool quickStopShied1;   // 屏蔽紧急停止1
  bool quickStopShied2;   // 屏蔽紧急停止2
  double quickStopTime;   // 快速停止时间，单位 毫秒(ms) 范围 [50,100]
  int quickStopShiedTime; // 屏蔽紧急停止时间，单位 秒(s)

  int screenPort1;   // 安全光幕端口1
  int screenPort2;   // 安全光幕端口2
  int screenValue1;  // 安全光幕参数1(0/1)
  int screenValue2;  // 安全光幕参数2(0/1)
  bool screenEnable; // 安全光幕使能
};

// 逻辑类型（条件指令比较运算，对应控制器条件语法）
enum class LogicType
{
  EQUAL_TO = 1,  // ==
  LESS,          // <
  GREATER,       // >
  LESS_EQUAL,    // <=
  GREATER_EQUAL, // >=
  NOT_EQUAL_TO   // !=
};

// 条件参数组（数值型/变量型二选一）
struct ParaGroup
{
  double data{0.0}; // 数值型赋值
  int secondvalue{0}; // 变量型是否存在二级变量，1存在 0不存在（如 "DOUT[I001]" 填1）
  int value{0};       // 0 数值型 1 变量型
  std::string varname{""}; // 变量型变量名（如 "I001"、"DOUT[I001]"）
};

// 条件指令参数（until/while/if 等逻辑控制指令）
struct Condition
{
  std::string desValue{"0"}; // 目标值描述
  std::string key{"0"};      // 变量名/关键字
  LogicType logicType{LogicType::EQUAL_TO};
  ParaGroup paraGroupOne;
  ParaGroup paraGroupTwo;
};

// IO 输出指令参数（作业文件插入 IO 输出指令用）
struct IOCommandParams
{
  std::string groupType;   // 输出路数选择：OT# 1路 / OGH# 4路 / OG# 8路
  int errorHanding;        // 0：输出值保存  1：计时结束停止
  ParaGroup paraGroupNum;  // 输出 IO 版/组号选择（OT# 范围[1,16] OGH# [1,16] OG# [1,8]）
  ParaGroup paraGroupTime; // 时间 [0,9999]s
  ParaGroup paraGroupValue; // 输出值（OT# 端口1 选1 OGH# 端口[1,4]按位 OG# 按位）
};
// 点位数据（变量点位 AP/GP，作业/队列指令插入用）
struct PositionData
{
  std::string key{""};         // 变量名（如 AP0001）
  std::vector<double> posData; // 坐标数据，长度 7
  int coord{0};                // 0关节 1直角 2工具 3用户
  int toolNum{0};              // 工具号
  int userNum{0};              // 用户坐标系号
  PosType type{PosType::data}; // 点位类型，按变量名前缀决定（如 AP→AP_TYPE）
};

// TCP参数
struct ModbusTCPParameter
{
  std::string IP{"192.168.1.13"};
  int port{503};
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

} // namespace tl
