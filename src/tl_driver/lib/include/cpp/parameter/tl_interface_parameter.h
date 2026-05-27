#ifndef INCLUDE_CPP_PARAMETER_TL_INTERFACE_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_TL_INTERFACE_PARAMETER_H_

#include <string>

struct ToolParam {
    double X;                       //X轴偏移方向
    double Y;                       //Y轴偏移方向
    double Z;                       //Z轴偏移方向
    double A;                       //绕A轴旋转
    double B;                       //绕B轴旋转
    double C;                       //绕C轴旋转
    double payloadMass;             //负载质量
    double payloadInertia;          //负载惯性
    double payloadMassCenter_X;     //负载质心X
    double payloadMassCenter_Y;     //负载质心Y
    double payloadMassCenter_Z;     //负载质心Z
};
struct HanYu{
    double PC = 0;
    double SP[3] = {0};//X,Y,Z
    double TL[3] = {0};//X,Y,Z
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


    double pitch{0};//螺距
    double sliding_lead_value{0};//滑动电动缸导程,酒槽机型用
    double uplift_lead_value{0};//顶升电动缸导程,酒槽机型用
    double spray_distance{0};//喷料距离,酒槽机型用

    double threeAxisDirection{0};//3轴方向
    double fiveAxisDirection{0};//五轴方向

    double twoAxisConversionRatio{0};
    double threeAxisConversionRatio{0};
    double amplificationRatio;

    double conversionratio_x{0};
    double conversionratio_y{0};
    double conversionratio_z{0};

    double conversionratio_J1{0};  //1轴转换比 五轴混动
    double conversionratio_J2{0};
    double conversionratio_J3{0};

    int upsideDown{0};

    HanYu hanyu;

};

struct ServoMovePara {
	bool clearBuffer;                       ///< 是否清除之前发送的，未开始插补计算的点位
	int targetMode;                         ///< 0-独立点 1-连续轨迹
	int sendMode;                           ///< 0-一次传输完全部轨迹 1-一次传输部分点位
	int runMode;                            ///< 0-接收完再运动 1-边接受边运动
	int sum;                                ///< 总传输次数
	int count;                              ///< 当前是第几次
	int coord;                              ///< 0-关节 1-直角
	int extMove;                            ///< 0- 1-
	int size;                               ///< 本次传输的点位数
	std::vector<std::vector<double>> pos;   ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个关节角度或笛卡尔坐标
	std::vector<std::vector<double>> axisvel; ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个轴的速度
	std::vector<std::vector<double>> axisacc; ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个轴的加速度
	std::vector<double> timeStamp;          ///< 长度为本次传输的点位数，表示到达该点位的时间，单位ms
};

struct ServoPointMovePara {
	bool end;                       ///< 是否清除之前发送的，未开始插补计算的点位
	int sum;                         ///< 总共要发的帧数
	int count;                           ///< 当前为第几帧
	std::vector<std::vector<double>> pos;   ///< 二维数组，一维表示本次传输的点位数，二维长度为12，各个关节角度或笛卡尔坐标  坐标：机器人（前7个）+ 外部轴（后5个）
};

struct RobotState {
    int channel = 1;    //查询的通道，最多9个通道
    bool stop = false; //为true时停止持续发送
    int mode = 0;       // 0-查询只回复一次  1-查询持续回复
    int interval = 10;  //仅mode = 1时有效，回复时间范围 [10,60000] ms
    bool ioState = false;   //查询IO
    int position = -1;   //0-关节坐标  1-直角坐标
    bool dataildmotionpos = false;  //机械臂的运动点位
    int posSum = 1;     //当查询机械臂运动点位时，posNum为每帧数据回复的点位数目
    std::vector<std::string> ioPort;    //IO端口，可查询的最大数量不可大于IO实际个数 例子:[ “DI1”, “DI16”, “DO1”, “DO3”, “DO17”]
    std::vector<std::string> optional;       //查询运动点位返回的坐标类型  "ACS"-关节参数 "MCS"-直角参数 "time"-时间戳 "reset"-重置点位记录
};

struct RobotJointParam {
    double reducRatio;          //减速比
    int encoderResolution;      //编码器位数
    double posSWLimit;          //轴正限位
    double negSWLimit;          //轴反限位
    double ratedRotSpeed;       //电机额定正转速
    double ratedDeRotSpeed;     //电机额定反转速
    double maxRotSpeed;         //电机最大正转速
    double maxDeRotSpeed;       //电机最大反转速
    double ratedVel;            //额定正速度
    double deRatedVel;          //额定反速度
    double maxAcc;              //最大加速度
    double maxDecel;            //最大减速度
    int direction;              //模型方向，1：正向，-1：反向
};

struct CollisionPara {
	std::vector<double> collisionDetection_run;    ///<数组，碰撞检测阈值（指令），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_run≤10000
	std::vector<double> collisionDetection_teach;  ///<数组，碰撞检测阈值（点动），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_teach≤10000
	double position_delay_time_ms_value{0.0};      ///<//指令位置响应时间，参数范围：0<position_delay_time_ms_value≤99
	double error_enable_time_ms_value{0.0};	       ///<误差允许时间，参数范围：0≤error_enable_time_ms_value≤99
	unsigned int axisum{6};                        ///<机器人轴数，默认为六轴机器人
};

struct DragParam {
    int drag_mode;                                  ///拖动模式 0-自由拖动 1-位置拖动 2-姿态拖动
    double start_threshold_F;                       ///启动阈值F
    double start_threshold_M;                       ///启动阈值M
    double dragInPosMaxVel;                         ///笛卡尔空间线速度限制
    double dragInPosMaxAngleVel;                    ///关节空间速度限制
    double drag_change_rate[6];                     ///变化率阈值[X, Y, Z, A, B, C]
    double drag_damper[6];                          ///阻尼系数[X, Y, Z, A, B, C]
    double drag_mass[6];                            ///质量系数[X, Y, Z, A, B, C]
};

struct VFDRunParam {
    int dir;                                        ///方向
    int vel;                                        ///速度
    int acceleration;                               ///加速度
    int deceleration;                               ///减速度
    bool independent_axis = true;                   ///是否为主轴
};

struct VFDState {
    int motor_vel;                                      ///电机转速
    int motor_current;                                  ///电机电流
    double spindle_angle;                               ///角度
};

struct IndependentAxisParam {
    int axis_num;                                   ///轴编号
    double angular_distance_conversion_ratio_value; ///转换比
    bool is_track;                                  ///是否是地轨 true:是 false:不是
    double encoder_bit_linedit_value;               ///编码器位数
    double inverse_limit_value;                    ///反限位
    double maximum_positive_speed_value;           ///最大正转速 单位是倍率
    double maximum_reverse_speed_value;            ///最大反转速 单位是倍率
    int motor_dir;                                  ///电机方向 1：正 -1：负
    double positive_limit_value;                   ///正限位
    double rated_positive_speed_value;             ///额定正转速 rpm
    double rated_reverse_speed_value;              ///额定反转速 rpm
    double reduction_ratio_value;                  ///减速比
    double max_acc_value;                          ///最大加速度 单位是速度的倍率
    double max_dec_value;                          ///最大减速度 单位是速度的倍率
    int select_bind_servo;                ///选择需要绑定的伺服编号,新建时值为0
};
struct IndependentAxisRun {
    int axis_num;                                   ///轴编号
    int vel;                                        //速度
    int dir;                                  ///方向 1：正 -1：负
    double acc;               ///加速度
    double dec;                    ///减速度
};

struct ToolCoordinateRange {
	double max_range_x;       ///< X轴最大范围
	double min_range_x;       ///< X轴最小范围
	double max_range_y;       ///< Y轴最大范围
	double min_range_y;       ///< Y轴最小范围
	double max_range_z;       ///< Z轴最大范围
	double min_range_z;       ///< Z轴最小范围
	int tool_hand_method;     ///< 工具握持方式
};

struct CartesianParam
{
    double maxSpeed;                /// 最大线速度 (mm/s)
    double maxAcc;                  /// 最大线加速度 (mm/s^2)
    double maxDec;                  /// 最大线减速度 (mm/s^2)
    double maxAttitudeVel;          /// 最大姿态角速度 (deg/s)
    int speedLimitMode;             /// 速度限制方式 0:位姿，1:位置
};
struct SixDimensionalForceCommunicationParams
{
        bool sensorDragEnable = true;  // 传感器拖拽使能// 默认值通常为 true
        int originDataInitialPara = 0;// 原始数据初始参数
        int sensorCommunicationType = 0;// 通讯类型 (0: EtherCAT, 1: Modbus RTU, 2: Modbus TCP)
        bool startupAutoConnectSensor = false;// 启动自动连接传感器
        int YDirection = 1;// Y 方向
        int etherCat_mapNum = 0;// EtherCAT 参数
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
struct SensorBaseParam {
    double sensorMass = 0.0;           //质量
    double sensorMassCenterX = 0.0;    //X方向质心
    double sensorMassCenterY = 0.0;    //Y方向质心
    double sensorMassCenterZ = 0.0;    //Z方向质心
    bool saveZero = false;             //是否已标零
    SensorBaseParam(double mass = 0.0, double cx = 0.0, double cy = 0.0, double cz = 0.0, bool zeroed = false)
        : sensorMass(mass), sensorMassCenterX(cx), sensorMassCenterY(cy), sensorMassCenterZ(cz), saveZero(zeroed) {}
};
struct PayloadParamBySensor {
    double payloadMass = 0.0;           ///< 计算出的负载质量
    double payloadMassCenterX = 0.0;    ///< X方向负载质心
    double payloadMassCenterY = 0.0;    ///< Y方向负载质心
    double payloadMassCenterZ = 0.0;    ///< Z方向负载质心
    // 构造函数
    PayloadParamBySensor(double mass = 0.0, double cx = 0.0, double cy = 0.0, double cz = 0.0)
        : payloadMass(mass), payloadMassCenterX(cx), payloadMassCenterY(cy), payloadMassCenterZ(cz) {}
};
#endif /* INCLUDE_API_TL_INTERFACE_PARAMETER_H_ */
