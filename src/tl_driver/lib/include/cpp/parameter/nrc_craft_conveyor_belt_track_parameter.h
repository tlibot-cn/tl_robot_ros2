#ifndef INCLUDE_CPP_PARAMETER_NRC_CRAFT_CONVEYOR_BELT_TRACK_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_NRC_CRAFT_CONVEYOR_BELT_TRACK_PARAMETER_H_

#include <string>
#include <vector>

struct ConveyorBasicParams
{
    int conveyorID;                             // 输送带ID
    double compensation_encoderVal;              // 补偿编码器值
    double compensation_time;                    // 补偿时间
    int conveyor_encoderDirection;                // 输送带编码器方向
    double conveyor_encoderResolution;            // 输送带编码器分辨率
    double conveyor_encoderValue;                 // 输送带编码器当前值
    double conveyor_maxEncoderVal;                // 输送带最大编码器值
    double conveyor_minEncoderVal;                // 输送带最小编码器值
    int conveyor_posRecordMode;                   // 输送带位置记录模式
    double conveyor_speed;                        // 输送带速度
    int conveyor_userCoord;                       // 用户坐标系
    int track_height;                             // 轨道高度
    int track_on_run_mode_with_target_overrun;    // 轨道运行模式与目标超限
};

struct ConveyorIdentificationParams
{
    int conveyorID;                                  // 输送带ID
    int detectSrc_DI_capturePos;                     // 捕获位置
    std::string detectSrc_globalVar;                 // 全局变量
    int detectSrc_type;                              // 检测源类型
    int detectSrc_visionID;                          // 视觉ID
    int detectSrc_vision_io_filter_type;             // 视觉IO过滤类型
    int detectSrc_vision_latch_encoder_value_type;   // 视觉锁存编码器值类型
    std::string identification_communication;        // 识别通信
    int identification_sensorTrg;                    // 识别传感器目标
    int identification_type;                          // 识别类型
};

struct ConveyorSensorParams
{
    int conveyorID;                                   // 输送带ID
    double sensorPosDeg_A;                            // A轴角度
    double sensorPosDeg_B;                            // B轴角度
    double sensorPosDeg_C;                            // C轴角度
    double sensorPosDeg_X;                            // X轴角度
    double sensorPosDeg_Y;                            // Y轴角度
    double sensorPosDeg_Z;                            // Z轴角度
};

struct ConveyorTrackRangeParams
{
    int conveyorID;                                   // 输送带ID
    double position_receLatestPos;                    // 最迟到接收位置
    double position_trackRangeXMax;                   // 轨道范围X最大
    double position_trackRangeYMax;                   // 轨道范围Y最大
    double position_trackRangeYMin;                   // 轨道范围Y最小
    double position_trackRangeZMax;                   // 轨道范围Z最大
    double position_trackRangeZMin;                   // 轨道范围Z最小
    double position_trackStartXPoint;                 // 轨道起始X点
};

struct ConveyorWaitPointParams
{
    int conveyorID;                                   // 输送带ID
    double delayDetectTime;                           // 延迟检测时间
    bool isWait;                                      // 是否等待
    std::vector<double> pos;                          // 位置数组
};

#endif /* INTERFACE_PARAMETER_NRC_CRAFT_CONVEYOR_BELT_TRACK_PARAMETER_H_ */
