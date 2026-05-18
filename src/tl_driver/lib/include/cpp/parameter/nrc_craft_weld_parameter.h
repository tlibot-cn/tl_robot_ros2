#ifndef INCLUDE_CPP_PARAMETER_NRC_CRAFT_WELD_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_NRC_CRAFT_WELD_PARAMETER_H_

struct ArcParam {
    double weldVoltage = 0;           //焊接电压 范围[-1000,1000]V
    double weldCurrent = 0;           //焊接电流 范围[0,1000]A
    double arcOnCurrent = 0;          //起弧电流 范围[0,1000]A
    double arcOnVoltage = 0;          //起弧电压 范围[-1000,1000]V
    double arcOnTime = 0;             //起弧时间 范围[0,5]S
    bool arcOnRampEnable = false;     //起弧渐变 0:未使能 1:使能
    int arcOnRampMode = 0;            //起弧方式 时间渐变
    double arcOnRampTime = 0;         //起弧渐变时间 范围[0，100000]ms
    double arcOffCurrent = 0;         //收弧电流 范围[0,1000]A
    double arcOffVoltage = 0;         //收弧电压 范围[-1000,1000]V
    double arcOffTime = 0;            //收弧时间 范围[0,5]S
    bool arcOffRampEnable = false;    //收弧渐变 0:未使能 1:使能
    int arcOffRampMode = 0;           //收弧方式 时间渐变
    double arcOffRampTime = 0;        //收弧渐变时间 范围[0，100000]ms
};

struct WaveParam {
    int type;                       //摆法方式 0 1 2 3 正弦摆 Z字形 圆形摆 外部轴定点摆
    double swingFreq;               //摆动频率
    double swingAmplitude;          //摆动幅度
    double radius;                  //半径
    double LTypeAngle;              //L型角度
    bool moveWhenEdgeStay;          //边缘停留
    double leftStayTime;            //左停留时间
    double rightStayTime;           // 右停留时间
    int initialDir;                 //初始方向
    double horizontalDeflection;    //水平偏转
    double verticalDeflection;      //垂直偏转
};

struct WeldState {
    int pistolSwitch{0};         //焊枪开关  -1:未使能 0:错误 1:使能
    int arcingSuccess{0};        //引弧成功  -1:未使能 0:错误 1:使能
    int handWireFeed{0};         //手动送丝  -1:未使能 0:错误 1:使能
    double weldCurrent{0};       //焊接电流
    double weldVoltage{0};       //焊接电压
    double weldTime{0};          //焊接时间
    double weldPWM{0};           //焊接占空比
};

#endif /* INTERFACE_PARAMETER_NRC_CRAFT_WELD_PARAMETER_H_ */
