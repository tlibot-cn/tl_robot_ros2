#ifndef INCLUDE_CPP_PARAMETER_TL_CRAFT_TRACK_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_TL_CRAFT_TRACK_PARAMETER_H_


struct TrackOffsetParam {
    double acc_factor = 1.0;         //补偿加速度倍数；取值范围[0.1,10],对应当前运动指令加速
    int algorithm_type = 0;       //偏差提取类型；取值范围[0,1]0:均值；1:手填值  左右补偿目前只支持均值
    double custom_value = 0;      //偏差提取类型手填值；取值范围[0,1000]A
    int begin_cycle_num = 4;         //开始采样周期数；取值范围[1,100]
    double differential = 0;         //微分系数；取值范围[0,1]
    double integral = 0.0005;        //积分系数；取值范围[0,1]
    double max_single_len = 2;       //每次最大补偿量；取值范围[0,10]
    double scale = 0.05;                //比例系数；取值范围[0,1]
    double switchon = 0;             //补偿开关；true：打开，false：关闭
};

#endif/*INCLUDE_CPP_PARAMETER_TL_CRAFT_WELD_PARAMETER_H_*/