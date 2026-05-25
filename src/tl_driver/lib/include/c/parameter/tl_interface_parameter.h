#ifndef INCLUDE_C_PARAMETER_TL_INTERFACE_PARAMETER_H_
#define INCLUDE_C_PARAMETER_TL_INTERFACE_PARAMETER_H_

#ifdef __cplusplus
extern "C" {
#endif

struct CRobotDHParam
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
};

struct CRobotJointParam {
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

struct CIndependentAxisParam {
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
struct CIndependentAxisRun {
    int axis_num;                                   ///轴编号
    int vel;                                        //速度
    int dir;                                  ///方向 1：正 -1：负
    double acc;               ///加速度
    double dec;                    ///减速度
};
#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_API_TL_INTERFACE_PARAMETER_H_ */
