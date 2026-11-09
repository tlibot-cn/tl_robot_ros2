# 机械臂关节参数
# 用途：仅作 srv 嵌套参数（GetRobotJointParam.param），不发布话题。
# 单位遵循 REP 103：角度 rad、转速 rad/s、角加速度 rad/s²；驱动在边界换算到控制器单位。
float64 reduction_ratio      # 减速比
int32 encoder_resolution     # 编码器位数
float64 pos_sw_limit         # 正向软限位，单位：rad
float64 neg_sw_limit         # 负向软限位，单位：rad
float64 rated_rot_speed      # 电机额定正转速，单位：rad/s
float64 rated_derot_speed    # 电机额定反转速，单位：rad/s（新 SDK 无对应字段，当前恒为 0）
float64 max_rot_speed        # 电机最大正转速，单位：rad/s
float64 max_derot_speed      # 电机最大反转速，单位：rad/s（新 SDK 无对应字段，当前恒为 0）
float64 rated_vel            # 额定正速度，单位：rad/s（新 SDK 无对应字段，当前恒为 0）
float64 rated_devel          # 额定反速度，单位：rad/s（新 SDK 无对应字段，当前恒为 0）
float64 max_acc              # 最大加速度，单位：rad/s²
float64 max_dec              # 最大减速度，单位：rad/s²
int32 direction              # 模型方向：1=正向，-1=反向
