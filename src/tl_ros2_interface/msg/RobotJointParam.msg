# 机械臂关节参数
# 单位约定：pos_sw_limit / neg_sw_limit 为 度；各转速与额定速度字段为 度/s。
float64 reduction_ratio      # 无量纲（减速比）
int32 encoder_resolution     # 编码器位数
float64 pos_sw_limit         # 单位: 度（轴正限位）
float64 neg_sw_limit         # 单位: 度（轴反限位）
float64 rated_rot_speed      # 单位: 度/s（电机额定正转速）
float64 rated_derot_speed    # 单位: 度/s（电机额定反转速）
float64 max_rot_speed        # 单位: 度/s（电机最大正转速）
float64 max_derot_speed      # 单位: 度/s（电机最大反转速）
float64 rated_vel            # 单位: 度/s（额定正速度）
float64 rated_devel          # 单位: 度/s（额定反速度）
float64 max_acc              # 最大加速度（SDK 头未标注单位）
float64 max_dec              # 最大减速度（SDK 头未标注单位）
int32 direction              # 模型方向：1=正向 -1=反向（无量纲）