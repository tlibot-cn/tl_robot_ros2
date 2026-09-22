# 机器人DH参数
# 单位约定：l1..l20、pitch、sliding_lead_value、uplift_lead_value、spray_distance、sp[3]/tl[3] 为 mm；
# couple_coe_*、*convertion_ratio*、amplification_ratio、upside_down 无量纲；
# 其余字段 SDK 头未标注单位，以控制器手册 / 现场核对为准。
float64 l1        # 单位: mm（DH / 连杆长度参数）
float64 l2        # 单位: mm（DH / 连杆长度参数）
float64 l3        # 单位: mm（DH / 连杆长度参数）
float64 l4        # 单位: mm（DH / 连杆长度参数）
float64 l5        # 单位: mm（DH / 连杆长度参数）
float64 l6        # 单位: mm（DH / 连杆长度参数）
float64 l7        # 单位: mm（DH / 连杆长度参数）
float64 l8        # 单位: mm（DH / 连杆长度参数）
float64 l9        # 单位: mm（DH / 连杆长度参数）
float64 l10       # 单位: mm（DH / 连杆长度参数）
float64 l11       # 单位: mm（DH / 连杆长度参数）
float64 l12       # 单位: mm（DH / 连杆长度参数）
float64 l13       # 单位: mm（DH / 连杆长度参数）
float64 l14       # 单位: mm（DH / 连杆长度参数）
float64 l15       # 单位: mm（DH / 连杆长度参数）
float64 l16       # 单位: mm（DH / 连杆长度参数）
float64 l17       # 单位: mm（DH / 连杆长度参数）
float64 l18       # 单位: mm（DH / 连杆长度参数）
float64 l19       # 单位: mm（DH / 连杆长度参数）
float64 l20       # 单位: mm（DH / 连杆长度参数）

float64 couple_coe_1_2   # 无量纲（耦合系数）
float64 couple_coe_2_3   # 无量纲（耦合系数）
float64 couple_coe_3_2   # 无量纲（耦合系数）
float64 couple_coe_3_4   # 无量纲（耦合系数）
float64 couple_coe_4_5   # 无量纲（耦合系数）
float64 couple_coe_4_6   # 无量纲（耦合系数）
float64 couple_coe_5_6   # 无量纲（耦合系数）

float64 dynamic_limit_max   # 动态限位上限（SDK 头未标注单位）
float64 dynamic_limit_min   # 动态限位下限（SDK 头未标注单位）

float64 pitch               # 单位: mm（螺距）
float64 sliding_lead_value  # 单位: mm（滑动电动缸导程，酒槽机型用）
float64 uplift_lead_value   # 单位: mm（顶升电动缸导程，酒槽机型用）
float64 spray_distance      # 单位: mm（喷料距离，酒槽机型用）

float64 three_axis_direction   # 3轴方向（SDK 头未标注单位）
float64 five_axis_direction    # 五轴方向（SDK 头未标注单位）

float64	two_axis_convertion_ratio     # 无量纲（转换比）
float64 three_axis_convertion_ratio   # 无量纲（转换比）
float64 amplification_ratio           # 无量纲（放大比）

float64 convertion_ratio_x   # 无量纲（转换比）
float64 convertion_ratio_y   # 无量纲（转换比）
float64 convertion_ratio_z   # 无量纲（转换比）

float64 convertion_ratio_j1   # 无量纲（转换比，五轴混动）
float64 convertion_ratio_j2   # 无量纲（转换比，五轴混动）
float64 convertion_ratio_j3   # 无量纲（转换比，五轴混动）

int32 upside_down   # 无量纲（是否倒装：0/1）

float64 pc          # 汉禹(HanYu)机型参数 PC（SDK 头未标注单位）
float64[] sp        # 汉禹机型 SP 点 X,Y,Z（长度 3），单位: mm
float64[] tl        # 汉禹机型 TL 点 X,Y,Z（长度 3），单位: mm

