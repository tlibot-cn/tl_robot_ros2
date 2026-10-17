# 工具参数
# 用途：仅作 srv 嵌套参数（SetToolParam.param），不发布话题。
# 单位与 SDK tl::ToolParam 一致：位置 mm、旋转 度、质量 kg。
float64 x                         # 末端工具 X 轴向偏移，单位：mm
float64 y                         # 末端工具 Y 轴向偏移，单位：mm
float64 z                         # 末端工具 Z 轴向偏移，单位：mm
float64 a                         # 绕 A 轴旋转，单位：度
float64 b                         # 绕 B 轴旋转，单位：度
float64 c                         # 绕 C 轴旋转，单位：度
float64 payload_mass              # 负载质量，单位：kg
float64 payload_inertia           # 负载惯量（SDK 未标注单位）
float64 payload_mass_center_x     # 负载质心 X，单位：mm
float64 payload_mass_center_y     # 负载质心 Y，单位：mm
float64 payload_mass_center_z     # 负载质心 Z，单位：mm
