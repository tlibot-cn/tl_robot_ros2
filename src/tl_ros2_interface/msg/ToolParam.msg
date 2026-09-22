# 工具参数
# 单位约定：x/y/z 与 payload_mass_center_x/y/z 为 mm；payload_mass 为 kg；payload_inertia 为 kg·m²；
# a/b/c 见对应行尾说明（SDK 头仅写「绕 A/B/C 轴旋转量」，未标注单位）。
float64 x                        # 单位: mm（工具 X 轴偏移量）
float64 y                        # 单位: mm（工具 Y 轴偏移量）
float64 z                        # 单位: mm（工具 Z 轴偏移量）
float64 a                        # 绕 A 轴旋转量：现文档标 度，SDK 头未标注（现场核对后再定稿）
float64 b                        # 绕 B 轴旋转量：现文档标 度，SDK 头未标注（现场核对后再定稿）
float64 c                        # 绕 C 轴旋转量：现文档标 度，SDK 头未标注（现场核对后再定稿）
float64 payload_mass             # 单位: kg（负载质量）
float64 payload_inertia          # 单位: kg·m²（负载惯量）
float64 payload_mass_center_x    # 单位: mm（负载质心 X）
float64 payload_mass_center_y    # 单位: mm（负载质心 Y）
float64 payload_mass_center_z    # 单位: mm（负载质心 Z）
