# 关节热状态（/tl_driver/joint_thermal，默认 0.5 Hz）
# 控制器逐关节查询单次约 150–350 ms，频率上限受此限制；设 0 可关闭。
std_msgs/Header header
float64[] temperature           # 关节温度，单位：°C
float64[] joint_voltage         # 关节电压，单位：V
float64[] positioner_voltage    # 外部轴电压，单位：V
