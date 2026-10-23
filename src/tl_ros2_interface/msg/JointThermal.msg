# 关节热状态（/tl_driver/joint_thermal，默认 1 Hz）
# 控制器逐关节查询单次约 150–350 ms，频率上限受此限制；设 0 可关闭。
# 数组定长，查询失败或数据不足时缺失元素为 NaN（quiet_NaN），消费方用 std::isnan() 判定。
std_msgs/Header header
float64[7] temperature          # 关节温度，单位：°C；缺失为 NaN
float64[7] joint_voltage        # 关节电压，单位：V；缺失为 NaN
float64[5] positioner_voltage   # 外部轴电压，单位：V；缺失为 NaN
