# 关节速度（/tl_driver/joint_speed，默认 100 Hz，feedback_rate 可配）
# 数据源为控制器 6001 端口的同步查询；驱动边界换算为 REP 103。
# 与 /tl_driver/line_speed 由同一次查询产生，共享 feedback_rate。
# 数组定长，元素顺序与关节一致；查询失败或数据不足时缺失元素为 NaN（quiet_NaN），消费方用 std::isnan() 判定。
std_msgs/Header header
float64[7] joint_speed          # 本体关节速度，单位：rad/s；超出实际轴数或缺失为 NaN
