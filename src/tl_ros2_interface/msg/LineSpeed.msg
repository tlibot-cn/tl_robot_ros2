# 末端线速度（/tl_driver/line_speed，默认 10 Hz，line_speed_rate 可配）
# 数据源为控制器 6001 端口的同步查询；单位 m/s。
std_msgs/Header header
float64 line_speed              # 末端线速度，单位：m/s；查询失败时该周期不发布，字段保持上一次有效值
