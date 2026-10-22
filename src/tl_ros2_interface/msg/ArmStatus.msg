# 机械臂运行状态
# 用途：话题载荷（/arm_status 发布，100 Hz）；不被任何 srv 引用。
builtin_interfaces/Time stamp   # 采样时间戳
string run_state                # 运行状态："STOP" / "PAUSE" / "RUNNING" / "UNKNOWN"（SDK 返回未识别值）
float64 speed                   # 运行速度百分比 %（1 Hz 查询缓存，与 SetSpeed 同口径）
bool speed_valid                # speed 是否有效（最近一次 SDK get_speed 查询成功）
