# 机械臂运行状态
# 用途：话题载荷（/arm_status 发布，100 Hz）；不被任何 srv 引用。
builtin_interfaces/Time stamp   # 采样时间戳
string run_state                # 运行状态："STOP" / "PAUSE" / "RUNNING" / "UNKNOWN"（SDK 返回未识别值）
