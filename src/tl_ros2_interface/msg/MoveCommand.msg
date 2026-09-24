# 运动指令
# 用途：话题载荷（/tl_driver/moveJ、/tl_driver/moveL 订阅）；同时作为 srv 嵌套参数（JobInsertMove.cmd、QueueMotionMoveJ.cmd）。
float64[] target_pos_value
string target_pos_name
int32 target_pos_type
int32 coord
float64 velocity
float64 velocity_sync
float64 acc
float64 dec
int32 pl
int32 time
int32 tool_num
int32 user_num
int32 posidtype
int32 configuration
int32 spin
bool para_sync