# 控制器错误/告警（/tl_driver/error_info，事件驱动，不节流）
# 数据源为控制器 6001 主端口的异步推送（set_receive_error_or_warnning_message_callback）；
# 每收到一条推送即发布一帧，不在驱动侧做频率限制或去重。
# message_type / message_code 由控制器原样透传，取值含义见控制器错误码表。
# header.stamp 为驱动收到推送的时刻；frame_id 留空。
std_msgs/Header header
int32 message_type              # SDK 回调 messageType
int32 message_code              # SDK 回调 messageCode
string message                  # SDK 回调 message 文本
