# Modbus RTU参数
# 用途：仅作 srv 嵌套参数（经 ModbusMasterParam 用于 ModbusRead / ModbusWrite），不发布话题。
int32 slave_id     # 从站 ID
int32 port         # 串口号
int32 baudrate     # 波特率，单位：bit/s
int32 data_bit     # 数据位
int32 stop_bit     # 停止位
string check_bit   # 校验方式（字符串，取值由控制器定义）
