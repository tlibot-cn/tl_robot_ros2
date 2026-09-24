# Modbus RTU参数
# 用途：仅作 srv 嵌套参数（经 ModbusMasterParam 用于 ModbusRead / ModbusWrite），不发布话题。
int32 slave_id
int32 port
int32 baudrate
int32 data_bit
int32 stop_bit
string check_bit

