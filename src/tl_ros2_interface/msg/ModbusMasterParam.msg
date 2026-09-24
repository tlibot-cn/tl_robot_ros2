# Modbus Master参数
# 用途：仅作 srv 嵌套参数（ModbusRead / ModbusWrite 请求的 master_param），不发布话题。
string type
bool start_addr
tl_ros2_interface/ModbusTCPParam tcp
tl_ros2_interface/ModbusRTUParam rtu
