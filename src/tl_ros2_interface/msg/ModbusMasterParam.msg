# Modbus Master参数
# 用途：仅作 srv 嵌套参数（ModbusRead / ModbusWrite 请求的 master_param），不发布话题。
string type                              # 通讯类型："TCP" 或 "RTU"（大小写不敏感）
bool start_addr                          # 起始地址标志（对应 SDK startAddress）
tl_ros2_interface/ModbusTCPParam tcp     # type="TCP" 时生效
tl_ros2_interface/ModbusRTUParam rtu     # type="RTU" 时生效
