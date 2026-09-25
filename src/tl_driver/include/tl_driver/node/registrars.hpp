#ifndef TL_DRIVER__NODE__REGISTRARS_HPP_
#define TL_DRIVER__NODE__REGISTRARS_HPP_

namespace tl_driver
{

class DriverNode;

/**
 * @brief 注册会话类服务（连接/断开/上下电/清错）。
 * @param[in] node 驱动节点
 */
void register_session_services(DriverNode& node);

/**
 * @brief 注册运动与伺服类服务（速度、点动、ServoJ 开关、队列运动）。
 * @param[in] node 驱动节点
 */
void register_motion_services(DriverNode& node);

/**
 * @brief 注册坐标与位姿变换类服务。
 * @param[in] node 驱动节点
 */
void register_transform_services(DriverNode& node);

/**
 * @brief 注册配置类服务（坐标系、工具、DH、模式、全局点位）。
 * @param[in] node 驱动节点
 */
void register_config_services(DriverNode& node);

/**
 * @brief 注册 IO 与外部通信类服务（数字量、Modbus、日志下载、轨迹）。
 * @param[in] node 驱动节点
 */
void register_io_services(DriverNode& node);

/**
 * @brief 注册作业文件类服务（job 运行/删除/插入、文件列表）。
 * @param[in] node 驱动节点
 */
void register_job_services(DriverNode& node);

/**
 * @brief 注册状态查询类服务（机器人状态、关节参数与温度/电压/电流、力矩、版本、拖拽）。
 * @param[in] node 驱动节点
 */
void register_state_services(DriverNode& node);

/**
 * @brief 注册运动订阅话题（moveJ/moveL/servoJ/servoL）。
 * @param[in] node 驱动节点
 */
void register_topics(DriverNode& node);

} // namespace tl_driver

#endif // TL_DRIVER__NODE__REGISTRARS_HPP_
