#ifndef INCLUDE_CPP_INTERFACE_TL_QUEUE_OPERATE_H_
#define INCLUDE_CPP_INTERFACE_TL_QUEUE_OPERATE_H_

#include "cpp/parameter/tl_define.h"

/**
* @brief 打开or关闭控制器的队列运动模式
* @param status 0-关闭 1-打开
* @return 0-成功 -1-失败 -2-未连接机器人
* @warning 无论是打开还是关闭，都将清空远端控制器已经存储的队列
*/
EXPORT_API Result queue_motion_set_status(SOCKETFD socketFd, bool status);
EXPORT_API Result queue_motion_set_status_robot(SOCKETFD socketFd, int robotNum, bool status);

/**
 * @brief 查询控制器当前是否打开队列运动模式
 * @param status 用来存储返回值 0-关闭 1-已打开
 */
EXPORT_API Result queue_motion_get_status(SOCKETFD socketFd, bool& status);
EXPORT_API Result queue_motion_get_status_robot(SOCKETFD socketFd, int robotNum, bool& status);

/**
 * @brief 清空缓存的运动队列数据
 */
EXPORT_API Result queue_motion_clear_Data(SOCKETFD socketFd);
EXPORT_API Result queue_motion_clear_Data_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 查询当前运动队列的长度
 */
EXPORT_API Result queue_motion_size(SOCKETFD socketFd, int& size);
EXPORT_API Result queue_motion_size_robot(SOCKETFD socketFd, int robotNum, int& size);

/**
 * @brief 查询当前运动队列剩余的指令数量
 */
EXPORT_API Result queue_motion_get_queuelen(SOCKETFD socketFd, int& len);
EXPORT_API Result queue_motion_get_queuelen_robot(SOCKETFD socketFd, int robotNum, int& len);

/**
* @brief 将本地队列的前size个数据发送到控制器
* @param size 要发送的队列大小 size = 0时将当前运动队列全部指令发送  范围[0,31];
* @return 0-成功 -1-失败 -2-未连接机器人 -3-传入的size超过队列长度
* @warning 控制器接受到队列后，将会立马开始运动。调用前请先调用queue_motion_set_status(true);
* @iscontinue 是否选择继续发送，true -继续；false -不继续，继续发送时可以继续发送点位，机器人不运动;不继续时机器人接收点位后立刻运动,默认为false
*/
EXPORT_API Result queue_motion_send_to_controller(SOCKETFD socketFd,int size,bool isContinue = false);
EXPORT_API Result queue_motion_send_to_controller_robot(SOCKETFD socketFd, int robotNum, int size,bool isContinue = false);

/**
 * @brief 暂停连续运动
 */
EXPORT_API Result queue_motion_suspend(SOCKETFD socketFd);
EXPORT_API Result queue_motion_suspend_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 暂停后再次运行运动队列
 */
EXPORT_API Result queue_motion_restart(SOCKETFD socketFd);
EXPORT_API Result queue_motion_restart_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 停止连续运动
 * 
 * @deprecated 该接口已废弃，建议使用 `queue_motion_stop_not_power_off` 替代。
 */
EXPORT_API Result queue_motion_stop(SOCKETFD socketFd);
EXPORT_API Result queue_motion_stop_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 停止连续运动保持上电状态
 */
EXPORT_API Result queue_motion_stop_not_power_off(SOCKETFD socketFd);
EXPORT_API Result queue_motion_stop_not_power_off_robot(SOCKETFD socketFd, int robotNum);

/**
* @brief 队列运动模式的本地队列最后插入一条moveJ运动
* @param moveCmd
*  targetPos 目标点坐标 长度7位
*  velocity 参数范围(1,100] 度/s
*  acc，dec 参数范围(1,100]
*  pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveJ(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveJ_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveL运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveL(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveL_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveC运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveC(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveC_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveCA运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveCA(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveCA_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveS运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveS(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveS_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveJExtra外部轴关节运动
* @param targetPos 目标点坐标 长度7+7 本体7位 外部轴7位
* @param velocity 参数范围(1,100] 度/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveJ_extra(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveJ_extra_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条moveLExtra外部轴关节运动
* @param targetPos 目标点坐标 长度7+7 本体7位 外部轴7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_moveL_extra(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_moveL_extra_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条imove增量运动
* @param targetPos 目标点坐标 长度7+7 本体7位 外部轴7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_imove(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_imove_rbobt(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条Timer指令
* @param time 延时 单位s
*/
EXPORT_API Result queue_motion_push_back_timer(SOCKETFD socketFd, double time);
EXPORT_API Result queue_motion_push_back_timer_robot(SOCKETFD socketFd, int robotNum, double time);

/**
* @brief 队列运动模式的本地队列最后插入一条DOUT指令
* @param port  端口
* @param value  
*/
EXPORT_API Result queue_motion_push_back_dout(SOCKETFD socketFd, int port, bool value);
EXPORT_API Result queue_motion_push_back_dout_robot(SOCKETFD socketFd, int robotNum, int port, bool value);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_CHECKPOS(传送带工件检测开始)指令
* @param id 工艺号
*/
EXPORT_API Result queue_motion_push_back_conveyor_check_pos(SOCKETFD socketFd,int id);
EXPORT_API Result queue_motion_push_back_conveyor_check_pos_robot(SOCKETFD socketFd, int robotNum,int id);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_CHECKEND(传送带工件检测结束)指令
* @param id 工艺号
*/
EXPORT_API Result queue_motion_push_back_conveyor_check_end(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_conveyor_check_end_robot(SOCKETFD socketFd, int robotNum, int id);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_ON(传送带跟踪开始)指令
* @param id 工艺号
* @param posTtype 0:需要手动传入点位   1：默认使用工件点
* @param pos 坐标 长度14位 [0-7]坐标信息(坐标系,坐标值的角度弧度制等) [8-14]坐标数据
* @param vel 参数范围[2,2000] mm/s
* @param acc 参数范围[1,100]%
*/
EXPORT_API Result queue_motion_push_back_conveyor_on(SOCKETFD socketFd,int id, int postype, std::vector<double> pos, int vel, int acc);
EXPORT_API Result queue_motion_push_back_conveyor_on_robot(SOCKETFD socketFd, int robotNum,int id, int postype, std::vector<double> pos, int vel, int acc);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_OFF(传送带跟踪结束)指令
* @param id 工艺号
*/
EXPORT_API Result queue_motion_push_back_conveyor_off(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_conveyor_off_robot(SOCKETFD socketFd, int robotNum, int id);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_POS(获取传送带跟踪位置)指令
* @param id 工艺号
* @param posName 全局点位名
*/
EXPORT_API Result queue_motion_push_back_conveyor_pos(SOCKETFD socketFd, int id, const std::string posName);
EXPORT_API Result queue_motion_push_back_conveyor_pos_robot(SOCKETFD socketFd, int robotNum, int id, const std::string posName);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_POS(获取传送带跟踪位置)指令
* @param id 工艺号
* @param removeType 删除范围 0 全部目标  1本次目标
*/
EXPORT_API Result queue_motion_push_back_conveyor_clear(SOCKETFD socketFd, int id, int removeType);
EXPORT_API Result queue_motion_push_back_conveyor_clear_robot(SOCKETFD socketFd, int robotNum, int id, int removeType);

/**
* @brief 队列运动模式的本地队列最后插入一条CONVEYOR_Wait(传送带等料)指令
* @param id 工艺号
* @param overtimr 超时时间单位s
* @param materialResults 来料结果
* @param materiallLocation 来料位置
*/
EXPORT_API Result queue_motion_push_back_conveyor_wait(SOCKETFD socketFd, int id, int overtimr, std::string materialResults, std::string materiallLocation);
EXPORT_API Result queue_motion_push_back_conveyor_wait_robot(SOCKETFD socketFd, int robotNum, int id, int overtimr, std::string materialResults, std::string materiallLocation);

/**
* @brief 队列运动模式的本地队列最后插入一条ARCON(焊接开始)指令
* @param id 工艺号
*/
EXPORT_API Result queue_motion_push_back_arc_on(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_arc_on_robot(SOCKETFD socketFd, int robotNum, int id);

/**
* @brief 队列运动模式的本地队列最后插入一条ARCOFF(焊接结束)指令
* @param id 工艺号
*/
EXPORT_API Result queue_motion_push_back_arc_off(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_arc_off_robot(SOCKETFD socketFd, int robotNum, int id);

/**
* @brief 队列运动模式的本地队列最后插入一条WVON(摆焊开始)指令
* @param id 工艺号 参数范围[1-9]
*/
EXPORT_API Result queue_motion_push_back_wave_on(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_wave_on_robot(SOCKETFD socketFd, int robotNum, int id);

/**
* @brief 队列运动模式的本地队列最后插入一条WVOFF(摆焊结束)指令
* @param id 工艺号 工艺号 参数范围[1-9]
*/
EXPORT_API Result queue_motion_push_back_wave_off(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_wave_off_robot(SOCKETFD socketFd, int robotNum, int id);

/**
 * @brief 队列运动模式的本地队列最后插入一条TIGWELD(鱼鳞焊开始)指令
 * @param type 1:点焊时间  2:点焊距离
 * @param l1 点焊时间参数范围[0.01-10]S   焊接距离参数范围[0.01-9999]MM
 * @param l2 空走距离参数范围[0.01-9999]MM
 */
EXPORT_API Result queue_motion_push_back_tigweld_on(SOCKETFD socketFd, int type, double l1, double l2);
EXPORT_API Result queue_motion_push_back_tigweld_on_robot(SOCKETFD socketFd, int robotNum, int type, double l1, double l2);

/**
 * @brief 队列运动模式的本地队列最后插入一条TIGWELD(鱼鳞焊结束)指令
 */
EXPORT_API Result queue_motion_push_back_tigweld_off(SOCKETFD socketFd);
EXPORT_API Result queue_motion_push_back_tigweld_off_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 队列运动模式的本地队列最后插入一条SPOTWEDL(点焊)指令
 * @param id    工艺号
 * @param time  点焊时间范围(0,1000]
 */
EXPORT_API Result queue_motion_push_back_spot_weld(SOCKETFD socketFd, int id, double time);
EXPORT_API Result queue_motion_push_back_spot_weld_robot(SOCKETFD socketFd, int robotNum, int id, double time);

/**
 * @brief 队列运动模式的本地队列最后插入一条VISION_RUN(开始视觉)指令
 * @param id    工艺号
 */
EXPORT_API Result queue_motion_push_back_vision_craft_start(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_vision_craft_start_robot(SOCKETFD socketFd, int robotNum, int id);

/**
 * @brief 队列运动模式的本地队列最后插入一条VISION_POS(获取视觉位置)指令
 * @param id    工艺号
 * @param posName  存放位置的变量名字GP0001
 */
EXPORT_API Result queue_motion_push_back_vision_craft_get_pos(SOCKETFD socketFd, int id, const std::string posName);
EXPORT_API Result queue_motion_push_back_vision_craft_get_pos_robot(SOCKETFD socketFd, int robotNum, int id, const std::string posName);

/**
 * @brief 队列运动模式的本地队列最后插入一条VISION_TARCE(获取视觉轨迹位置)指令(24.03可用)
 */
EXPORT_API Result queue_motion_push_back_vision_craft_trajectory(SOCKETFD socketFd, VisualTrajectoryPosition param);
EXPORT_API Result queue_motion_push_back_vision_craft_trajectory_robot(SOCKETFD socketFd, int robotNum, VisualTrajectoryPosition param);

/**
 * @brief 队列运动模式的本地队列最后插入一条VISION_TRG(触发视觉)指令
 * @param id    工艺号
 * @param time  点焊时间范围(0,1000]
 */
EXPORT_API Result queue_motion_push_back_vision_craft_visual_trigger(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_vision_craft_visual_trigger_robot(SOCKETFD socketFd, int robotNum, int id);

/**
 * @brief 队列运动模式的本地队列最后插入一条VISION_ENDL(视觉结束)指令
 * @param id    工艺号
 * @param time  点焊时间范围(0,1000]
 */
EXPORT_API Result queue_motion_push_back_vision_craft_visual_end(SOCKETFD socketFd, int id);
EXPORT_API Result queue_motion_push_back_vision_craft_visual_end_robot(SOCKETFD socketFd, int robotNum, int id);

/****************寻位跟踪*******************/
/**
* @brief 队列运动模式的本地队列最后插入电弧跟踪开始
* @param craf_id 电弧跟踪文件号 1-99
* @param id 跟踪路径数据 0-9（0：不使用）
*/
EXPORT_API Result queue_motion_push_back_arctrackon(SOCKETFD socketFd,int craf_id, int id);
EXPORT_API Result queue_motion_push_back_arctrackon_rbobt(SOCKETFD socketFd, int robotNum, int craf_id, int id);

/**
* @brief 队列运动模式的本地队列最后插入电弧跟踪结束
* @param craf_id 电弧跟踪文件号 1-99
*/
EXPORT_API Result queue_motion_push_back_arctrackoff(SOCKETFD socketFd,int craf_id);
EXPORT_API Result queue_motion_push_back_arctrackoff_rbobt(SOCKETFD socketFd, int robotNum, int craf_id);

/**
* @brief 队列运动模式的本地队列最后插入弧压跟踪开始，目前未添加参数接口
* @param craf_id 电弧跟踪文件号 1-99
*/
EXPORT_API Result queue_motion_push_back_arcvtrackon(SOCKETFD socketFd,int craf_id);
EXPORT_API Result queue_motion_push_back_arcvtrackon_rbobt(SOCKETFD socketFd, int robotNum, int craf_id);

/**
* @brief 队列运动模式的本地队列最后插入弧压跟踪结束，目前未添加参数接口
* @param craf_id 电弧跟踪文件号 1-99
*/
EXPORT_API Result queue_motion_push_back_arcvtrackoff(SOCKETFD socketFd,int craf_id);
EXPORT_API Result queue_motion_push_back_arcvtrackoff_rbobt(SOCKETFD socketFd, int robotNum, int craf_id);


/**
 * @brief 队列运动模式的本地队列最后插入一条UNTIL(直到)指令
 * @param socketFd 套接字文件描述符
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层的逻辑类型，0为"与"，1为"或"
 * @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
 */
EXPORT_API Result queue_motion_push_back_until(SOCKETFD socketFd, const std::vector<std::vector<Condition>>& conditionGroups,const std::vector<int>& logic,const std::vector<std::vector<int>>& logicGroup);
EXPORT_API Result queue_motion_push_back_until_robot(SOCKETFD socketFd, int robotNum, const std::vector<std::vector<Condition>>& conditionGroups,const std::vector<int>& logic,const std::vector<std::vector<int>>& logicGroup);

/**
 * @brief 队列运动模式的本地队列最后插入一条ENDUNTIL(结束直到)指令
 */
EXPORT_API Result queue_motion_push_back_end_until(SOCKETFD socketFd);
EXPORT_API Result queue_motion_push_back_end_until_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 队列运动模式的本地队列最后插入一条WHILE(循环)指令
 * @param socketFd 套接字文件描述符
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层的逻辑类型，0为"与"，1为"或"
 * @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
 */
EXPORT_API Result queue_motion_push_back_while(SOCKETFD socketFd, const std::vector<std::vector<Condition>>& conditionGroups,const std::vector<int>& logic,const std::vector<std::vector<int>>& logicGroup);
EXPORT_API Result queue_motion_push_back_while_robot(SOCKETFD socketFd, int robotNum, const std::vector<std::vector<Condition>>& conditionGroups,const std::vector<int>& logic,const std::vector<std::vector<int>>& logicGroup);

/**
 * @brief 队列运动模式的本地队列最后插入一条ENDWHILE(结束直到)指令
 */
EXPORT_API Result queue_motion_push_back_end_while(SOCKETFD socketFd);
EXPORT_API Result queue_motion_push_back_end_while_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 队列运动模式的本地队列最后插入一条IF(如果)指令
 * @param socketFd 套接字文件描述符
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层的逻辑类型，0为"与"，1为"或"
 * @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
 */
EXPORT_API Result queue_motion_push_back_if(SOCKETFD socketFd, const std::vector<std::vector<Condition>>& conditionGroups,const std::vector<int>& logic,const std::vector<std::vector<int>>& logicGroup);
EXPORT_API Result queue_motion_push_back_if_robot(SOCKETFD socketFd, int robotNum, const std::vector<std::vector<Condition>>& conditionGroups,const std::vector<int>& logic,const std::vector<std::vector<int>>& logicGroup);

/**
 * @brief 队列运动模式的本地队列最后插入一条ENDIF(结束如果)指令
 */
EXPORT_API Result queue_motion_push_back_end_if(SOCKETFD socketFd);
EXPORT_API Result queue_motion_push_back_end_if_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 队列运动模式的本地队列最后插入一条LABEL(标签)指令
 * @param label 标签名字
 */
EXPORT_API Result queue_motion_push_back_label(SOCKETFD socketFd, const std::string &label);
EXPORT_API Result queue_motion_push_back_label_robot(SOCKETFD socketFd, int robotNum, const std::string &label);

/**
 * @brief 队列运动模式的本地队列最后插入一条JUMP(跳转)指令
 */
EXPORT_API Result queue_motion_push_back_jump(SOCKETFD socketFd, const std::vector<std::vector<Condition>>& conditionGroups, const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup, bool jumpConditionFlag, const std::string& label);
EXPORT_API Result queue_motion_push_back_jump_robot(SOCKETFD socketFd, int robotNum, const std::vector<std::vector<Condition>>& conditionGroups, const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup, bool jumpConditionFlag, const std::string& label);

/**
 * @brief 队列运动模式的本地队列最后插入一条SAMOV(定点移动)指令
 */
EXPORT_API Result queue_motion_push_back_samov(SOCKETFD socketFd, MoveCmd moveCmd,PositionData posData);
EXPORT_API Result queue_motion_push_back_samov_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd,PositionData posData);

/**
 * @brief 队列运动模式的本地队列最后插入一条CIL(相贯线)指令
 */
EXPORT_API Result queue_motion_push_back_cil(SOCKETFD socketFd, MoveCmd moveCmd,int id);
EXPORT_API Result queue_motion_push_back_cil_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd,int id);

/**
 * @brief 队列运动模式的本地队列最后插入一条TOFFSETON(轨迹偏移开始)指令
 */
EXPORT_API Result queue_motion_push_back_TOFFSETON(SOCKETFD socketFd, OffsetCommandParam params);
EXPORT_API Result queue_motion_push_back_TOFFSETON_robot(SOCKETFD socketFd, int robotNum, OffsetCommandParam params);

/**
 * @brief 队列运动模式的本地队列最后插入一条TOFFSETOFF(轨迹偏移结束)指令
 */
EXPORT_API Result queue_motion_push_back_TOFFSETOFF(SOCKETFD socketFd);
EXPORT_API Result queue_motion_push_back_TOFFSETOFF_robot(SOCKETFD socketFd, int robotNum);

/**
* @brief 队列运动模式的本地队列最后插入一条轴运动Position控制指令
* @param 参数请看AXISPostion的头文件说明
*/
EXPORT_API Result queue_motion_push_back_axis_motion_position(SOCKETFD socketFd, AXISPostion axisParam);
EXPORT_API Result queue_motion_push_back_axis_motion_position_rbobt(SOCKETFD socketFd, int robotNum, AXISPostion axisParam);

/**
* @brief 队列运动模式的本地队列最后插入一条轴运动velocity控制指令
* @param 参数请看axisParam的头文件说明
*/
EXPORT_API Result queue_motion_push_back_axis_motion_velocity(SOCKETFD socketFd, AxisVelocity axisParam);
EXPORT_API Result queue_motion_push_back_axis_motion_velocity_rbobt(SOCKETFD socketFd, int robotNum, AxisVelocity axisParam);

/**
* @brief 队列运动模式的本地队列最后插入一条轴运动torque控制指令
* @param 参数请看axisParam的头文件说明
*/
EXPORT_API Result queue_motion_push_back_axis_motion_torque(SOCKETFD socketFd, AxisTorque axisParam);
EXPORT_API Result queue_motion_push_back_axis_motion_torque_rbobt(SOCKETFD socketFd, int robotNum, AxisTorque axisParam);

/**
* @brief 队列运动模式的本地队列最后插入一条轴运动stop控制指令
* @param axisNum 主轴编号
*/
EXPORT_API Result queue_motion_push_back_axis_motion_stop(SOCKETFD socketFd, int axisNum);
EXPORT_API Result queue_motion_push_back_axis_motion_stop_rbobt(SOCKETFD socketFd, int robotNum, int axisNum);

/**
* @brief 队列运动模式的本地队列最后插入一条轴运动CancelIndependentControl控制指令
* @param axisNum 主轴编号
*/
EXPORT_API Result queue_motion_push_back_axis_motion_cancel_independent_control(SOCKETFD socketFd, int axisNum);
EXPORT_API Result queue_motion_push_back_axis_motion_cancel_independent_control_rbobt(SOCKETFD socketFd, int robotNum, int axisNum);

/**
* @brief 队列运动模式的本地队列最后插入轴运动速度判断指令
* @param 参数请看Axisspeedjudge的头文件说明
*/
EXPORT_API Result queue_motion_push_back_axis_motion_speed_judge(SOCKETFD socketFd, Axisspeedjudge param);
EXPORT_API Result queue_motion_push_back_axis_motion_speed_judge_rbobt(SOCKETFD socketFd, int robotNum, Axisspeedjudge param);

/**
* @brief 队列运动模式的本地队列最后插入主轴准停指令
* @param angle 角度
*/
EXPORT_API Result queue_motion_push_back_axis_motion_position_stop(SOCKETFD socketFd, double angle);
EXPORT_API Result queue_motion_push_back_axis_motion_position_stop_rbobt(SOCKETFD socketFd, int robotNum, double angle);

/**
* @brief 队列运动模式的本地队列最后插入一条EImoveL运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_EImoveL(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_EImoveL_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条EImoveC运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_EImoveC(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_EImoveC_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
* @brief 队列运动模式的本地队列最后插入一条EImoveCA运动
* @param targetPos 目标点坐标 长度7位
* @param velocity 参数范围(1,1000] mm/s
* @param acc，dec 参数范围(1,100]
* @param pl 多条轨迹之间平滑参数 参数范围[0,5]
*/
EXPORT_API Result queue_motion_push_back_EImoveCA(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result queue_motion_push_back_EImoveCA_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/*
* @brief 队列运动模式的本地队列最后插入声明协调参数指令
* @param crafId 协调任务号 1-999
* @param robotGroup 需要同步的机器人组 传入需要同步的机器人编号1-4
* @
*/
EXPORT_API Result queue_motion_push_back_tasks(SOCKETFD socketFd, int crafId, std::vector<int> robotGroup);
EXPORT_API Result queue_motion_push_back_tasks_rbobt(SOCKETFD socketFd, int robotNum, int crafId, std::vector<int> robotGroup);

/**
* @brief 队列运动模式的本地队列最后插入等待任务同步点指令
* @param crafId 协调任务号 1-999
* @param waitSignal 同步信号量
*/
EXPORT_API Result queue_motion_push_back_wait_sync_task(SOCKETFD socketFd,int crafId, std::string waitSignal);
EXPORT_API Result queue_motion_push_back_wait_sync_task_rbobt(SOCKETFD socketFd, int robotNum, int crafId, std::string waitSignal);

/**
* @brief 队列运动模式的本地队列最后插入外部点指令
* @param moveType 插补方式 "MovJ", "MovL"
* @param m_vel 速度 "MOVJ"速度范围[1, 100] "MOVL"速度范围[1, 1000]
* @param m_acc 插补方式 "MOVJ", "MOVL"
*/
EXPORT_API Result queue_motion_push_moveComm(SOCKETFD socketFd, std::string moveType, double m_vel, double m_acc, double m_dec, int m_time, int m_pl);
EXPORT_API Result queue_motion_push_moveComm_rbobt(SOCKETFD socketFd, int robotNum, std::string moveType, double m_vel, double m_acc, double m_dec, int m_time, int m_pl);
#endif /* INCLUDE_API_TL_QUEUE_OPERATE_H_ */
