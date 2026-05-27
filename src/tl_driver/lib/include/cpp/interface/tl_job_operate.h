#ifndef INCLUDE_CPP_INTERFACE_TL_JOB_OPERATE_H_
#define INCLUDE_CPP_INTERFACE_TL_JOB_OPERATE_H_

#include "cpp/parameter/tl_define.h"
#include <string>

/**
* @brief 获取所有作业文件名
* @param robotsFile 二维数组，一维长度是4，对应4个机器人
*/
EXPORT_API Result job_get_all_jobfile_name(SOCKETFD socketFd, std::vector<std::vector<std::string>>& robotsFile);

/**
* @brief 根据文件夹上传一整个文件夹的作业文件
* @param directoryPath 目录的完整路径
*/
EXPORT_API Result job_upload_by_directory(SOCKETFD socketFd, const std::string& directoryPath);

/**
* @brief 根据文件名上传一个作业文件
* @param filePath 文件的完整路径
*/
EXPORT_API Result job_upload_by_file(SOCKETFD socketFd, const std::string& filePath);

/**
 * @brief 上传作业文件同步刷新示教器
 */
EXPORT_API Result job_sync_job_file(SOCKETFD socketFd);

/**
* @brief 下载所有作业文件到指定文件夹
* @param directoryPath 目录的完整路径
*/
EXPORT_API Result job_download_by_directory(SOCKETFD socketFd, const std::string& directoryPath, bool isCover);

/**
 * @brief 下载指定数量的日志文件到指定文件夹
 * @param counts 文件数量
 * @param directoryPath 目录的完整路径
 */
EXPORT_API Result log_download_by_quantity(SOCKETFD socketFd, int counts, const std::string& directoryPath);

/**
 * @brief 一键备份系统，会保存至当前执行程序目录下
 */
EXPORT_API Result backup_system(SOCKETFD socketFd);

/**
* @brief 新建作业文件
* @param jobName 作业文件名 只允许字母开头，字母数字组合
* @test 新建QQQ.JBR job_create(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API Result job_create(SOCKETFD socketFd, const std::string& jobName);
EXPORT_API Result job_create_robot(SOCKETFD socketFd, int robotNum, const std::string& jobName);

/**
* @brief 删除指定的作业文件
* @param jobName 作业文件名
* @test 删除QQQ.JBR job_delete(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API Result job_delete(SOCKETFD socketFd, const std::string& jobName);
EXPORT_API Result job_delete_robot(SOCKETFD socketFd, int robotNum, const std::string& jobName);

/**
* @brief 打开指定的作业文件
* @param jobName 作业文件名
* @test 打开QQQ.JBR job_open(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API Result job_open(SOCKETFD socketFd, const std::string& jobName);
EXPORT_API Result job_open_robot(SOCKETFD socketFd, int robotNum, const std::string& jobName);

/**
* @brief 获取作业文件总行号
*/
EXPORT_API Result job_get_command_total_lines(SOCKETFD socketFd, int& totalLines);
EXPORT_API Result job_get_command_total_lines_robot(SOCKETFD socketFd, int robotNum, int& totalLines);

/**
* @brief 获取对应行号的作业文件内容
* @param line 行号
*/
EXPORT_API Result job_get_command_content_by_line(SOCKETFD socketFd, int line, std::string& jobContent);
EXPORT_API Result job_get_command_content_by_line_robot(SOCKETFD socketFd, int robotNum, int line, std::string& jobContent);

/**
* @brief 删除指定行号
*/
EXPORT_API Result job_delete_command_by_line(SOCKETFD socketFd, int line);
EXPORT_API Result job_delete_command_by_line_robot(SOCKETFD socketFd, int robotNum, int line);

/**
* @brief 运行指定的作业文件
* @param jobName 作业文件名
* @test 运行QQQ.JBR job_run(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API Result job_run(SOCKETFD socketFd, const std::string& jobName);
EXPORT_API Result job_run_robot(SOCKETFD socketFd, int robotNum, const std::string& jobName);

/**
* @brief 单步运行指定的作业文件的某一行
* @param jobName 作业文件名
* @param line 行号 [1,最大行号]
* @test 运行QQQ.JBR的第一行 job_step(SOCKETFD socketFd,"QQQ",1);
*/
EXPORT_API Result job_step(SOCKETFD socketFd, const std::string& jobName, int line);
EXPORT_API Result job_step_robot(SOCKETFD socketFd, int robotNum, const std::string& jobName, int line);

/**
* @brief 暂停作业文件
*/
EXPORT_API Result job_pause(SOCKETFD socketFd);
EXPORT_API Result job_pause_robot(SOCKETFD socketFd, int robotNum);

/**
* @brief 继续运行作业文件
* @note 需要运行模式
*/
EXPORT_API Result job_continue(SOCKETFD socketFd);
EXPORT_API Result job_continue_robot(SOCKETFD socketFd, int robotNum);

/**
* @brief 停止作业文件(不会下电)
*/
EXPORT_API Result job_stop(SOCKETFD socketFd);
EXPORT_API Result job_stop_robot(SOCKETFD socketFd, int robotNum);

/**
* @brief 设置作业文件运行次数
* @param index 运行次数 0-无限次
*/
EXPORT_API Result job_run_times(SOCKETFD socketFd, int index);
EXPORT_API Result job_run_times_robot(SOCKETFD socketFd, int robotNum, int index);

/**
 * @brief 继续运行作业文件
 * @note 需要运行模式
 */
EXPORT_API Result job_break_point_run(SOCKETFD socketFd, const std::string& jobName);
EXPORT_API Result job_break_point_run_robot(SOCKETFD socketFd, int robotNum, const std::string& jobName);

/**
 * @brief 获取当前打开的作业文件名称(c#用下面这个支持的类型)
 * @param jobname 当前打开的作业文件
 */
EXPORT_API Result job_get_current_file(SOCKETFD socketFd, std::string& jobName);
EXPORT_API Result job_get_current_file_robot(SOCKETFD socketFd, int robotNum, std::string& jobName);

/**
 * @brief 获取当前打开的作业文件名称(c#用这个)
 * @param jobname 当前打开的作业文件
 */
EXPORT_API Result job_get_current_file_csharp(SOCKETFD socketFd, std::vector<char>& jobName);
EXPORT_API Result job_get_current_file_csharp_robot(SOCKETFD socketFd, int robotNum, std::vector<char>& jobName);

/**
 * @brief 获取当前打开的作业文件运行到的行数
 * @param  line 运行到的行数
 */
EXPORT_API Result job_get_current_line(SOCKETFD socketFd, int& line);
EXPORT_API Result job_get_current_line_robot(SOCKETFD socketFd, int robotNum, int& line);

/**
* @brief 作业文件插入一个局部点位
* @param posData 位置数据
*/
EXPORT_API Result job_insert_local_position(SOCKETFD socketFd, PositionData posData);
EXPORT_API Result job_insert_local_position_robot(SOCKETFD socketFd, int robotNum, PositionData posData);

/**
* @brief 根据点位名修改当前作业文件局部点位
* @param posName 需要修改的点位名
* @param posInfo[14]  [0]坐标系 0：关节 1：直角 2：工具 3：用户   [1]:0 角度制  1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
*/
EXPORT_API Result job_set_local_position(SOCKETFD socketFd, const std::string& posName, std::vector<double> posInfo);
EXPORT_API Result job_set_local_position_robot(SOCKETFD socketFd, int robotNum, const std::string& posName, std::vector<double> posInfo);

//====================================================================
//========================向作业文件插入运动指令==========================
//====================================================================
/**
* @brief 向作业文件插入一条moveJ关节运动
* @param moveCmd  运动指令参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_moveJ(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_moveJ_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

/**
* @brief 向作业文件插入一条moveL
* @param moveCmd  运动指令参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_moveL(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_moveL_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

/**
* @brief 向作业文件插入一条moveC
* @param moveCmd  运动指令参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_moveC(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_moveC_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

/**
* @brief 向作业文件插入一条增量指令IMOVE
* moveCmd.targetPosType = 3
* moveCmd.targetPosName = "RP0001"
*/
EXPORT_API Result job_insert_imove(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_imove_rbobt(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

/**
* @brief 向作业文件插入一条外部点指令moveComm
* @param moveType 插补方式 "MovJ", "MovL"
*/
EXPORT_API Result job_insert_moveComm(SOCKETFD socketFd, int line, std::string moveType, double m_vel, double m_acc, double m_dec, int m_time, int m_pl);
EXPORT_API Result job_insert_moveComm_rbobt(SOCKETFD socketFd, int robotNum, int line, std::string moveType, double m_vel, double m_acc, double m_dec, int m_time, int m_pl);

/**
* @brief 向作业文件插入一条SAMOV(定点移动)
* @param moveCmd  运动指令参数 点位数据用 PositionData
* @param line 插入的位置
*/
EXPORT_API Result job_insert_samov_command(SOCKETFD socketFd, int line, MoveCmd moveCmd, PositionData posData);
EXPORT_API Result job_insert_samov_command_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd, PositionData posData);

/**
* @brief 向作业文件插入一条EImoveL
* @param moveCmd  运动指令参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_EImoveL(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_EImoveL_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

/**
* @brief 向作业文件插入一条EImoveC
* @param moveCmd  运动指令参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_EImoveC(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_EImoveC_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

/**
* @brief 向作业文件插入一条EImoveCA
* @param moveCmd  运动指令参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_EImoveCA(SOCKETFD socketFd, int line, MoveCmd moveCmd);
EXPORT_API Result job_insert_EImoveCA_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd);

//====================================================================
//========================向作业文件插入延迟指令==========================
//====================================================================
/**
* @brief 在作业文件中插入一条延时指令
* @param time 延时时间 单位:s
* @param line 插入的位置
*/
EXPORT_API Result job_insert_timer_command(SOCKETFD socketFd, int line, double time);
EXPORT_API Result job_insert_timer_command_robot(SOCKETFD socketFd, int robotNum, int line, double time);

//====================================================================
//========================向作业文件插入IO指令===========================
//====================================================================
/**
* @brief 在作业文件中插入一条IO输出
* @param line 插入的行数
*/
EXPORT_API Result job_insert_io_out_command(SOCKETFD socketFd, int line, IOCommandParams params);
EXPORT_API Result job_insert_io_out_command_robot(SOCKETFD socketFd, int robotNum, int line, IOCommandParams params);

//==================================================================
//========================向作业文件插入逻辑控制指令=====================
//==================================================================
/**
* @brief 向作业文件插入一条until
* @param conditionGroups 条件组的二维向量
* @param logic 第一层的逻辑类型，0为"与"，1为"或"
* @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
*/
EXPORT_API Result job_insert_until(SOCKETFD socketFd, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup);
EXPORT_API Result job_insert_until_robot(SOCKETFD socketFd, int robotNum, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup);

/**
* @brief 向作业文件插入一条enduntil
*/
EXPORT_API Result job_insert_end_until(SOCKETFD socketFd, int line);
EXPORT_API Result job_insert_end_until_robot(SOCKETFD socketFd, int robotNum, int line);

/**
* @brief 向作业文件插入一条while
* @param conditionGroups 条件组的二维向量
* @param logic 第一层的逻辑类型，0为"与"，1为"或"
* @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
*/
EXPORT_API Result job_insert_while(SOCKETFD socketFd, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup);
EXPORT_API Result job_insert_while_robot(SOCKETFD socketFd, int rbobtNum, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup);

 /**
 * @brief 向作业文件插入一条ENDWHILE(结束直到)指令
 */
EXPORT_API Result job_insert_end_while(SOCKETFD socketFd, int line);
EXPORT_API Result job_insert_end_while_robot(SOCKETFD socketFd, int rbobtNum, int line);

/**
 * @brief 向作业文件插入一条IF
 * @param socketFd 套接字文件描述符
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层的逻辑类型，0为"与"，1为"或"
 * @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
 */
EXPORT_API Result job_insert_if(SOCKETFD socketFd, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup);
EXPORT_API Result job_insert_if_robot(SOCKETFD socketFd, int robotNum, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup);

 /**
 * @brief 向作业文件插入一条ENDIF(结束如果)指令
 */
EXPORT_API Result job_insert_end_if(SOCKETFD socketFd, int line);
EXPORT_API Result job_insert_end_if_robot(SOCKETFD socketFd, int robotNum, int line);

/**
 * @brief 向作业文件插入一条LABEL(标签)指令
 * @param label 标签名字
 */
EXPORT_API Result job_insert_label(SOCKETFD socketFd, int line, const std::string &label);
EXPORT_API Result job_insert_label_robot(SOCKETFD socketFd, int robotNum, int line, const std::string &label);

/**
 * @brief 向作业文件插入一条JUMP(跳转)指令
 */
EXPORT_API Result job_insert_jump(SOCKETFD socketFd, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup, bool jumpConditionFlag, const std::string& label);
EXPORT_API Result job_insert_jump_robot(SOCKETFD socketFd, int robotNum, int line, const std::vector<std::vector<Condition>>& conditionGroups,
		const std::vector<int>& logic, const std::vector<std::vector<int>>& logicGroup, bool jumpConditionFlag, const std::string& label);

//====================================================================
//========================向作业文件插入视觉工艺指令=======================
//====================================================================
/**
* @brief 向作业文件插入一条VISION_RUN(开始视觉)指令
* @param id 工艺号
* @param line 插入位置
*/
EXPORT_API Result job_insert_vision_craft_start(SOCKETFD socketFd, int line, int id);
EXPORT_API Result job_insert_vision_craft_start_robot(SOCKETFD socketFd, int robotNum, int line, int id);

/**
* @brief 向作业文件插入一条VISION_POS(获取视觉位置)指令
* @param id 工艺号
* @param line 插入位置
* @param posName 存放位置的变量名 例如GP0001
*/
EXPORT_API Result job_insert_vision_craft_get_pos(SOCKETFD socketFd, int line, int id, const std::string posName);
EXPORT_API Result job_insert_vision_craft_get_pos_robot(SOCKETFD socketFd, int robotNum, int line, int id, const std::string posName);

/**
 * @brief 向作业文件插入一条VISION_TARCE(获取视觉轨迹位置)指令(24.03可用)
 */
EXPORT_API Result job_insert_vision_craft_trajectory(SOCKETFD socketFd, int line, VisualTrajectoryPosition param);
EXPORT_API Result job_insert_vision_craft_trajectory_robot(SOCKETFD socketFd, int robotNum, int line, VisualTrajectoryPosition param);

/**
* @brief 向作业文件插入一条VISION_POS(触发视觉)指令
* @param id 工艺号
* @param line 插入位置
*/
EXPORT_API Result job_insert_vision_craft_visual_trigger(SOCKETFD socketFd, int line, int id);
EXPORT_API Result job_insert_vision_craft_visual_trigger_robot(SOCKETFD socketFd, int robotNum, int line, int id);

/**
* @brief 向作业文件插入一条VISION_END(视觉结束)指令
* @param id 工艺号
* @param line 插入位置
*/
EXPORT_API Result job_insert_vision_craft_visual_end(SOCKETFD socketFd, int line, int id);
EXPORT_API Result job_insert_vision_craft_visual_end_robot(SOCKETFD socketFd, int robotNum, int line, int id);

//====================================================================
//========================向作业文件插入传送带工艺指令=====================
//====================================================================
/**
* @brief 向作业文件插入一条CONVEYOR_CHECKPOS(传送带工件检测开始)指令
* @param id 工艺号
*/
EXPORT_API Result job_insert_conveyor_check_pos(SOCKETFD socketFd, int line, int id);
EXPORT_API Result job_insert_conveyor_check_pos_robot(SOCKETFD socketFd, int robotNum, int line, int id);

/**
* @brief 向作业文件插入一条CONVEYOR_CHECKEND(传送带工件检测结束)指令
* @param id 工艺号
*/
EXPORT_API Result job_insert_conveyor_check_end(SOCKETFD socketFd, int line, int id);
EXPORT_API Result job_insert_conveyor_check_end_robot(SOCKETFD socketFd, int robotNum, int line, int id);

/**
* @brief 向作业文件插入一条CONVEYOR_ON(传送带跟踪开始)指令
* @param id 工艺号
* @param posTtype 0:需要手动传入点位   1：默认使用工件点
* @param pos 坐标 长度14位 [0-7]坐标信息(坐标系,坐标值的角度弧度制等) [8-14]坐标数据
* @param vel 参数范围[2,2000] mm/s
* @param acc 参数范围[1,100]%job_insert_
* @param line 插入位置
*/
EXPORT_API Result job_insert_conveyor_on(SOCKETFD socketFd, int line, int id, int postype, std::vector<double> pos, int vel, int acc);
EXPORT_API Result job_insert_conveyor_on_robot(SOCKETFD socketFd, int robotNum, int line, int id, int postype, std::vector<double> pos, int vel, int acc);

/**
* @brief 向作业文件插入一条CONVEYOR_OFF(传送带跟踪结束)指令
* @param id 工艺号
*/
EXPORT_API Result job_insert_conveyor_off(SOCKETFD socketFd, int line, int id);
EXPORT_API Result job_insert_conveyor_off_robot(SOCKETFD socketFd, int robotNum, int line, int id);

/**
* @brief 向作业文件插入一条CONVEYOR_POS(获取传送带跟踪位置)指令
* @param id 工艺号
* @param posName 全局点位名
*/
EXPORT_API Result job_insert_conveyor_pos(SOCKETFD socketFd, int line, int id, const std::string posName);
EXPORT_API Result job_insert_conveyor_pos_robot(SOCKETFD socketFd, int robotNum, int line, int id, const std::string posName);

/**
* @brief 向作业文件插入一条CONVEYOR_POS(获取传送带跟踪位置)指令
* @param id 工艺号
* @param removeType 删除范围 0 全部目标  1本次目标
*/
EXPORT_API Result job_insert_conveyor_clear(SOCKETFD socketFd, int line, int id, int removeType);
EXPORT_API Result job_insert_conveyor_clear_robot(SOCKETFD socketFd, int robotNum, int line, int id, int removeType);

/**
* @brief 向作业文件插入一条CONVEYOR_Wait(传送带等料)指令(24.03可用)
* @param id 工艺号
* @param line 插入的位置
* @param overtimr 超时时间单位s
* @param materialResults 来料结果
* @param materiallLocation 来料位置
*/
EXPORT_API Result job_insert_conveyor_wait(SOCKETFD socketFd, int line, int id, int overtimr, std::string materialResults, std::string materiallLocation);
EXPORT_API Result job_insert_conveyor_wait_robot(SOCKETFD socketFd, int robotNum, int line, int id, int overtimr, std::string materialResults, std::string materiallLocation);

//====================================================================

/**
* @brief 向作业文件插入一条CIL(相贯线)
* @param moveCmd  指令参数
* @param id 工艺号参数
* @param line 插入的位置
*/
EXPORT_API Result job_insert_cil(SOCKETFD socketFd, int line, MoveCmd moveCmd, int id);
EXPORT_API Result job_insert_cil_robot(SOCKETFD socketFd, int robotNum, int line, MoveCmd moveCmd, int id);

/**
* @brief 向作业文件插入轴点到点运行控制指令
* @param line 插入的位置
*/
EXPORT_API Result job_insert_axis_motion_position(SOCKETFD socketFd, int line, AXISPostion axisParam);
EXPORT_API Result job_insert_axis_motion_position_rbobt(SOCKETFD socketFd, int robotNum, int line, AXISPostion axisParam);

/**
* @brief 向作业文件插入轴恒转速控制指令
* @param line 插入的位置
*/
EXPORT_API Result job_insert_axis_motion_velocity(SOCKETFD socketFd, int line, AxisVelocity axisParam);
EXPORT_API Result job_insert_axis_motion_velocity_rbobt(SOCKETFD socketFd, int robotNum, int line, AxisVelocity axisParam);

/**
* @brief 向作业文件插入轴恒转矩控制指令
* @param line 插入的位置
*/
Result job_insert_axis_motion_torque(SOCKETFD socketFd, int line, AxisTorque axisParam);
Result job_insert_axis_motion_torque_rbobt(SOCKETFD socketFd, int robotNum, int line, AxisTorque axisParam);

/**
* @brief 向作业文件插入轴运动stop控制指令
* @param line 插入的位置
*/
EXPORT_API Result job_insert_axis_motion_stop(SOCKETFD socketFd, int line, int axisNum);
EXPORT_API Result job_insert_axis_motion_stop_rbobt(SOCKETFD socketFd, int robotNum, int line, int axisNum);

/**
* @brief 向作业文件插入轴运动CancelIndependentControl控制指令
* @param line 插入的位置
*/
EXPORT_API Result job_insert_axis_motion_cancel_independent_control(SOCKETFD socketFd, int line, int axisNum);
EXPORT_API Result job_insert_axis_motion_cancel_independent_control_rbobt(SOCKETFD socketFd, int robotNum, int line, int axisNum);

/**
* @brief 向作业文件插入轴运动速度判断指令
* @param line 插入的位置
*/
EXPORT_API Result job_insert_axis_motion_speed_judge(SOCKETFD socketFd, int line, Axisspeedjudge param);
EXPORT_API Result job_insert_axis_motion_speed_judge_rbobt(SOCKETFD socketFd, int robotNum, int line, Axisspeedjudge param);

/**
* @brief 向作业文件插入主轴准停指令
* @param line 插入的位置
* @param angle 角度
*/
EXPORT_API Result job_insert_axis_motion_position_stop(SOCKETFD socketFd, int line, double angle);
EXPORT_API Result job_insert_axis_motion_position_stop_rbobt(SOCKETFD socketFd, int robotNum, int line, double angle);

#endif /* INCLUDE_API_TL_JOB_OPERATE_H_ */

//====================================================================
//========================向作业文件插入多机协调指令=====================
//====================================================================

/**
* @brief 向作业文件插入声明协调参数指令
* @param line 插入的位置
* @param crafId 协调任务号 1-999
* @param robotGroup 需要同步的机器人组 传入需要同步的机器人编号1-4
* @
*/
EXPORT_API Result job_insert_tasks(SOCKETFD socketFd, int line, int crafId, std::vector<int> robotGroup);
EXPORT_API Result job_insert_tasks_rbobt(SOCKETFD socketFd, int robotNum, int line, int crafId, std::vector<int> robotGroup);

/**
* @brief 向作业文件插入等待任务同步点指令
* @param line 插入的位置
* @param crafId 协调任务号 1-999
* @param waitSignal 同步信号量
*/
EXPORT_API Result job_insert_wait_sync_task(SOCKETFD socketFd, int line, int crafId, std::string waitSignal);
EXPORT_API Result job_insert_wait_sync_task_rbobt(SOCKETFD socketFd, int robotNum, int line, int crafId, std::string waitSignal);
