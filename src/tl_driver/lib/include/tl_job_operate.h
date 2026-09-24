/**
 * @file tl_job_operate.h
 * @brief 作业文件接口（namespace tl）
 */
#ifndef TL_SDK_TL_JOB_H
#define TL_SDK_TL_JOB_H

#include <string>
#include <vector>
#include "tl_types.h"

// ==================== 协议层结构体前置声明（临时桥接） ====================
// 以下结构体当前仅在协议层参数头中有完整定义，tl 侧对应结构体
// （PositionData / Condition / IOCommandParams）尚未加入 tl_types.h。
// 此处仅前置声明，保证本头文件自包含；待主会话在 tl_types.h 中
// 统一补充 tl 侧对应结构体后，相关函数将切换为 tl 侧类型，
// 并交由内部转换函数完成与协议层参数的转换。
struct PositionData;
struct Condition;
struct IOCommandParams;

namespace tl
{


/**
 * @brief 获取所有作业文件名
 * @param robotsFile 二维数组，一维长度是4，对应4个机器人
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_get_all_jobfile_name(SOCKETFD socketFd,
                                       std::vector<std::vector<std::string>>& robotsFile);

/**
 * @brief 新建作业文件
 * @param jobName 作业文件名 只允许字母开头，字母数字组合
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如作业文件名不合法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
 * 异常；-6=TIMEOUT 超时
 * @test 新建QQQ.JBR job_create(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_create(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 删除指定的作业文件
 * @param jobName 作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 删除QQQ.JBR job_delete(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_delete(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 打开指定的作业文件
 * @param jobName 作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 打开QQQ.JBR job_open(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_open(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 运行指定的作业文件
 * @param jobName 作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 运行QQQ.JBR job_run(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_run(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 根据文件名上传一个作业文件
 * @param filePath 文件的完整路径
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如文件不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_upload_by_file(SOCKETFD socketFd, const std::string& filePath);

/**
 * @brief 下载所有作业文件到指定文件夹
 * @param directoryPath 目录的完整路径
 * @param isCover 是否覆盖同名文件，true=覆盖 / false=不覆盖
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如目录不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_download_by_directory(SOCKETFD socketFd, const std::string& directoryPath,
                                        bool isCover);

/**
 * @brief 下载指定数量的日志文件到指定文件夹
 * @param counts 文件数量
 * @param directoryPath 目录的完整路径
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result log_download_by_quantity(SOCKETFD socketFd, int counts,
                                       const std::string& directoryPath);

/**
 * @brief 向作业文件插入一条moveJ关节运动
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
 * 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveJ(SOCKETFD socketFd, int line, MoveCmd moveCmd);

/**
 * @brief 向作业文件插入一条moveL
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
 * 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveL(SOCKETFD socketFd, int line, MoveCmd moveCmd);

/**
 * @brief 向作业文件插入一条moveC
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveC(SOCKETFD socketFd, int line, MoveCmd moveCmd);

/**
 * @brief 向作业文件插入一条增量指令IMOVE
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @note moveCmd.targetPosType 必须设置为 PosType::RP_TYPE(3)，moveCmd.targetPosName 如 "RP0001"
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_imove(SOCKETFD socketFd, int line, MoveCmd moveCmd);
// ====================================================================
// ===================== 作业文件传输与系统备份 =====================
// ====================================================================

/**
 * @brief 根据文件夹上传一整个文件夹的作业文件
 * @param directoryPath 目录的完整路径
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如目录不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_upload_by_directory(SOCKETFD socketFd, const std::string& directoryPath);

/**
 * @brief 上传作业文件后同步刷新示教器中的作业列表
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_sync_job_file(SOCKETFD socketFd);

/**
 * @brief 一键备份系统，备份文件保存至当前执行程序所在目录下
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result backup_system(SOCKETFD socketFd);

// ====================================================================
// ========================= 作业运行控制 =========================
// ====================================================================

/**
 * @brief 单步运行指定作业文件的某一行
 * @param jobName 作业文件名
 * @param line 行号，范围 [1, 最大行号]
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如行号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 运行 QQQ.JBR 的第一行 job_step(socketFd, "QQQ", 1);
 */
TL_API Result job_step(SOCKETFD socketFd, const std::string& jobName, int line);

/**
 * @brief 暂停作业文件
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_pause(SOCKETFD socketFd);

/**
 * @brief 继续运行已暂停的作业文件
 * @note 需要处于运行模式
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_continue(SOCKETFD socketFd);

/**
 * @brief 停止作业文件（不会下电）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_stop(SOCKETFD socketFd);

/**
 * @brief 设置作业文件运行次数
 * @param index 运行次数，0 表示无限次
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_run_times(SOCKETFD socketFd, int index);

/**
 * @brief 从断点处继续运行作业文件
 * @param jobName 作业文件名
 * @note 需要处于运行模式
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_break_point_run(SOCKETFD socketFd, const std::string& jobName);

// ====================================================================
// ========================= 作业文件信息查询 =========================
// ====================================================================

/**
 * @brief 获取当前打开的作业文件名称
 * @param jobName 输出参数，当前打开的作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_get_current_file(SOCKETFD socketFd, std::string& jobName);

/**
 * @brief 获取当前打开的作业文件名称（字符容器形式，供 C# 侧使用）
 * @param jobName 输出参数，当前打开的作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_get_current_file_csharp(SOCKETFD socketFd, std::vector<char>& jobName);

/**
 * @brief 获取当前打开的作业文件运行到的行数
 * @param line 输出参数，运行到的行数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_get_current_line(SOCKETFD socketFd, int& line);

// ====================================================================
// ======================= 向作业文件插入指令 =======================
// ====================================================================

/**
 * @brief 向作业文件插入一个局部点位
 * @param posData 点位数据参数（key/posData/coord/toolNum/userNum/type，完整定义见协议层参数头）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误（如点位数据非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT
 * 超时
 */
TL_API Result job_insert_local_position(SOCKETFD socketFd, PositionData posData);

/**
 * @brief 向作业文件插入一条外部点指令 moveComm
 * @param line 插入的位置
 * @param moveType 插补方式，如 "MovJ"、"MovL"
 * @param m_vel 速度
 * @param m_acc 加速度
 * @param m_dec 减速度
 * @param m_time 时间
 * @param m_pl 平滑级别
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveComm(SOCKETFD socketFd, int line, std::string moveType, double m_vel,
                                  double m_acc, double m_dec, int m_time, int m_pl);

/**
 * @brief 向作业文件插入一条 SAMOV（定点移动）指令
 * @param line 插入的位置
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @param posData 点位数据参数（详见协议层参数头 PositionData）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_samov_command(SOCKETFD socketFd, int line, MoveCmd moveCmd,
                                       PositionData posData);

/**
 * @brief 在作业文件中插入一条延时指令
 * @param line 插入的位置
 * @param time 延时时间，单位 s
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_timer_command(SOCKETFD socketFd, int line, double time);

/**
 * @brief 在作业文件中插入一条 IO 输出指令
 * @param line 插入的位置
 * @param params IO
 * 输出参数（groupType/errorHanding/paraGroupNum/paraGroupTime/paraGroupValue，完整定义见协议层参数头）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_io_out_command(SOCKETFD socketFd, int line, IOCommandParams params);

/**
 * @brief 向作业文件插入一条 CIL（相贯线）指令
 * @param line 插入的位置
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @param id 工艺号参数
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_cil(SOCKETFD socketFd, int line, MoveCmd moveCmd, int id);

/**
 * @brief 向作业文件插入一条 UNTIL（直到）指令
 * @param line 插入的位置
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层逻辑类型，0 为“与”，1 为“或”
 * @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_until(SOCKETFD socketFd, int line,
                               const std::vector<std::vector<Condition>>& conditionGroups,
                               const std::vector<int>& logic,
                               const std::vector<std::vector<int>>& logicGroup);

/**
 * @brief 向作业文件插入一条 ENDUNTIL（结束直到）指令
 * @param line 插入的位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_end_until(SOCKETFD socketFd, int line);

/**
 * @brief 向作业文件插入一条 WHILE 指令
 * @param line 插入的位置
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层逻辑类型，0 为“与”，1 为“或”
 * @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_while(SOCKETFD socketFd, int line,
                               const std::vector<std::vector<Condition>>& conditionGroups,
                               const std::vector<int>& logic,
                               const std::vector<std::vector<int>>& logicGroup);

/**
 * @brief 向作业文件插入一条 ENDWHILE（结束循环）指令
 * @param line 插入的位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_end_while(SOCKETFD socketFd, int line);

/**
 * @brief 向作业文件插入一条 IF 指令
 * @param line 插入的位置
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层逻辑类型，0 为“与”，1 为“或”
 * @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_if(SOCKETFD socketFd, int line,
                            const std::vector<std::vector<Condition>>& conditionGroups,
                            const std::vector<int>& logic,
                            const std::vector<std::vector<int>>& logicGroup);

/**
 * @brief 向作业文件插入一条 ENDIF（结束条件）指令
 * @param line 插入的位置
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_end_if(SOCKETFD socketFd, int line);

/**
 * @brief 向作业文件插入一条 LABEL（标签）指令
 * @param line 插入的位置
 * @param label 标签名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_label(SOCKETFD socketFd, int line, const std::string& label);

/**
 * @brief 向作业文件插入一条 JUMP（跳转）指令
 * @param line 插入的位置
 * @param conditionGroups 条件组的二维向量
 * @param logic 第一层逻辑类型，0 为“与”，1 为“或”
 * @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
 * @param jumpConditionFlag 是否带跳转条件
 * @param label 跳转的目标标签名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_jump(SOCKETFD socketFd, int line,
                              const std::vector<std::vector<Condition>>& conditionGroups,
                              const std::vector<int>& logic,
                              const std::vector<std::vector<int>>& logicGroup,
                              bool jumpConditionFlag, const std::string& label);

// ====================================================================
// ========================= 多机协调指令 =========================
// ====================================================================

/**
 * @brief 向作业文件插入声明协调参数指令
 * @param line 插入的位置
 * @param crafId 协调任务号，范围 1-999
 * @param robotGroup 需要同步的机器人编号列表（1-4）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_tasks(SOCKETFD socketFd, int line, int crafId,
                               std::vector<int> robotGroup);

/**
 * @brief 向作业文件插入等待任务同步点指令
 * @param line 插入的位置
 * @param crafId 协调任务号，范围 1-999
 * @param waitSignal 同步信号量
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
 * 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_wait_sync_task(SOCKETFD socketFd, int line, int crafId,
                                        std::string waitSignal);

} // namespace tl

#endif /* TL_SDK_TL_JOB_H */
