/**
 * @file tl_job.h
 * @brief 作业文件接口（namespace tl）
 */
#ifndef TL_SDK_TL_JOB_H
#define TL_SDK_TL_JOB_H

#include <string>
#include <vector>
#include "tl_types.h"

namespace tl
{


/**
 * @brief 获取所有作业文件名
 * @param robotsFile 二维数组，一维长度是4，对应4个机器人
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_get_all_jobfile_name(SOCKETFD socketFd, std::vector<std::vector<std::string>>& robotsFile);

/**
 * @brief 新建作业文件
 * @param jobName 作业文件名 只允许字母开头，字母数字组合
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如作业文件名不合法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 新建QQQ.JBR job_create(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_create(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 删除指定的作业文件
 * @param jobName 作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 删除QQQ.JBR job_delete(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_delete(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 打开指定的作业文件
 * @param jobName 作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 打开QQQ.JBR job_open(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_open(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 运行指定的作业文件
 * @param jobName 作业文件名
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 * @test 运行QQQ.JBR job_run(SOCKETFD socketFd,"QQQ");
 */
TL_API Result job_run(SOCKETFD socketFd, const std::string& jobName);

/**
 * @brief 根据文件名上传一个作业文件
 * @param filePath 文件的完整路径
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如文件不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_upload_by_file(SOCKETFD socketFd, const std::string& filePath);

/**
 * @brief 下载所有作业文件到指定文件夹
 * @param directoryPath 目录的完整路径
 * @param isCover 是否覆盖同名文件，true=覆盖 / false=不覆盖
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如目录不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_download_by_directory(SOCKETFD socketFd, const std::string& directoryPath, bool isCover);

/**
 * @brief 下载指定数量的日志文件到指定文件夹
 * @param counts 文件数量
 * @param directoryPath 目录的完整路径
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result log_download_by_quantity(SOCKETFD socketFd, int counts, const std::string& directoryPath);

/**
 * @brief 向作业文件插入一条moveJ关节运动
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveJ(SOCKETFD socketFd, int line, MoveCmd moveCmd);

/**
 * @brief 向作业文件插入一条moveL
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveL(SOCKETFD socketFd, int line, MoveCmd moveCmd);

/**
 * @brief 向作业文件插入一条moveC
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_moveC(SOCKETFD socketFd, int line, MoveCmd moveCmd);

/**
 * @brief 向作业文件插入一条增量指令 IMOVE
 * @param line 插入的行号
 * @param moveCmd 运动指令参数，详见 MoveCmd；增量指令需 targetPosType=3（变量）、targetPosName="RP0001"（厂商约定）
 * @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR 参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
 */
TL_API Result job_insert_imove(SOCKETFD socketFd, int line, MoveCmd moveCmd);

} // namespace tl

#endif /* TL_SDK_TL_JOB_H */
