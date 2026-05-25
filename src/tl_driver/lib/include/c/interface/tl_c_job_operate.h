/*
 * tl_c_job_operate.h
 *
 *  Created on: 2025年8月19日
 *      Author: shanyufeng
 */

#ifndef INCLUDE_C_INTERFACE_TL_C_QUEUE_OPERATE_H_
#define INCLUDE_C_INTERFACE_TL_C_QUEUE_OPERATE_H_

#include "c/parameter/tl_define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 回到设定的零点
 */
EXPORT_API int robot_go_home_c(SOCKETFD socketFd);
EXPORT_API int robot_go_home_robot_c(SOCKETFD socketFd, int robotNum);

/**
* @brief 获取所有作业文件名
* @param robotNum
* @param buffer 字符串流
* @param maxFileNum 当前需要获取的作业数量（为0时获取全部）
* @return
*/
EXPORT_API int job_get_all_jobfile_name_c(SOCKETFD socketFd, int robotNum, uint8_t buffer[][64], size_t maxFileNum);

/**
* @brief 根据文件夹上传一整个文件夹的作业文件
* @param directoryPath 目录的完整路径
*/
EXPORT_API int job_upload_by_directory_c(SOCKETFD socketFd, const char* directoryPath);

/**
* @brief 根据文件名上传一个作业文件
* @param filePath 文件的完整路径
*/
EXPORT_API int job_upload_by_file_c(SOCKETFD socketFd, const char* filePath);

/**
 * @brief 上传作业文件同步刷新示教器
 */
EXPORT_API int job_sync_job_file_c(SOCKETFD socketFd);

/**
* @brief 下载所有作业文件到指定文件夹
* @param directoryPath 目录的完整路径
*/
EXPORT_API int job_download_by_directory_c(SOCKETFD socketFd, const char* directoryPath, bool isCover);

/**
 * @brief 下载指定数量的日志文件到指定文件夹
 * @param counts 文件数量
 * @param directoryPath 目录的完整路径
 */
EXPORT_API int log_download_by_quantity_c(SOCKETFD socketFd, int counts, const char* directoryPath);

/**
 * @brief 一键备份系统，会保存至当前执行程序目录下
 */
EXPORT_API int backup_system_c(SOCKETFD socketFd);

/**
* @brief 新建作业文件
* @param jobName 作业文件名 只允许字母开头，字母数字组合
* @test 新建QQQ.JBR job_create(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API int job_create_c(SOCKETFD socketFd, const char* jobName);
EXPORT_API int job_create_robot_c(SOCKETFD socketFd, int robotNum, const char* jobName);

/**
* @brief 删除指定的作业文件
* @param jobName 作业文件名
* @test 删除QQQ.JBR job_delete(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API int job_delete_c(SOCKETFD socketFd, const char* jobName);
EXPORT_API int job_delete_robot_c(SOCKETFD socketFd, int robotNum, const char* jobName);

/**
* @brief 打开指定的作业文件
* @param jobName 作业文件名
* @test 打开QQQ.JBR job_open(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API int job_open_c(SOCKETFD socketFd, const char* jobName);
EXPORT_API int job_open_robot_c(SOCKETFD socketFd, int robotNum, const char* jobName);

/**
* @brief 获取作业文件总行号
*/
EXPORT_API int job_get_command_total_lines_c(SOCKETFD socketFd, int* totalLines);
EXPORT_API int job_get_command_total_lines_robot_c(SOCKETFD socketFd, int robotNum, int* totalLines);

/**
* @brief 获取对应行号的作业文件内容
* @param line 行号
*/
EXPORT_API int job_get_command_content_by_line_c(SOCKETFD socketFd, int line, char* jobContent);
EXPORT_API int job_get_command_content_by_line_robot_c(SOCKETFD socketFd, int robotNum, int line, char* jobContent);

/**
* @brief 删除指定行号
*/
EXPORT_API int job_delete_command_by_line_c(SOCKETFD socketFd, int line);
EXPORT_API int job_delete_command_by_line_robot_c(SOCKETFD socketFd, int robotNum, int line);

/**
* @brief 运行指定的作业文件
* @param jobName 作业文件名
* @test 运行QQQ.JBR job_run(SOCKETFD socketFd,"QQQ");
*/
EXPORT_API int job_run_c(SOCKETFD socketFd, const char* jobName);
EXPORT_API int job_run_robot_c(SOCKETFD socketFd, int robotNum, const char* jobName);

/**
* @brief 单步运行指定的作业文件的某一行
* @param jobName 作业文件名
* @param line 行号 [1,最大行号]
* @test 运行QQQ.JBR的第一行 job_step(SOCKETFD socketFd,"QQQ",1);
*/
EXPORT_API int job_step_c(SOCKETFD socketFd, const char* jobName, int line);
EXPORT_API int job_step_robot_c(SOCKETFD socketFd, int robotNum, const char* jobName, int line);

/**
* @brief 暂停作业文件
*/
EXPORT_API int job_pause_c(SOCKETFD socketFd);
EXPORT_API int job_pause_robot_c(SOCKETFD socketFd, int robotNum);

/**
* @brief 继续运行作业文件
* @note 需要运行模式
*/
EXPORT_API int job_continue_c(SOCKETFD socketFd);
EXPORT_API int job_continue_robot_c(SOCKETFD socketFd, int robotNum);

/**
* @brief 停止作业文件(不会下电)
*/
EXPORT_API int job_stop_c(SOCKETFD socketFd);
EXPORT_API int job_stop_robot_c(SOCKETFD socketFd, int robotNum);

/**
* @brief 设置作业文件运行次数
* @param index 运行次数 0-无限次
*/
EXPORT_API int job_run_times_c(SOCKETFD socketFd, int index);
EXPORT_API int job_run_times_robot_c(SOCKETFD socketFd, int robotNum, int index);

/**
 * @brief 继续运行作业文件
 * @note 需要运行模式
 */
EXPORT_API int job_break_point_run_c(SOCKETFD socketFd, const char* jobName);
EXPORT_API int job_break_point_run_robot_c(SOCKETFD socketFd, int robotNum, const char* jobName);

/**
 * @brief 获取当前打开的作业文件名称
 * @param jobname 当前打开的作业文件
 */
EXPORT_API int job_get_current_file_c(SOCKETFD socketFd, char* jobName);
EXPORT_API int job_get_current_file_robot_c(SOCKETFD socketFd, int robotNum, char* jobName);

/**
 * @brief 获取当前打开的作业文件运行到的行数
 * @param  line 运行到的行数
 */
EXPORT_API int job_get_current_line_c(SOCKETFD socketFd, int* line);
EXPORT_API int job_get_current_line_robot_c(SOCKETFD socketFd, int robotNum, int* line);

//========================向作业文件插入运动指令==========================
/**
* @brief 向作业文件插入一条moveJ关节运动
* @param line 插入的位置
* @param vel 速度 范围(0,100]
* @param acc 加速度 范围(0,100]
* @param dec 减速度 范围(0,100]
* @param pl 平滑 范围[0,5]
* @param posName 全局变量位置名称  例: "GP0001"
*/
EXPORT_API int job_insert_moveJ_c(SOCKETFD socketFd,  int line, double vel, double acc, double dec, int pl, const char* posName);
EXPORT_API int job_insert_moveJ_robot_c(SOCKETFD socketFd, int robotNum,  int line, double vel, double acc, double dec, int pl, const char* posName);

/**
* @brief 向作业文件插入一条moveL
* @param line 插入的位置
* @param vel 速度 范围(0,1000]
* @param acc 加速度 范围(0,100]
* @param dec 减速度 范围(0,100]
* @param pl 平滑 范围[0,5]
* @param posName 全局变量位置名称  例: "GP0001"
*/
EXPORT_API int job_insert_moveL_c(SOCKETFD socketFd,  int line, double vel, double acc, double dec, int pl, const char* posName);
EXPORT_API int job_insert_moveL_robot_c(SOCKETFD socketFd, int robotNum,  int line, double vel, double acc, double dec, int pl, const char* posName);

/**
* @brief 向作业文件插入一条moveC
* @param line 插入的位置
* @param vel 速度 范围(0,1000]
* @param acc 加速度 范围(0,100]
* @param dec 减速度 范围(0,100]
* @param pl 平滑 范围[0,5]
* @param posName 全局变量位置名称  例: "GP0001"
*/
EXPORT_API int job_insert_moveC_c(SOCKETFD socketFd,  int line, double vel, double acc, double dec, int pl, const char* posName);
EXPORT_API int job_insert_moveC_robot_c(SOCKETFD socketFd, int robotNum,  int line, double vel, double acc, double dec, int pl, const char* posName);

#ifdef __cplusplus
}
#endif
#endif /* INTERFACE_C_INTERFACE_TL_C_QUEUE_OPERATE_H_ */
