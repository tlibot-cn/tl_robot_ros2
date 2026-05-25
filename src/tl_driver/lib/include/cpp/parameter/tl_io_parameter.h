#ifndef INCLUDE_CPP_PARAMETER_TL_IO_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_TL_IO_PARAMETER_H_

#include <string>
#include <vector>
#include "tl_define.h"

struct AlarmdIO {
    int msgType;            // 消息类型 0：普通消息，1：警告消息，2：错误消息, 22.07没有此参数，全部设为 0 即可
    int value;              // IO有效参数设置(0/1)
    int enable;             // 使能设置(0/1)
    std::string msg;        // 消息内容
};

struct RemoteProgram {
    int port;       // 远程程序端口绑定
    int value;      // 使用远程IO功能时有效参数(0/1),使用远程状态提示功能时有效参数(0/1/2)
};

struct RemoteControl {
    int clearStashPort;     // 清除断电保持数据绑定端口
    int faultResetPort;     // 清除报警端口
    int pausePort;          // 暂停端口
    int startPort;          // 启动端口
    int stopPort;           // 停止端口

    int clearStashValue;    // 清除断电保持数据端口的有效参数(0/1),与clearStashPort相对应
    int faultResetValue;    // 清除报警端口的有效参数(0/1),与faultResetPort相对应
    int pauseValue;         // 暂停端口的有效参数(0/1),与pausePort相对应
    int startValue;         // 启动端口的有效参数(0/1),与startPort相对应
    int stopValue;          // 停止端口的有效参数(0/1),与stopPort相对应

    std::vector<RemoteProgram> program; // 远程程序端口设置,详见 RemoteProgram
};

struct RemoteProgramSetting {
    std::string job;        // 远程程序选择
    int times;              // 远程程序运行次数
};

struct SafeIO {
    int quickStopPort1;     // 紧急停止端口1
    int quickStopPort2;     // 紧急停止端口2
    int quickStopValue1;    // 紧急停止参数1(0/1)
    int quickStopValue2;    // 紧急停止参数2(0/1)
    bool quickStopEnable;   // 紧急停止使能
    bool quickStopShied1;   // 屏蔽紧急停止1
    bool quickStopShied2;   // 屏蔽紧急停止2
    double quickStopTime;   // 快速停止时间，单位 毫秒(ms) 范围 [50,100]
    int quickStopShiedTime; // 屏蔽紧急停止时间， 单位 秒(s)

    int screenPort1;        // 安全光幕端口1
    int screenPort2;        // 安全光幕端口2
    int screenValue1;       // 安全光幕参数1(0/1)
    int screenValue2;       // 安全光幕参数2(0/1)
    bool screenEnable;      // 安全光幕使能
};

struct IOtype {
    int num;                                        //IO板数量
    std::vector<std::string> type;                  //IO型号
    std::vector<std::vector<int>> io_port_sum;      //IO端口数量, 一维数组: [数字输入端口数量,数字输出端口数量,模拟输入端口数量,模拟输出端口数量]
};
#endif /* INCLUDE_API_TL_IO_PARAMETER_H_ */
