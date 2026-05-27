#ifndef INCLUDE_CPP_PARAMETER_TL_CRAFT_VISION_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_TL_CRAFT_VISION_PARAMETER_H_

#include <string>
#include <vector>

struct CameraList {
	std::string currentName{"customize"};
	int listNum{0};
};

struct Protocol {
	std::string addDataInitialPara{"GD001"}; // 附加数据初始参数
	int addDataNum{0};                   // 附加数据数量
	int angleUnit{1};                    // 角度单位
	std::string endMark{"$"};            // 结束标志
	std::string failFlag{"NG"};          // 失败标志
	std::string frameHeader{""};         // 帧头
	bool hasTCS{false};                  // 是否有TCS
	bool hasUCS{false};                  // 是否有UCS
	std::string separator{","};          // 分隔符
	bool singleTarget{true};             // 单目标
	std::string successFlag{"OK"};       // 成功标志
	int timeOut{30};                     // 超时时间
	int type{1};                         // 类型
};

struct Socket {
	std::string IP{"192.168.1.120"}; // IP地址
	int cameraDataType{0};           // 相机数据类型
	int portNum{1};                  // 端口号
	int portOne{5050};               // 端口1
	int portTwo{5051};               // 端口2
	bool server{true};               // 是否为服务器
};

struct Trigger {
	int IOPort{0};             // IO端口
	int duration{1000};           // 持续时间
	int intervals{35};          // 间隔时间
	int triggerMode{2};        // 触发模式 1 IO  2 Ethercat
	bool triggerOnce{true};       // 单次触发
	std::string triggerStr{"TRG"}; // 触发字符串
};

struct VisionParam {
	CameraList cameraList;
	Protocol protocol;
	Socket socket;
	Trigger trigger;
    int userCoordNum{0};  // 用户坐标编号
};

struct VisionRange {
    std::string maxX{""};  // 最大X坐标
    std::string maxY{""};  // 最大Y坐标
    std::string maxZ{""};  // 最大Z坐标
    std::string minX{""};  // 最小X坐标
    std::string minY{""};  // 最小Y坐标
    std::string minZ{""};  // 最小Z坐标
};

struct Excursion {
	double Xexcursion{0.0};       // X轴偏移
	double Yexcursion{0.0};       // Y轴偏移
	double Zexcursion{0.0};       // Z轴偏移
	double angle{0.0};           // 角度
};

struct Position {
	int angleDirection{-1};          // 视角方向（角度）
	std::string cameraData{""};      // 相机数据
	std::vector<double> cameraPoint{14, 0.0};   // 相机点位
	std::vector<double> datumPoint{14, 0.0};    // 参考点位
	std::vector<double> datumPointDeg{14, 0.0}; // 参考点位（角度）
	Excursion excursion;
	int recvPointsType{0};                     // 接收点类型
	std::string sampleData{"x,y,Rz,h,$"};      // 样本数据
	double scale{1.0};                         // 比例
};

struct VisionPositionParam {
	Position position;
	int protocol{0}; // 协议类型
};

struct CalibrationPoint {
	std::vector<double> pixel_pos{7,0.0};         // 像素位置
	std::vector<double> pixel_pos_deg{7,0.0};     // 像素位置（角度）
	std::vector<double> robot_pos{7,0.0};         // 机器人位置
	std::vector<double> robot_pos_deg{7,0.0};     // 机器人位置（角度）
};

struct Calibration {
	bool calibrated;                       // 是否已校准
	std::vector<CalibrationPoint> point;   // 校准点
	int point_num;                         // 点的数量

	void addCalibrationPoint(const CalibrationPoint& pos) {
		point.push_back(pos);
	}

	CalibrationPoint getPoint(int index) const {
		if (static_cast<size_t>(index) < point.size() && index >= 0) {
			return point[index];
		}
		return CalibrationPoint();
	}

	Calibration(int num_points = 6) : calibrated(false), point_num(num_points) {
		// 确保点数在合理范围内
		if (point_num < 6) point_num = 6;
		else if (point_num > 30) point_num = 30;
	}
};

struct VisionCalibrationData {
    int visionNum;                         // 视觉编号
    Calibration calibration;               // 校准数据
    VisionCalibrationData(int num_points = 6)
    : visionNum(0), calibration(num_points) {}
};

#endif /* INTERFACE_PARAMETER_TL_CRAFT_VISION_PARAMETER_H_ */
