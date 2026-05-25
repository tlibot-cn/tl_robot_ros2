#ifndef INCLUDE_CPP_PARAMETER_TL_CRAFT_LASER_CUTTING_PARAMETER_H_
#define INCLUDE_CPP_PARAMETER_TL_CRAFT_LASER_CUTTING_PARAMETER_H_

struct LaserCuttingEquipment {
	double RetreatDistance{0.0};
	int arrivalOutLightMode{0};
	double collisionDistance{10.0};
	int delAspiratedMode{1};
	double delAspiratedTime{0.0};
	bool focusCompensation{false};
	double focusCompensationConstant{0.0};
	double focusCompensationPower{0.0};
	double focusCompensationTime{0.0};
	int focusFormula{0};
	int follow{0};
	double preAspiratedTime{0.0};
	int rePerforate{1};
	double waitFollowTime{1.0};
	double waitLiftUpTime{1.0};
};

struct LaserCuttingGlobalParam {
	LaserCuttingEquipment equipment;
};

struct LaserCuttingParam {
	int dutyRatio{0};
	double focusPosition{0.0};
	double followHeight{0.10};
	int freq{1};
	int power{1};
	double pressure{0.0};
	std::string note{""};
};

struct LaserCuttingCraftParam {
	int num;  // 工艺号
	LaserCuttingParam cut;
};

struct Curve {
    int level;
    std::vector<double> x;
    std::vector<double> y;
};

struct LaserCuttingAnalogMatch {
	Curve laserPower;  // 激光功率匹配
	Curve pressure;  // 气压匹配
};

struct LaserCuttingAnalogParam {
	LaserCuttingAnalogMatch analogMatch;
};

struct IO {
    int AO_laserPower;
    int AO_pressure;
    int DI_backMiddleArrival;
    int DI_capacitance_;
    int DI_followArrival;
    int DI_liftUpArrival;
    int DI_perforateArrival;
    int DO_aspiration;
    int DO_backMiddle;
    int DO_capacitance_;
    int DO_follow;
    int DO_highPressgas;
    int DO_liftUp;
    int DO_lightGate;
    int DO_lowPressgas;
    int DO_redLight;
    int pwm_port_;
};

struct Fault {
    int input_port;
};

struct LaserCuttingIOParam {
    IO io;
    Fault laser_fault;
    Fault pressure_fault;
    Fault regulator_fault;
    Fault water_cooler_fault;
};

#endif /* INTERFACE_PARAMETER_TL_CRAFT_LASER_CUTTING_PARAMETER_H_ */
