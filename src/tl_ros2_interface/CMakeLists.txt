cmake_minimum_required(VERSION 3.8)
project(tl_ros2_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  
find_package(std_msgs REQUIRED)      
find_package(geometry_msgs REQUIRED)      

# 添加消息和动作定义
rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/ArmStatus.msg"                       
  "msg/MoveCommand.msg"                    
  "msg/CartesianPose.msg"
  "msg/ToolParam.msg"
  "msg/ModbusTCPParam.msg"
  "msg/ModbusRTUParam.msg"
  "msg/ModbusMasterParam.msg"
  "msg/RobotDHParam.msg"
  "msg/JobFileName.msg"
  "msg/RobotJointParam.msg"
  "msg/ObjectInfo.msg"

  #srv
  "srv/SetSpeed.srv"
  "srv/GetSpeed.srv"
  "srv/Jogging.srv"
  "srv/SetDragMode.srv"
  "srv/TrackSave.srv"
  "srv/TrackPlayback.srv"
  "srv/SetToolParam.srv"
  "srv/SetUserCoord.srv"
  "srv/SetAxisZeroPos.srv"
  "srv/SetCurrentCoord.srv"
  "srv/GetCurrentCoord.srv"
  "srv/SetCoordNum.srv"
  "srv/GetCoordNum.srv"
  "srv/ToolHandCalib.srv"
  "srv/SetDigitalOutput.srv"
  "srv/GetDigitalInputOutput.srv"
  "srv/ModbusWrite.srv"
  "srv/ModbusRead.srv"
  "srv/CoordTransform.srv"
  "srv/GetPosReachable.srv"
  "srv/SetDHParam.srv"
  "srv/GetDHParam.srv"
  "srv/GetAllJobFileName.srv"
  "srv/JobRun.srv"
  "srv/JobInsertMove.srv"
  "srv/SetGlobalPos.srv"
  "srv/GetGlobalPos.srv"
  "srv/OpenServoJ.srv"
  "srv/SetCurrentMode.srv"
  "srv/GetCurrentMode.srv"
  "srv/QueueMotionSetStatus.srv"
  "srv/QueueMotionMoveJ.srv"
  "srv/GetRobotState.srv"
  "srv/GetRobotJointParam.srv"
  "srv/SetRobotJointParam.srv"
  "srv/GetJointTemperature.srv"
  "srv/GetJointVoltage.srv"
  "srv/GetMotorCurrent.srv"
  "srv/GetJointSoftwareVersion.srv"
  "srv/RestoreDefaultDHParam.srv"
  "srv/LogDownload.srv"
  "srv/GetPosTransform.srv"
  "srv/SetControllerIP.srv"
  "srv/GetCurrentMotorTorque.srv"
  "srv/GetCurrentLineJointSpeed.srv"

  DEPENDENCIES 
    action_msgs
    builtin_interfaces        
    std_msgs             
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
