cmake_minimum_required(VERSION 3.16)
project(tl_ros2_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)      
find_package(geometry_msgs REQUIRED)      

# 添加消息和动作定义
rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/MoveCommand.msg"                    
  "msg/CartesianPose.msg"
  "msg/ToolParam.msg"
  "msg/ModbusTCPParam.msg"
  "msg/ModbusRTUParam.msg"
  "msg/ModbusMasterParam.msg"
  "msg/RobotDHParam.msg"
  "msg/RobotJointParam.msg"
  "msg/JointMonitor.msg"
  "msg/JointSpeed.msg"
  "msg/JointTorque.msg"
  "msg/LineSpeed.msg"
  "msg/ErrorInfo.msg"

  #action
  "action/Move.action"
  "action/PlayTrack.action"

  #srv
  "srv/SetSpeed.srv"
  "srv/GetSpeed.srv"
  "srv/GetArmStatus.srv"
  "srv/Fk.srv"
  "srv/Ik.srv"
  "srv/Jogging.srv"
  "srv/SetDragMode.srv"
  "srv/SetTeachType.srv"
  "srv/GetTeachType.srv"
  "srv/TrackSave.srv"
  "srv/DragTrajectoryPlayback.srv"
  "srv/DragTrajectoryDelete.srv"
  "srv/SetToolParam.srv"
  "srv/SetUserCoord.srv"
  "srv/SetCurrentCoord.srv"
  "srv/GetCurrentCoord.srv"
  "srv/SetCoordNum.srv"
  "srv/GetCoordNum.srv"
  "srv/SetDigitalOutput.srv"
  "srv/GetDigitalInputOutput.srv"
  "srv/ModbusWrite.srv"
  "srv/ModbusRead.srv"
  "srv/CoordTransform.srv"
  "srv/GetPosReachable.srv"
  "srv/GetDHParam.srv"
  "srv/OpenServoJ.srv"
  "srv/OpenServoCst.srv"
  "srv/SetCurrentMode.srv"
  "srv/GetCurrentMode.srv"
  "srv/QueueMotionSetStatus.srv"
  "srv/QueueMotionMoveJ.srv"
  "srv/GetRobotJointParam.srv"
  "srv/SetControllerIP.srv"

  DEPENDENCIES 
    action_msgs
    std_msgs             
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
