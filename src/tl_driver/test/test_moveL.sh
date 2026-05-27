#!/bin/bash

echo "Publishing MoveL command..."

ros2 topic pub --once /tl_driver/moveL tl_ros2_interface/msg/MoveCommand "{
  target_pos_value: [
    10.0, 230.0, 245.0, -3.14, 0.0, -1.57, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  ],
  target_pos_name: '',
  target_pos_type: 0,
  coord: 1,
  velocity: 20.0,
  velocity_sync: 0.0,
  acc: 20.0,
  dec: 20.0,
  pl: 0,
  time: 0,
  tool_num: 0,
  user_num: 0,
  posidtype: 0,
  configuration: 0,
  spin: 0,
  para_sync: false
}"

echo "MoveL command published."