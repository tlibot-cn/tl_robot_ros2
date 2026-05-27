#!/bin/bash

echo "Publishing JobInsertMoveL command..."

ros2 topic pub --once /tl_driver/job_insert_moveL tl_ros2_interface/msg/JobInsertMove "{
  line: 20,
  cmd: {
    target_pos_value: [
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  }
}"

echo "JobInsertMoveL command published."