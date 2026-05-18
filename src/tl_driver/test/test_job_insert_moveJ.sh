#!/bin/bash

echo "Publishing JobInsertMoveJ command..."

ros2 topic pub --once /tl_driver/job_insert_moveJ tl_ros2_interface/msg/JobInsertMove "{
  line: 1,
  cmd: {
    target_pos_value: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ],
    target_pos_name: '',
    target_pos_type: 0,
    coord: 0,
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

echo "JobInsertMoveJ command published."