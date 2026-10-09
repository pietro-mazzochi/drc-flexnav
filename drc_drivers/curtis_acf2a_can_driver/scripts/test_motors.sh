#!/bin/bash
ros2 topic pub --once /right_motor/throttle std_msgs/msg/Float32 "{data: $1}" &
ros2 topic pub --once /left_motor/throttle std_msgs/msg/Float32 "{data: $2}" &
wait