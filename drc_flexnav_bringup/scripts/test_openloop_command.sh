#!/bin/bash
ros2 topic pub -t 3 /left_wheel_joint_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [$1]}" &
ros2 topic pub -t 3 /right_wheel_joint_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [$2]}" &
# ros2 topic pub -t 3 /dynamics_controller/command geometry_msgs/msg/Twist "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}" &
wait