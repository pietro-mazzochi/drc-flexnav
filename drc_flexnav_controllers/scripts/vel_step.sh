#!/bin/bash

ros2 topic pub -1 /mrac_controller/command geometry_msgs/Twist "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}"
# ros2 topic pub -1 /dynamics_controller/command geometry_msgs/Twist "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}"
# ros2 topic pub -1 /base_vel_cmd geometry_msgs/Twist "{linear: {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}"
