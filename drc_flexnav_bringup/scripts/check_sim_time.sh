#!/bin/bash

nodes=$(ros2 param list | awk '/^[^ ]/ {node=$1} /use_sim_time/ {print node} ' | awk -F':' '{print $1}')

for node in $nodes; do
      value=$(ros2 param get "$node" use_sim_time | awk '{print $4}')
      echo "Node: $node, use_sim_time: $value"
done
