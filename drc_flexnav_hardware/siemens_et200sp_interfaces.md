# Controllers and Broadcasters:

## OdometryBroadcaster:

### state:

- pose.position.x
- pose.position.y
- pose.position.z
- pose.orientation.x
- pose.orientation.y
- pose.orientation.z
- pose.orientation.w
- twist.linear.x
- twist.linear.y
- twist.linear.z
- twist.angular.x
- twist.angular.y
- twist.angular.z

### state:

- time_stamp
- delta_t
- plc_connected
- last_plc_read_write_time

## RobotSignalingController:

### command:

- green_front_light
- green_rear_light
- red_front_light
- red_rear_light
- left_light
- right_light
- buzzer

### state:

- time_stamp
- delta_t
- plc_connected
- last_plc_read_write_time

## RobotLoadController:

### command:

- load_unload_timeout_seconds
- load_command

### state:

- time_stamp
- delta_t
- table_error
- load_error
- unload_error
- table_front_load_sensor
- table_rear_load_sensor
- loaded
- plc_connected
- last_plc_read_write_time

## LaserScannerController

### command:

- ls_config

### state:

- time_stamp
- delta_t
- is_emergency_lock
- plc_connected
- last_plc_read_write_time

## SiemensET200SPBroadcaster

Publishes all state interfaces in a Siemenset200spmodbusdriver.msg

### state:

- time_stamp
- delta_t
- left_encoder
- right_encoder
- left_encoder_rad
- right_encoder_rad
- left_encoder_rpm
- right_encoder_rpm
- joy_up
- joy_down
- joy_left
- joy_right
- docked_front
- docked_rear
- charging
- front_ls_violation
- rear_ls_violation
- reset_wait
- undefined_error
- restart_detected
- left_motor_error
- right_motor_error
- table_error
- load_error
- unload_error
- battery_error
- docking_error
- auto_sw
- man_sw
- reset_sw
- table_front_load_sensor
- table_rear_load_sensor
- joy_connected
- is_emergency_lock
- loaded
- communication_faults
- plc_connected
- last_plc_read_write_time

---

## All command interfaces :

- green_front_light
- green_rear_light
- red_front_light
- red_rear_light
- left_light
- right_light
- buzzer
- load_unload_timeout_seconds
- load_command
- ls_config

## All state interfaces:

- time_stamp
- delta_t
- left_encoder
- right_encoder
- left_encoder_rad
- right_encoder_rad
- left_encoder_rpm
- right_encoder_rpm
- joy_up
- joy_down
- joy_left
- joy_right
- docked_front
- docked_rear
- charging
- front_ls_violation
- rear_ls_violation
- reset_wait
- undefined_error
- restart_detected
- left_motor_error
- right_motor_error
- table_error
- load_error
- unload_error
- battery_error
- docking_error
- auto_sw
- man_sw
- reset_sw
- table_front_load_sensor
- table_rear_load_sensor
- joy_connected
- is_emergency_lock
- loaded
- communication_faults
- plc_connected
- last_plc_read_write_time
