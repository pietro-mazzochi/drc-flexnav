# Frequency Check Lib

Provides a ROS 2 node that monitors the publication rate of various sensors. It subscribes to standard topics (such as scans, lidar, IMU, magnetometer, and joint_states) and publishes a message containing the status of each sensor according to the configured rates. If any sensor does not meet the minimum limits or exceeds the maximum limits, the node generates an alert.

## Compilation

The library consists of two packages:
- **frequency_check** – Code for the node that performs subscriptions and frequency checks.
- **frequency_check_msg** – Defines the custom message (FrequencyCheck.msg) that contains the publication status for each sensor.

### Requirements
- ROS 2 (Humble/Foxy or later)
- Colcon build and ament_cmake

### Compilation Instructions

1. Create a workspace (if you don't have one already):

   ```sh
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone or copy the package directories (frequency_check and frequency_check_msg) into the src folder of the workspace.

3. Return to the root of the workspace:

   ```sh
   cd ~/ros2_ws
   ```

4. Compile the workspace with colcon:

   ```sh
   colcon build --symlink-install
   ```

5. Source the setup to use the packages:

   ```sh
   source install/setup.bash
   ```

## Configuration

The node configuration is done through a YAML file located in the `config` directory of the frequency_check package. In it, you can define:

- The topic names for each sensor (e.g., "/rear_scan", "/imu_sensor/imu", "/joint_states", etc.).
- The window size (`window_size`) used for frequency calculation.
- A timeout (`timeout_ms`) indicating how long can pass without messages before the sensor is considered inactive.
- The list of sensors to be monitored (`sensors.names`), the frequency limits for each (`sensors.min_frequency` and `sensors.max_frequency`), and the effectively active sensors (`sensors.active_sensors`).

### Example Configuration File (config.yaml)

```yaml
frequency_check:
  ros__parameters:
    rear_scan_topic: "/rear_scan"
    rear_lidar_topic: "/rear_lidar"
    front_scan_topic: "/front_scan"
    front_lidar_topic: "/front_lidar"
    magnetometer_topic: "/imu_sensor/magnetic_field"
    imu_topic: "/imu_sensor/imu"
    joint_states_topic: "/joint_states"
    window_size: 100
    timeout_ms: 3000
    sensors:
      names: ["imu", "magnetometer", "front_lidar", "front_scan", "rear_lidar", "rear_scan", "joint_states"]
      min_frequency: [10.0, 10.0, 4.0, 4.0, 4.0, 4.0, 5.0]
      max_frequency: [120.0, 120.0, 10.0, 10.0, 10.0, 10.0, 50.0]
      active_sensors: ["imu", "magnetometer", "front_lidar", "rear_lidar", "joint_states"]
```

Make the desired adaptations according to your scenario.

## Semantics

The node monitors the publication frequency of sensors based on the following premises:

- Each time a message is received from a sensor, a timestamp is recorded.
- A window with a fixed set of timestamps (defined by `window_size`) is maintained for each sensor.
- Regularly (every 3 seconds, configured via an internal timer), the sensor frequency is calculated using the elapsed time between the first and last message in that window.
- If the elapsed time exceeds `timeout_ms`, the sensor is considered "offline" or inactive, and the status is considered a failure.
- If the calculated frequency is outside the defined range (between `min_frequency` and `max_frequency`) for an active sensor, the node will trigger an alert and update the respective field for that sensor in the `FrequencyCheck` message.

The `FrequencyCheck` message has the following fields:
- `alert`: indicates that some sensor has a frequency outside the limits.
- `imu_publishing`: status of the IMU sensor.
- `mag_publishing`: status of the magnetometer sensor.
- `front_lidar_publishing`: status of the front lidar sensor.
- `front_scan_publishing`: status of the front scan sensor.
- `rear_lidar_publishing`: status of the rear lidar sensor.
- `rear_scan_publishing`: status of the rear scan sensor.
- `joint_states_publishing`: status of the joint_states topic.
- `frequency_error`: string listing the sensors that are out of range or with timeout.

## Execution

To run the node, use the launch file:

```sh
ros2 launch frequency_check frequency_check.launch.py
```

The launch file uses the YAML configuration file to define the parameters at both the node and sensor levels. If necessary, you can pass other arguments (such as enabling sim time or log level) via the command line.

## Final Considerations

- Ensure that the topics configured in the YAML file correspond to the topics used by the sensors in your environment.
- In case of data inadequacy (e.g., very small window or short timeouts), adjust the `window_size` and `timeout_ms` parameters according to the expected behavior of the sensors.
