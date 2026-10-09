
# IMU LPMS_CU3 Driver

Driver library to work with lpresearch [LPMS-cu3 IMU](https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1941864458/LPMS3+Command+List) and [premium IMU](./docs/premium_imu.md) (experimental).


The published topics are **imu/data** (sensor_msgs::msg::Imu) and **magnetometer/data** (sensor_msgs::msg::MagneticField)

## How to use

Compile the package with `colcon build --symlink-install`

Set permissions to _/dev/ttyUSB_ then execute the node with launch command:

```bash
ros2 launch imu_lpms_cu3 imu_lpms_cu3.launch.xml
```

or run as hardware interface using the `drc_flexnav_description` package.

