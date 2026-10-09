# DRC - Simulation

## Guide

- [Installing ROS2](#installing-ros-2)
- [Building WorkSpace](#building-workspace)
- [Gazebo Installation](#gazebo-installation)
- [How to run](#how-to-run)

## Installing ROS 2

We use the ROS 2 Humble, for the installation follow the steps in the [ROS 2 Humble Ubuntu Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Instal the desktop full and dev-tools version.

```bash
sudo apt install ros-humble-desktop-full
sudo apt install ros-dev-tools
sudo apt install libeigen3-dev
sudo apt install libmodbus-dev
```

Make sure that the default WS is working executing the example steps.

## Building WorkSpace

Source the underlay:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Create the workspace folder and its source folder

```bash
mkdir -p ~/ros_ws/src
```

Make sure the dependencies are installed

```bash
sudo apt install cmake
sudo apt install build-essential
sudo apt install libeigen3-dev
```

Clone the repository inside the `src` folder

```bash
cd ~/ros_ws/src
git clone https://github.com/isisim/drc-simulation.git
git clone https://github.com/isisim/drc-flexnav1500.git
git clone https://github.com/isisim/drc-sandbox.git
git clone https://github.com/isisim/Livox-SDK2.git livox_sdk2
git clone https://github.com/isisim/livox_laser_simulation_RO2.git livox_laser_simulation_ros2
git clone https://github.com/isisim/livox_ros_driver2.git livox_ros_driver2
git clone https://github.com/isisim/ros2-control-toolbox
```

🚩 Necessary simulation files for build

The necessary files are located in the Sharepoint.
To make the built system work, it is necessary to download the [meshes](https://sesirs.sharepoint.com/:f:/r/sites/DTI-ProjetoDRC/Documentos%20Compartilhados/General/07_DOCUMENTOS_TECNICOS/03_SIMULA%C3%87%C3%83O/2_ARQUIVO_SIMULACAO/meshes?csf=1&web=1&e=IIbqeo) folder and place it in the root folder of the `drc_simulation` package.

You'll also have to download the [meshes](https://sesirs.sharepoint.com/:f:/r/sites/DTI-ProjetoDRC/Documentos%20Compartilhados/General/07_DOCUMENTOS_TECNICOS/03_SIMULA%C3%87%C3%83O/2_ARQUIVO_SIMULACAO/robot_models?csf=1&web=1&e=F3b8FU) for the robot and place it in the `drc_flexnav_description` package, inside the folder `meshes/<robot_name>`.

If it is the first time that `rosdep` tool is being used, run:

```bash
sudo rosdep init
rosdep update
```

Inside the workspace folder, install the ROS2 dependencies, build the packages in the workspace with colcon and source the overlay

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source ~/ros_ws/install/setup.bash
```

## How To Run

Build the workspace and source the overlay

```bash
cd ~/ros_ws
colcon build --symlink-install
source ~/ros_ws/install/setup.bash
```

Launch the robot and world with the launch files `.launch.xml` from `drc_simulation` or `drc_flexnav_bringup package.

**To launch only a robot, spawned on a Gazebo environment, use the following command**:

```bash
ros2 launch drc_simulation gazebo.launch.xml
```

**To launch the robot, spawned on a Gazebo environment, with the controllers use the following command**:

```bash
ros2 launch drc_flexnav_bringup simulation_controller.launch.xml
```

## Known Issues

### 🚩Unable to find shader lib:

```bash
[gzclient-2] [Err] [RTShaderSystem.cc:480] Unable to find shader lib. Shader generating will fail. Your GAZEBO_RESOURCE_PATH is probably improperly set. Have you sourced <prefix>/share/gazebo/setup.bash?
[gzclient-2] [Wrn] [GuiIface.cc:120] QStandardPaths: wrong permissions on runtime directory /run/user/1000/, 0755 instead of 0700
```

To solve the issue:

```bash
echo export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11 >> ~/.bashrc
source ~/.bashrc
```
