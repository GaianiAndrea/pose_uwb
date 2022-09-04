# Pose_UWB

Master Thesis project

## Pre-requisites

### ROS2
Install a current version of ROS2 (foxy or galactic), following the [official instructions](https://docs.ros.org/en/galactic/Installation.html).

### Robomaster ROS
Install [this repository](https://github.com/jeguzzi/robomaster_ros) that contains a ROS2 driver for the DJI Robomaster family of robots (EP and S1). 

### Robomaster Sim
Install [this repository](https://github.com/jeguzzi/robomaster_sim) that contains a library that emulates the firmware and remote API server of DJI Robomaster S1 and DJI Robomaster EP robots. They also implement a plugin for CoppeliaSim that simulates most of the functionality of the real robots.

### CoppeliaSim
To compile and then use the CoppeliaSim [CoppeliaSim](https://www.coppeliarobotics.com) plugin.
Download the latest release. Export the location where you place it.

```bash
export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
```
which on Linux is the root folder that you download, while on MacOs is `/Applications/coppeliaSim.app/Contents/Resources`, if you install the app to the default location.

### Required ROS packages:

Clone the following packages in `{path-to-ros-workspace}/src/`:

#### simExtROS2 - branch galactic

```
git clone -b galactic https://github.com/jeguzzi/simExtROS2.git
```

#### teleop_tools - branch dashing-devel

```
git clone -b dashing-devel https://github.com/jeguzzi/teleop_tools.git
```

#### volaly_kinematics - branch ros2

```
git clone -b ros2 https://github.com/Gabry993/volaly_kinematics.git
```

#### volaly_msgs - branch ros2

```
git clone -b ros2 https://github.com/Gabry993/volaly_msgs.git
```

#### oneswarm-hri

```
git clone https://github.com/idsia-robotics/oneswarm-hri.git
```

#### manipulation_robomaster_utils - branch uwb_Andrea

```
git clone -b uwb_Andrea https://github.com/jeguzzi/manipulation_robomaster_utils.git
```


## Usage
Use one of the three launch files {wrist_uwb|rm_uwb|uwb}.launch to launch the scenario and the robot .

### Launch files
 - Use wrist_uwb.launch to launch the wrist localization and the scenario to handle LED strip
 - Use rm_uwb.launch to launch the RoboMasster localization
 - Use uwb.launch to launch both previous launches

### Launch files
For the scenario to interact with LED strip use:
```bash
ros2 launch pose_uwb wrist_uwb.launch strips:=True
``` 

To localize and use the robot use rm_uwb launch. 
```bash
ros2 launch pose_uwb rm_uwb.launch
```

For the scenario to use pointing-based interaction with robot use:
```bash
ros2 launch pose_uwb uwb.launch
```

#### Arguments
The launch files can accept a list of arguments:

```bash
ros2 launch pose_uwb {single_robot|multirobot}.launch <key_1>:=<value_1> <key_2>:=<value_2> ...
```

### Wrist_uwb configuration
| key              | type    | valid values              | default | description                                                                                           |
| ---------------- | ------- | ------------------------- | ------- | ----------------------------------------------------------------------------------------------------- |
| user_name        | string  | valid ROS name            | user1   | a name used as ROS namespace                                                                          |
| map_path         | string  |                           | map/lab.yaml      | Path to the .yaml map file.                                               |
| strips           | bool    |                           | False   | Define if we are using the LED strip, and then we launch also [oneswarm-hri](https://github.com/idsia-robotics/oneswarm-hri) nodes.|
| pointer_cmap     | string  | valid matplotlib colormap | ' '     | If a valid matplotlib colormap name is passed, the pointer color is set as a function of the ray-target distance and the colormap. |
| sim_imu          | bool    |                           | False   | This must be True if we want to run just in simulation with Bill's IMU. False otherwise.              |
| user_kinematics  | string  |                           | bill    | Name of the file containing the user kinematics.                                                      |
| single_LED       | bool    |                           | False   | If True, the pointing cursor will interact only with single LEDs. If False, the pointing cursor will interact with LEDs strips instead.|
| imu_topic        | bool    |                           | False   | If True, we expect IMU-type message from user/imu topics. Otherwise, we expect a Quaternion-type message |
| filter           | bool    |                           | False   | If True, we launch the filter node. Otherwise not. |
| ranges_topic     | string  | valid ROS name            | ranges  | a name used as ROS namespace. Define the name of ranges topic. |


### Rm_uwb configuration
| key              | type    | valid values              | default | description                                                                                           |
| ---------------- | ------- | ------------------------- | ------- | ----------------------------------------------------------------------------------------------------- |
| rm_name          | string  | valid ROS name            | rm1     | a name used as ROS namespace. Name of the robot.                                                      |
| model            | string  |                           | s1      | Model of the robot. |
| map_path         | string  |                           | map/lab.yaml      | Path to the .yaml map file.                                               |
| filter           | bool    |                           | False   | If True, we launch the filter node. Otherwise not. |
| ranges_topic     | string  | valid ROS name            | ranges  | a name used as ROS namespace. Define the name of ranges topic. |

### Uwb configuration
uwb.launch calls the previous two launches, so we can use their configurations.