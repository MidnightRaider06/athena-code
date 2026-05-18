# athena_drive

This repository contains all packages for the drive subteam for UMD Loop's 2024-25 rover, Athena.

-----

## Guidelines
* Fork this repository for development, you can open a pull request to this repo to add your changes for everyone.
* Do NOT force push.
* Follow best practices (descriptive commit messages, well documented code, etc.)

## Versions
This code is written for an Ubuntu 22.04 system with ROS2 Humble Hawksbill and Gazebo Harmonic.

## Building

Note that to build this repository, you must have a [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). You can clone this repository into the src/ folder.

### Installing Dependencies

Inside of your workspace, run:
```bash
rosdep install --from-paths src -y --ignore-src
```
This will search through the `package.xml` files and install the necessary packages. Read more about rosdep and how to properly configure your packages [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html).

### Building

Inside of your workspace, run:
```bash
colcon build --packages-select  <name-of-pkg>
```

You can also do ``colcon build`` to build all packages.

After building, source your install with:
```bash
source install/setup.bash
```

<b>Velocity Testing, data is in rad/s:</b>
```
ros2 control switch_controllers --deactivate ackermann_steering_controller --deactivate drive_position_controller
ros2 control switch_controllers --activate drive_velocity_controller 
ros2 topic pub /drive_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```


### Launch Modes

`athena_drive.launch.py` accepts a `mode` argument that controls which nodes are started.

By default (`mode:=standalone`), all nodes run on a single machine. For competition, split across two machines using the `mode` argument:

- `standalone` (default): starts all nodes on one machine (control + teleop)
- `jetson`: starts only the control and hardware nodes, to be run on the rover
- `base_station`: starts only the teleop nodes (joystick + teleop_twist_joy), to be run on the base station

```bash
ros2 launch drive_bringup athena_drive.launch.py
```

```bash
ros2 launch drive_bringup athena_drive.launch.py mode:=jetson
```

```bash
ros2 launch drive_bringup athena_drive.launch.py mode:=base_station
```

---

### How to use Control Switching
**Open another terminal, source the workspace, and call the service to set controllers:**
```bash
source install/setup.bash
ros2 service call /set_controller athena_drive_msgs/srv/SetController "{controller_names: [INCLUDE CONTROLLER(S) YOU WANT WITHIN BRACKETS]}"
```
Example: `ros2 service call /set_controller athena_drive_msgs/srv/SetController "{controller_names: [drive_velocity_controller]}"`

ros2 topic pub --once /power_module_gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues "
interface_groups: ['drive_power_module']
interface_values: 
- interface_names: ['kill_jetson']
  values: [1.0] 
"