# ROS2 workspace for UMD Loop's Arm software integration

## Current package list in arm-control:

<ul>
  <li><b>actuator_ros2_control</b>: contains ros2_control hardware interface for servo motors. Currently Empty.</li>
  <li><b>athena_arm_bringup</b>: contains launch file for ros2_control and additional config files for rviz and controllers</li>
  <li><b>athena_arm_controllers</b>: contains athena arm manual controller</li>
  <li><b>athena_arm_description</b>: contains the full urdf file for the arm including ros2_control urdfs</li>
  <li><b>athena_arm_moveit</b>: contains athena arm moveit setup</li>
  <li><b>athena_arm_msgs</b>: contains messages used for athena mainly for logging</li>
  <li><b>manual_control</b>: testing setup to control each motor on the arm with a PS4 controller. We now have a controller in the ros2_control setup that does the same thing</li>
  <li><b>ros_odrive</b>: contains node and ros2_control hardware interface for ODrives.</li>
  <li><b>smc_ros2_control</b>: contains ros2_control hardware interface for SMC motors</li>
  <li><b>talon_ros2_control</b>: contains ros2_control hardware interface for Pololu motors using Talon motor controller</li>
  <li><b>umdloop_can</b>: contains can_node</li>
  <li><b>umdloop_theseus_can_messages</b>: contains custom messages for can_node</li>
</ul>

## How to use:

### Keep in Mind
- Data should be passed in meters for prismatic joints, radians for revolute
- ENSURE THAT THE AMOUNT OF VALUES PASSED IN DATA MATCHES THE AMOUNT OF INTERFACES AKA JOINTS
- Home position (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) has the arm outstretched parallel to the ground
- Safe position: (0.0 -1.0 -1.8 -0.55 0.0)
- Don't activate multiple controllers at once, you should only have one controller and the joint state broadcaster active at a given time

---

### Hardware Setup:

- Ensure CAN Bus and controller (if using manual control) are connected
- Make sure hardware is configured to the correct ids
  - Manual: go into config/manual_control.yaml, ensure all the ids are correct
  - ros2_control: go into athena_arm_description/ros2_control and check that each of the ros2_control files have the correct ids inside of the parameter "node_id" for the appropriate joint

---

### Software Setup:

#### Workspace setup:
```bash
./submodule_commands.sh
./clean.sh
```

#### Live Hardware setup:
```bash
./can_setup.sh
```

#### Virtual setup:
```bash
./virtual_can_setup.sh
```

Use *ip link* to check that there is a can0/vcan0.

---


### How to use Control Switching

**Open another terminal, source the workspace, and call the service to set controllers:**
```bash
source install/setup.bash
ros2 service call /set_controller msgs/srv/SetController "{controller_names: [INCLUDE CONTROLLER(S) YOU WANT WITHIN BRACKETS]}"
```
Example: `ros2 service call /set_controller msgs/srv/SetController "{controller_names: [joint_trajectory_controller, arm_velocity_controller]}"`

---

### Testing:

#### Manual testing:

From the workspace source, run:
```bash
./manual.sh
```

#### PS4 Controller:

- **Base Yaw**: Left/Right on Left Joystick                 (Axes 0)
- **Shoulder Pitch**: Up/Down on Left Joystick              (Axes 1)
- **Elbow Pitch**: Up/Down on Right Joystick                (Axes 3)

- **Wrist Pitch**: Up/Down on Left Joystick AND O button    (Axes 1)
- **Wrist Roll**: Left/Right on Left Joystick AND O button  (Axes 0)

- **Open Claw (max speed)**: Left Bumper                    (Button 5)
- **Close Claw (max speed)**: Right Bumper                  (Button 4)
- **Open Claw (speed control)**: Left Trigger               (Axes 5)
- **Close Claw (speed control)**: Right Trigger             (Axes 4)

 
#### Thrustmaster (not tested):

- **Base Yaw**: Rotate Stick                                        (Axes 0)
- **Shoulder Pitch**: Pull Back Stick                               (Axes 1)
- **Elbow Pitch**: Pull Back Stick AND press trigger                (Axes 1 and Button 0)

- **Wrist Pitch**: Pull Back Stick AND press top button on stick    (Axes 1)
- **Wrist Roll**: Left/Right on Stick                               (Axes 2)

- **Close Claw**: Left Side, Top Left Button                        (Button 4)
- **Open Claw**: Right Side, Top Middle Button                      (Button 5)



#### ROS2 control testing:

*Hardware*:
```bash
source install/setup.bash
ros2 launch athena_arm_bringup athena_arm_ros2_control.launch.py use_mock_hardware:=False
```

*Mock*:
```bash
source install/setup.bash
ros2 launch athena_arm_bringup athena_arm_ros2_control.launch.py use_mock_hardware:=True
```

#### Velocity Testing:
1. Open a new terminal
2. Switch to velocity controller
```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```


#### Joint Trajectory Testing:
1. Open a new terminal
2. Switch to joint trajectory controller
3. Use the below command
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{ \
  trajectory:{ \
    joint_names: ['base_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper_claw', 'actuator'], \
    points: [ \
      {positions: [0.0, -1.0, -2.0, -1.0, -0.16, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}, \
    ] \
  } \
}"
```

#### Manual Controller Testing

1. Make sure to plug in joystick controller
2. Open a new terminal
3. Switch to desired controller
    - Joint by Joint
    - Cylindrical

---

