# Athena Code

This repository contains all the code for UMDLoop's 2025-26 rover, Athena.

When cloning, be sure to use `git clone --recursive [URL to Git repository]` to pull in the submodules found in `src/third-party`. Alternatively, run `git submodule update --init --recursive` if you have already cloned.

## How To Use

### Hardware Setup

- Ensure CAN Bus is connected and hardware is configured to the correct IDs
- Verify that each of the ros2_control xacro files (in `src/description/ros2_control/`) have the correct IDs inside of the `node_id` parameter for each joint

### CAN Setup

_Hardware:_
```bash
./src/athena-code/src/tools/scripts/can_setup.sh
```

_Virtual:_
```bash
./src/athena-code/src/tools/scripts/virtual_can_setup.sh
```

Use `ip link` to verify that `can0` or `vcan0` is up.

### Building

1. Install required dependencies:
```bash
rosdep install --from-paths src -y --ignore-src
```

2. Build the workspace:
```bash
colcon build --symlink-install
```

3. Source the workspace:
```bash
source install/setup.bash
```

---

## Controller Switching

All subsystems use a controller switching service. Open a new terminal, source the workspace, and call:

```bash
source install/setup.bash
ros2 service call /set_controller msgs/srv/SetController "{controller_names: [CONTROLLER_NAME]}"
```

Only one motion controller should be active at a time (alongside the `joint_state_broadcaster`).

---

## Testing

### Arm

#### Launch

_Hardware:_
```bash
ros2 launch arm_bringup athena_arm.launch.py use_mock_hardware:=false
```

_Mock (no hardware):_
```bash
ros2 launch arm_bringup athena_arm.launch.py use_mock_hardware:=true
```

#### PS4 Controller

- **Base Yaw**: Left/Right on Left Joystick (Axes 0)
- **Shoulder Pitch**: Up/Down on Left Joystick (Axes 1)
- **Elbow Pitch**: Up/Down on Right Joystick (Axes 3)
- **Wrist Pitch**: Up/Down on Left Joystick AND O button (Axes 1)
- **Wrist Roll**: Left/Right on Left Joystick AND O button (Axes 0)
- **Open Claw (max speed)**: Left Bumper (Button 5)
- **Close Claw (max speed)**: Right Bumper (Button 4)
- **Open Claw (speed control)**: Left Trigger (Axes 5)
- **Close Claw (speed control)**: Right Trigger (Axes 4)

#### Velocity Testing

1. Switch to velocity controller using the controller switching service
2. In a new terminal:
```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### Joint Trajectory Testing

1. Switch to joint trajectory controller using the controller switching service
2. In a new terminal:
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

---

### Drive

#### Launch

_Hardware:_
```bash
ros2 launch drive_bringup athena_drive.launch.py use_mock_hardware:=false
```

_Mock (no hardware):_
```bash
ros2 launch drive_bringup athena_drive.launch.py use_mock_hardware:=true
```

The default controller is `single_ackermann_controller`. To use the built-in ackermann steering controller instead:
```bash
ros2 launch drive_bringup athena_drive.launch.py robot_controller:=ackermann_steering_controller
```

---

### Science

#### Launch

_Hardware:_
```bash
ros2 launch science_bringup athena_science.launch.py use_mock_hardware:=false
```

_Mock (no hardware):_
```bash
ros2 launch science_bringup athena_science.launch.py use_mock_hardware:=true
```

---

## How To Contribute

### Learning Git

If you're new to Git and GitHub, start with this beginner-friendly tutorial: [GitHub's Hello World Guide](https://docs.github.com/en/get-started/quickstart/hello-world)

For a more comprehensive introduction, check out: [Git and GitHub Tutorial for Beginners](https://www.freecodecamp.org/news/git-and-github-for-beginners/)

### Setting up Git

Before you can contribute, you'll need to have Git installed and configured on your machine:

1. **Install Git** - Follow GitHub's guide to [set up Git](https://docs.github.com/en/get-started/git-basics/set-up-git) on your system
2. **Configure Git** - Set your username and email that will be associated with your commits

### SSH Keys

To securely authenticate with GitHub without entering your password each time, set up SSH keys:

1. **Check for existing SSH keys** - See if you already have SSH keys: [Checking for existing SSH keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys)
2. **Generate a new SSH key** - If needed, create a new SSH key pair: [Generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
3. **Add SSH key to GitHub** - Upload your public key to your GitHub account: [Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

### Making a Pull Request (PR)

Once you're set up, here's how to contribute your changes:

1. **Fork the repository** - Create your own copy of the repository: [Fork a repo](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo)

2. **Clone your fork** - Download your forked repository to your local machine using `git clone`.

3. **Create a feature branch** - Make a new branch for your specific feature or fix:
```bash
git checkout -b feature/your-feature-name
```

4. **Make your changes** - Write your code, test it thoroughly, and commit your changes

5. **Push to your fork** - Upload your feature branch to your GitHub fork
```bash
git push origin feature/your-feature-name
```

6. **Open a Pull Request** - Submit your changes for review: [Creating a pull request from a fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork)

Your PR will be reviewed by your lead, and once approved, it will be merged into the main codebase!
