# Navigation

## Running In Simulation

```bash
ros2 launch simulation bringup.launch.py publish_ground_truth_tf:=true rviz:=true
ros2 launch athena_planner navigation.launch.py sim:=true
```