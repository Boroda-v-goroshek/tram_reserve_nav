# Hackathon solution checker

ROS 2 node that compares two solution streams independently with reference
odometry:

- `/result/velocity` (`tram_vehicle_msgs/msg/VelocitySensor`) against
  `/localization/kinematic_state.twist.twist.linear.x`;
- `/result/position` (`nav_msgs/msg/Odometry`) against
  `/localization/kinematic_state.pose.pose.position`.

The node prints RMSE and maximum absolute error every five seconds and once
again when it is stopped. Position output contains errors for each axis and the
3D Euclidean distance.

```bash
colcon build --packages-select tram_vehicle_msgs hackathon_solution_checker
source install/setup.bash
ros2 run hackathon_solution_checker metrics
```

Topics and synchronization can be changed through ROS parameters:

```bash
ros2 run hackathon_solution_checker metrics --ros-args \
  -p reference_topic:=/localization/kinematic_state \
  -p velocity_topic:=/result/velocity \
  -p position_topic:=/result/position \
  -p sync_tolerance_sec:=0.05
```

If `VelocitySensor` stores its scalar value in a field other than `velocity`,
set its dot-separated path, for example:

```bash
ros2 run hackathon_solution_checker metrics --ros-args \
  -p result_velocity_field:=velocity_mps
```
