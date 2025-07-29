_Literraly as the name implies_

Launching Gazebo (Ign Gz): ros2 launch control_drive gazebo.launch.py

Few commands:

- ros2 topic pub /control_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

- ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/control_drive_controller/cmd_vel_unstamped
