## Control Drive Differential Drive Bot

https://github.com/user-attachments/assets/0ca53fca-9ba5-4698-ad27-b10539069632

#### Requirements:
- ROS2 Humble
- Gazebo (Ign) 

Building
```bash
colcon build
```

### Launching Gazebo (Ign Gz): 
```bash
ros2 launch control_drive gazebo.launch.py
```

### Launching rviz 

```bash
 ros2 launch control_drive display.launch.py
``` 
Change Fixed Frame in Global options to odom

### Test linear velocity publisher

``` bash
ros2 topic pub /control_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"\
```

### Run with teleop twist keyboard 

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/control_drive_controller/cmd_vel_unstamped
```

#### custom teleop control

Custom control script in `control_drive/custom_teleop.py`

```bash
ros2 run control_drive custom_teleop --ros-args -r /cmd_vel:=/control_drive_controller/cmd_vel_unstamped
```

