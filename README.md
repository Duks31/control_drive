# Control Drive — Differential Drive Robot

https://github.com/user-attachments/assets/bfa14671-e47c-4b4c-9c9c-9bed5e9809de

## Requirements
- ROS2 Humble
- Ignition Fortress (Gazebo)
- Python 3.10+

---

## Building
```bash
colcon build && source install/setup.bash
```

---

## Launching the Simulation
```bash
ros2 launch control_drive gazebo.launch.py
```

## Launching RViz
```bash
ros2 launch control_drive display.launch.py
```
> Change **Fixed Frame** in Global Options to `odom`

---

## Control Modes

This package uses a modular control architecture. All input sources publish
to `/cmd_vel` which is relayed to the robot via a `cmd_vel_relay` node.
To add a new input source, inherit from `RobotController` and implement `run()`.

```
[keyboard / gamepad / script]
            ↓
         /cmd_vel
            ↓
       [cmd_vel_relay]
            ↓
/control_drive_controller/cmd_vel_unstamped
            ↓
    [diff_drive_controller]
            ↓
       [Gazebo Robot]
```

---

### Keyboard Controller
Full keyboard teleoperation with speed presets and emergency stop.

```bash
ros2 run control_drive keyboard_controller
```

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Turn left |
| `d` | Turn right |
| `q/e` | Forward + left/right |
| `z/c` | Backward + left/right |
| `+/-` | Increase/decrease speed by 10% |
| `1/2/3` | Speed presets (slow/medium/fast) |
| `SPACE` | Emergency stop |
| `CTRL+C` | Quit |

---

### Gamepad Controller
Works with any HID gamepad — PS4, PS5, Xbox, or generic controllers.
Auto-detects the connected device and runs a one-time calibration to map axes.

```bash
ros2 run control_drive gamepad_controller
```

| Input | Action |
|-------|--------|
| Forward/backward stick | Linear speed |
| Turning stick | Angular speed |
| R2 / RT | Increase speed (up to 2x) |
| L2 / LT | Decrease speed (down to 0.3x) |
| L2 + R2 together | Emergency stop |
| OPTIONS / START | Quit |

**First run:** calibration wizard runs automatically and saves mapping to
`~/.ros/gamepad_mapping.json`. Subsequent runs load the saved mapping instantly.

**To recalibrate:**
```bash
rm ~/.ros/gamepad_mapping.json
ros2 run control_drive gamepad_controller
```

**Launch file (handles permissions + starts node):**
```bash
ros2 launch control_drive gamepad.launch.py
```

> **WSL2 users:** Run this first in PowerShell (Admin) before launching:
> ```powershell
> usbipd attach --wsl --busid <your-busid>
> ```
> Find your busid with `usbipd list`.

---

### Script Controller
Write autonomous robot behaviors directly in Python.
No keyboard or gamepad needed — the robot executes your script automatically.

```bash
ros2 run control_drive script_controller
```

Edit `control_drive/script_controller.py` and write your behavior inside `run()`:

```python
def run(self):
    self.move_forward(speed=0.5, duration=2.0)
    self.turn_left(speed=1.0, duration=1.57)   # ~90 degrees
    self.stop()
```

Available helpers:

| Method | Description |
|--------|-------------|
| `move_forward(speed, duration)` | Move forward for N seconds |
| `move_backward(speed, duration)` | Move backward for N seconds |
| `turn_left(speed, duration)` | Turn left for N seconds |
| `turn_right(speed, duration)` | Turn right for N seconds |
| `pause(duration)` | Stop and wait for N seconds |
| `send_command(linear, angular)` | Direct velocity command |
| `stop()` | Immediate stop |

---

## Architecture

| File | Purpose |
|------|---------|
| `robot_controller.py` | Base class — publisher, safety limits, speed helpers |
| `keyboard_controller.py` | Keyboard input source |
| `gamepad_controller.py` | Generic gamepad input source with auto-calibration |
| `script_controller.py` | Script-based autonomous control |
| `cmd_vel_relay.py` | Relays `/cmd_vel` → `/control_drive_controller/cmd_vel_unstamped` |

---

## Manual Testing

Test velocity directly without any controller:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Or using the original unstamped topic directly:
```bash
ros2 topic pub /control_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```