import sys
import select
import termios
import tty
import threading

import rclpy
from control_drive.robot_controller import RobotController

# ------------------------------------------------------------------
# Key bindings
# Format: key -> (linear, angular)
# Positive linear  = forward
# Positive angular = turn left (CCW)
# ------------------------------------------------------------------

MOVE_BINDINGS = {
    "w": (1, 0),    # forward
    "s": (-1, 0),   # backward
    "a": (0, 1),    # turn left
    "d": (0, -1),   # turn right
    "q": (1, 1),    # forward + left
    "e": (1, -1),   # forward + right
    "z": (-1, 1),   # backward + left
    "c": (-1, -1),  # backward + right
}

SPEED_BINDINGS = {
    "+": "increase",
    "=": "increase",
    "-": "decrease",
    "_": "decrease",
}

PRESET_BINDINGS = {
    "1": (0.2, 0.5),  # slow
    "2": (0.5, 1.0),  # medium
    "3": (1.0, 2.0),  # fast
}

HELP_MSG = """
Keyboard Controller
-------------------
Movement:   q  w  e
            a  s  d
            z     c

Speed:      +/-  adjust by 10%
            1/2/3  presets (slow/medium/fast)

SPACE  : emergency stop
CTRL+C : quit

Current speed → linear: {:.2f}  angular: {:.2f}
"""

class KeyboardController(RobotController):
    """
    Keyboard input source.

    Inherits all publisher/safety logic from RobotController.
    This class ONLY handles: reading keys and translating them to commands.

    Notice: no publisher setup, no clamp logic, no Twist construction —
    all of that lives in the base class.
    """

    def __init__(self):
        super().__init__("keyboard_controller")

        self.settings = termios.tcgetattr(sys.stdin)
        self.lock = threading.Lock()
        self.get_logger().info("Keyboard Controller Started")

    # ------------------------------------------------------------------
    # Terminal input helper
    # ------------------------------------------------------------------

    def _get_key(self):
        """Read a single keypress from stdin (non-blocking with 0.1s timeout)."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    # ------------------------------------------------------------------
    # Main input loop — implements the base class contract
    # ------------------------------------------------------------------

    def run(self):
        """
        Read keyboard input in a loop and translate to robot commands.
        This is the only method you need to read to understand what this
        controller does — all complexity is pushed to the base class.
        """
        try:
            print(HELP_MSG.format(self.speed, self.turn))

            while rclpy.ok():
                key = self._get_key()

                if key in MOVE_BINDINGS:
                    lin_dir, ang_dir = MOVE_BINDINGS[key]
                    # Multiply direction (-1, 0, 1) by current speed
                    self.send_command(
                        linear=lin_dir * self.speed,
                        angular=ang_dir * self.turn,
                    )

                elif key in SPEED_BINDINGS:
                    if SPEED_BINDINGS[key] == "increase":
                        self.increase_speed()
                    else:
                        self.decrease_speed()

                elif key in PRESET_BINDINGS:
                    lin, ang = PRESET_BINDINGS[key]
                    self.set_speed(lin, ang)
                    print(f"Preset {key}: linear={lin:.2f}, angular={ang:.2f}")

                elif key == " ":
                    self.stop()
                    print("EMERGENCY STOP")

                elif key == "\x03":  # CTRL+C
                    break

                else:
                    # No key pressed — gradually decelerate to zero
                    # (same smoothing logic as your original teleop)
                    pass

        except Exception as e:
            self.get_logger().error(f"Keyboard controller error: {e}")

        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    controller = None
    try:
        controller = KeyboardController()

        # ROS spin runs in a background thread so the main thread can
        # block on keyboard input without freezing the node
        spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
        spin_thread.start()

        controller.run()

    except KeyboardInterrupt:
        pass
    finally:
        if controller:
            controller.stop()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()