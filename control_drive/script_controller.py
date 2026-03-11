import time
import rclpy
from control_drive.robot_controller import RobotController


class ScriptController(RobotController):
    """
    Script-based input source.

    Instead of reading from a keyboard or gamepad, you write your robot's
    behavior directly as Python code inside the run() method below.

    This is perfect for:
      - Testing sequences (move forward, turn, stop)
      - Autonomous behaviors
      - Replaying recorded paths
      - Any logic you want to automate

    Because this inherits from RobotController, the same safety limits,
    speed controls, and publisher are automatically available.

    HOW TO USE:
    -----------
    Write your commands inside run() using:
        self.send_command(linear=..., angular=...)
        self.stop()
        self.set_speed(linear, angular)
        time.sleep(seconds)   ← to hold a command for a duration

    EXAMPLE COMMANDS:
        self.send_command(linear=0.5)           # move forward at 0.5 m/s
        self.send_command(angular=1.0)          # turn left at 1.0 rad/s
        self.send_command(linear=0.5, angular=0.5)  # curve left
        self.stop()                             # immediate stop
    """

    def __init__(self):
        super().__init__(node_name="script_controller", default_speed=0.5, default_turn=1.0)
        self.get_logger().info("Script controller ready.")

    # ------------------------------------------------------------------
    # HELPER: move for a duration
    # These make your scripts much more readable
    # ------------------------------------------------------------------

    def move_forward(self, speed: float = None, duration: float = 1.0):
        """Move forward at `speed` m/s for `duration` seconds."""
        speed = speed or self.speed
        self.get_logger().info(f"Moving forward: {speed:.2f} m/s for {duration:.1f}s")
        self.send_command(linear=speed)
        time.sleep(duration)

    def move_backward(self, speed: float = None, duration: float = 1.0):
        """Move backward at `speed` m/s for `duration` seconds."""
        speed = speed or self.speed
        self.get_logger().info(f"Moving backward: {speed:.2f} m/s for {duration:.1f}s")
        self.send_command(linear=-speed)
        time.sleep(duration)

    def turn_left(self, speed: float = None, duration: float = 1.0):
        """Turn left at `speed` rad/s for `duration` seconds."""
        speed = speed or self.turn
        self.get_logger().info(f"Turning left: {speed:.2f} rad/s for {duration:.1f}s")
        self.send_command(angular=speed)
        time.sleep(duration)

    def turn_right(self, speed: float = None, duration: float = 1.0):
        """Turn right at `speed` rad/s for `duration` seconds."""
        speed = speed or self.turn
        self.get_logger().info(f"Turning right: {speed:.2f} rad/s for {duration:.1f}s")
        self.send_command(angular=-speed)
        time.sleep(duration)

    def pause(self, duration: float = 1.0):
        """Stop and wait for `duration` seconds."""
        self.get_logger().info(f"Pausing for {duration:.1f}s")
        self.stop()
        time.sleep(duration)

    # ------------------------------------------------------------------
    # YOUR SCRIPT GOES HERE
    # This is the only method you need to edit to change robot behavior.
    # ------------------------------------------------------------------

    def run(self):
        """
        Write your robot behavior here.

        This runs once when the node starts. Use the helper methods above
        or call send_command() directly for full control.

        Feel free to use loops, conditionals, and any Python logic you want.
        """
        self.get_logger().info("Script starting...")

        # --- Example: Square pattern ---
        # The robot drives in a square by repeating:
        # forward 2 seconds → turn 90° → repeat x4

        for i in range(4):
            self.get_logger().info(f"Square: side {i + 1}/4")

            self.move_forward(speed=1.0, duration=2.0)
            self.pause(0.5)

            # ~90 degree turn: angular=1.0 rad/s for ~1.57s = π/2 radians
            self.turn_left(speed=1.0, duration=1.57)
            self.pause(0.5)

        self.get_logger().info("Script complete.")
        self.stop()

        # --- More examples (uncomment to try) ---

        # # Zigzag
        # for _ in range(3):
        #     self.send_command(linear=0.5, angular=0.5)
        #     time.sleep(1.0)
        #     self.send_command(linear=0.5, angular=-0.5)
        #     time.sleep(1.0)
        # self.stop()

        # # Spin in place
        # self.send_command(angular=2.0)
        # time.sleep(3.14)   # ~180 degrees
        # self.stop()


def main(args=None):
    rclpy.init(args=args)
    controller = None
    try:
        controller = ScriptController()

        # Run the script in a background thread so ROS can spin normally
        import threading
        script_thread = threading.Thread(target=controller.run, daemon=True)
        script_thread.start()

        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass
    finally:
        if controller:
            controller.stop()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()