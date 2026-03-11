import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotController(Node):
    """
    Base class for all robot input sources.

    WHAT THIS DOES:
    ---------------
    Every input source (keyboard, PS4, script) inherits from this class.
    This means they all automatically get:
      - A publisher to /cmd_vel
      - Speed limits and velocity constraints
      - A stop() method
      - A send_command() method with safety checks

    WHY THIS MATTERS:
    -----------------
    Instead of each input source managing its own publisher and reinventing
    safety checks, they all share this one implementation. To add a new input
    source, you just inherit from this class and implement `run()`.

    HOW TO USE:
    -----------
    class MyController(RobotController):
        def __init__(self):
            super().__init__(node_name="my_controller")  # always call this first

        def run(self):
            # your input logic here, then call:
            self.send_command(linear=0.5, angular=0.0)    
    """

    MAX_LINEAR_SPEED = 2.0 # m/s
    MAX_ANGULAR_SPEED = 2.0 # rad/s 

    def __init__(self, node_name: str, default_speed : float = 1.0, default_turn: float = 1.0):
        """
        Args:
            node_name (str): Name of the ROS2 node
            default_speed (float): Starting linear speed (m/s)
            default_turn (float): Starting angular speed (rad/s)
        """

        super().__init__(node_name=node_name)

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.speed = default_speed
        self.turn = default_turn

        self.get_logger().info(f"{node_name} initialized with default speed {self.speed} m/s and turn {self.turn} rad/s")

    # -----------------------------------------------------------------
    # CORE COMMAND METHODS
    # This is the main method that all subclassed will call to move the robot
    # -----------------------------------------------------------------

    def send_command(self, linear: float=0.0, angular: float=0.0):
        """
        Send a velocity command to the robot.

        Args:
            linear:  forward/backward speed in m/s  (positive = forward)
            angular: rotation speed in rad/s         (positive = left/CCW)

        Safety: values are automatically clamped to MAX limits before sending.

        Example:
            self.send_command(linear=0.5, angular=0.0)   # move forward
            self.send_command(linear=0.0, angular=1.0)   # turn left
            self.send_command(linear=-0.3, angular=0.5)  # backward + left
        """
        twist = Twist()
        twist.linear.x = self._clamp(linear, -self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED)
        twist.angular.z = self._clamp(angular, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
        self.publisher.publish(twist)

    def stop(self):
        """
        Send a zero-velocity command to stop the robot immediately.
        Always call this on shutdown or when switching input sources.
        """
        self.publisher.publish(Twist())  # all zeros
        self.get_logger().info("Robot Stopped")

    # ------------------------------------------------------------------
    # SPEED HELPERS
    # Subclasses use these to adjust speed cleanly.
    # ------------------------------------------------------------------

    def increase_speed(self, factor: float = 1.1):
        """Multiply current speed by factor (e.g. 1.1 = +10%)"""
        self.speed = self._clamp(self.speed * factor, 0.0, self.MAX_LINEAR_SPEED)
        self.turn = self._clamp(self.turn * factor, 0.0, self.MAX_ANGULAR_SPEED)
        self.get_logger().info(f"Speed increased → linear: {self.speed:.2f}, angular: {self.turn:.2f}")

    def decrease_speed(self, factor: float = 0.9):
        """Multiply current speed by factor (e.g. 0.9 = -10%)"""
        self.speed = self._clamp(self.speed * factor, 0.0, self.MAX_LINEAR_SPEED)
        self.turn = self._clamp(self.turn * factor, 0.0, self.MAX_ANGULAR_SPEED)
        self.get_logger().info(f"Speed decreased → linear: {self.speed:.2f}, angular: {self.turn:.2f}")

    def set_speed(self, linear: float, angular: float):
        """Set speed directly (e.g. for presets)"""
        self.speed = self._clamp(linear, 0.0, self.MAX_LINEAR_SPEED)
        self.turn = self._clamp(angular, 0.0, self.MAX_ANGULAR_SPEED)
        self.get_logger().info(f"Speed set → linear: {self.speed:.2f}, angular: {self.turn:.2f}")

    # ------------------------------------------------------------------
    # INTERNAL HELPERS
    # ------------------------------------------------------------------

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp a value between min and max. Used for safety limits."""
        return max(min_val, min(max_val, value))
    
    # ------------------------------------------------------------------
    # INTERFACE CONTRACT
    # Subclasses SHOULD override run() with their input logic.
    # It's not enforced (no abstractmethod) to keep things simple,
    # but every controller is expected to implement it.
    # ------------------------------------------------------------------

    def run(self):
        """
        Override this in your subclass with your input loop.
        This is where keyboard reading, PS4 polling, or script logic goes.
        """
        raise NotImplementedError(
            f"[{self.get_name()}] You must implement run() in your controller subclass."
        )