import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import threading


# Format: key -> (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
moveBindings = {
    "w": (1, 0, 0, 0, 0, 0),  # forward
    "s": (-1, 0, 0, 0, 0, 0),  # backward
    "a": (0, 0, 0, 0, 0, 1),  # turn left
    "d": (0, 0, 0, 0, 0, -1),  # turn right
    "q": (1, 0, 0, 0, 0, 1),  # forward + left
    "e": (1, 0, 0, 0, 0, -1),  # forward + right
    "z": (-1, 0, 0, 0, 0, 1),  # backward + left
    "c": (-1, 0, 0, 0, 0, -1),  # backward + right
}

# Speed adjustments settings
speedBindigns = {
    "+": (1.1, 1.1),  # increase speed
    "=": (1.1, 1.1),  # increase speed (without shift)
    "-": (0.9, 0.9),  # decrease speed
    "_": (0.9, 0.9),  # decrease speed (with shift)
}

# Gear/Speed presets
presetBindings = {
    "1": (0.2, 0.5),  # slow
    "2": (0.5, 1.0),  # medium
    "3": (1.0, 2.0),  # fast
}

msg = """ 
ROS2 custom Teleop Twist controller

Movement controls 

q  w  e 
a  s  d
z  x  c 

w/s : forward/backward
a/d : turn left/right
q/e : forward+left/forward+right
z/c : backward+left/backward+right

Speed Controls:
+/- : increase/decrease speeds by 10%%  
1/2/3 : set speed presets (slow/medium/fast) 

Current Settings:
Linear Speed: %.2f
Angular Speed: %.2f

Special Keys:
SPACE : emergency stop (all velocities to 0)
CTRL+C : quit

Press any movement key to start...

"""


class CustomTeleop(Node):
    def __init__(self):
        super().__init__(node_name="custom_teleop_twist_keyboard")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.speed = 0.5
        self.turn = 1.0

        self.twist = Twist()

        self.settings = termios.tcgetattr(sys.stdin)

        self.status = 0
        self.last_twist = Twist()

        self.timer = self.create_timer(0.1, self.publish_twist)

        self.should_publish = False

        self.twist_lock = threading.Lock()

        self.get_logger().info("Custom Teleop Twist Keyboard Controller Started")
        self.get_logger().info(f"Publishing to: {self.publisher.topic_name}")

    def getKey(self):
        "Get a single key press from stdin"
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        """Returns a formatted string with current velocities"""
        return "Linear: %.2f, Angular: %.2f" % (speed, turn)

    def makeSimpleProfile(self, output, input, slop):
        """Create a simple velocity profile with smoothing"""
        if input > slop:
            output = min(input, output + slop)
        elif input < -slop:
            output = max(input, output - slop)
        else:
            output = 0.0
        return output

    def constrain(self, input_vel, low_boundary, high_boundary):
        """Constrain velocity withing boundaries"""
        if input_vel < low_boundary:
            input_vel = low_boundary
        elif input_vel > high_boundary:
            input_vel = high_boundary
        return input_vel

    def checkLinearLimitVelocity(self, velocity):
        """Check linear limit velocity"""
        return self.constrain(velocity, -2.0, 2.0)

    def checkAngularLimitVelocity(self, velocity):
        """Check angular limit velocity"""
        return self.constrain(velocity, -2.0, 2.0)

    def publish_twist(self):
        if self.should_publish:
            with self.twist_lock:
                self.publisher.publish(self.twist)

    def stop_robot(self):
        """Send stop command to robot"""
        with self.twist_lock:
            self.twist = Twist()
        self.publisher.publish(self.twist)

    def run(self):
        """Main control loop"""
        try:
            print(msg.format(self.speed, self.turn))

            while rclpy.ok():
                key = self.getKey()

                if key in moveBindings.keys():
                    x = float(moveBindings[key][0] * self.speed)
                    y = float(moveBindings[key][1] * self.speed)
                    z = float(moveBindings[key][2] * self.speed)
                    th_x = float(moveBindings[key][3] * self.turn)
                    th_y = float(moveBindings[key][4] * self.turn)
                    th_z = float(moveBindings[key][5] * self.turn)

                    with self.twist_lock:
                        self.twist.linear.x = x
                        self.twist.linear.y = y
                        self.twist.linear.z = z
                        self.twist.angular.x = th_x
                        self.twist.angular.y = th_y
                        self.twist.angular.z = th_z

                    self.should_publish = True
                    print(
                        f"Moving: {key} | Linear: ({x:.2f}, {y:.2f}, {z:.2f}) | Angular: ({th_x:.2f}, {th_y:.2f}, {th_z:.2f})"
                    )

                elif key in speedBindigns.keys():
                    self.speed = float(self.speed * speedBindigns[key][0])
                    self.turn = float(self.turn * speedBindigns[key][1])
                    self.speed = self.checkLinearLimitVelocity(self.speed)
                    self.turn = self.checkAngularLimitVelocity(self.turn)

                    print(f"Speed Adjustment: {self.vels(self.speed, self.turn)}")
                    if self.status == 14:
                        print(msg.format(self.speed, self.turn))
                    self.status = (self.status + 1) % 15

                elif key in presetBindings.keys():
                    self.speed = float(presetBindings[key][0])
                    self.turn = float(presetBindings[key][1])
                    print(f"Speed preset {key}: {self.vels(self.speed, self.turn)}")

                elif key == " ":
                    with self.twist_lock:
                        self.twist = Twist()
                    self.should_publish = True
                    print("EMERGENCY STOP - All velocities set to 0")

                elif key == "\x03":
                    break

                else:
                    with self.twist_lock:
                        self.twist.linear.x = float(self.makeSimpleProfile(
                            self.twist.linear.x, 0.0, 0.01
                        ))
                        self.twist.linear.y = float(self.makeSimpleProfile(
                            self.twist.linear.y, 0.0, 0.01
                        ))
                        self.twist.linear.z = float(self.makeSimpleProfile(
                            self.twist.linear.z, 0.0, 0.01
                        ))
                        self.twist.angular.x = float(self.makeSimpleProfile(
                            self.twist.angular.x, 0.0, 0.1
                        ))
                        self.twist.angular.y = float(self.makeSimpleProfile(
                            self.twist.angular.y, 0.0, 0.1
                        ))
                        self.twist.angular.z = float(self.makeSimpleProfile(
                            self.twist.angular.z, 0.0, 0.1
                        ))

                    all_zero = (
                        abs(self.twist.linear.x or 0.0) < 0.01
                        and abs(self.twist.linear.y or 0.0) < 0.01
                        and abs(self.twist.linear.z or 0.0) < 0.01
                        and abs(self.twist.angular.x or 0.0) < 0.1
                        and abs(self.twist.angular.y or 0.0) < 0.1
                        and abs(self.twist.angular.z or 0.0) < 0.1
                    )

                    self.should_publish = not all_zero

                    if key == "\x03":
                        break

        except Exception as e:
            self.get_logger().error(f"Error in teleop controller: {e}")

        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info("Teleop controller stopped")


def main(args=None):
    rclpy.init(args=args)

    controller = None
    try:
        controller = CustomTeleop()

        spin_thread = threading.Thread(target=rclpy.spin, args=(controller,))
        spin_thread.daemon = True
        spin_thread.start()

        controller.run()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if controller is not None:
            controller.stop_robot()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
