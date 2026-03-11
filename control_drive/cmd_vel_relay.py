import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.publisher = self.create_publisher(
            Twist,
            "/control_drive_controller/cmd_vel_unstamped",
            10
        )
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.relay_callback,
            10
        )
        self.get_logger().info("cmd_vel relay ready: /cmd_vel → /control_drive_controller/cmd_vel_unstamped")

    def relay_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
