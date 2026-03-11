from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():

    # Step 1 — Set hidraw permissions
    # This is the equivalent of: sudo chmod 666 /dev/hidraw*
    set_hidraw_permissions = ExecuteProcess(
        cmd=["sudo", "chmod", "666", "/dev/hidraw0"],
        output="screen",
        name="set_hidraw_permissions",
    )

    # Step 2 — Verify PS4 controller is connected
    # Checks lsusb for Sony DualShock 4 (054c:09cc)
    check_controller = ExecuteProcess(
        cmd=["bash", "-c",
             "lsusb | grep -q '054c:09cc' && echo 'PS4 controller detected!' || (echo 'ERROR: PS4 controller not found. Run: usbipd attach --wsl --busid 2-1' && exit 1)"
        ],
        output="screen",
        name="check_controller",
    )

    # Start gamepad controller node
    gamepad_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="control_drive",
                executable="gamepad_controller",
                name="gamepad_controller",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        set_hidraw_permissions,
        check_controller,
        gamepad_node,
    ])