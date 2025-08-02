from launch_ros.actions import Node
from launch import LaunchDescription

# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("control_drive")
    rviz_config_file = os.path.join(share_dir, "config", "display.rviz")

    # xacro_file = os.path.join(share_dir, "urdf", "diff_drive.xacro")

    # robot_description_config = xacro.process_file(xacro_file)
    # robot_urdf = robot_description_config.toxml()

    # use_gazebo_arg = DeclareLaunchArgument(
    #     name="use_gazebo",
    #     default_value="False",
    #     description="Set to True when using Gazebo (Disables joint state publishers)",
    # )

    # gui_arg = DeclareLaunchArgument(
    #     name="gui", default_value="True", description="Start joint state publisher GUI"
    # )

    # show_gui = LaunchConfiguration("gui")
    # use_gazebo = LaunchConfiguration("use_gazebo")

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     parameters=[{"robot_description": robot_urdf}],
    #     output = "screen",
    # )

    # joint_state_publisher_node = Node(
    #     condition=UnlessCondition(use_gazebo),
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     output = "screen",
    # )

    # joint_state_publisher_gui_node = Node(
    #     condition=IfCondition(show_gui) and UnlessCondition(use_gazebo),
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     output = "screen",
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription(
        [
            # use_gazebo_arg,
            # gui_arg,
            # robot_state_publisher_node,
            # joint_state_publisher_node,
            # joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
