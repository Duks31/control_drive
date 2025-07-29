from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("control_drive")

    set_env_vars = AppendEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH", os.path.join(share_dir, "meshes")
    )

    set_model_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(share_dir, "meshes")
    )

    xacro_file = os.path.join(share_dir, "urdf", "diff_drive.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_urdf}, {"use_sim_time": True}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
    )

    # Ignition Gazebo server launch
    ignition_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -s ", "empty.sdf"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Ignition Gazebo client launch
    ignition_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={"gz_args": "-g "}.items(),
    )

    urdf_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "diff_drive", "-topic", "robot_description"],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    diff_drive_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "control_drive_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    # Bridge from ROS2 to Gazebo
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    share_dir, "config", "control_drive_bridge.yaml"
                ),
                "use_sim_time": True
            }
        ],
        output="screen",
    )


    return LaunchDescription(
        [
            set_env_vars,
            set_model_path,
            robot_state_publisher_node,
            joint_state_publisher_node,
            ignition_gazebo_server,
            ignition_gazebo_client,
            urdf_spawn_node,
            bridge,
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
        ]
    )
