import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription,
                            SetEnvironmentVariable, OpaqueFunction, GroupAction, Shutdown, ExecuteProcess)
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from ros2launch.api import get_share_file_path_from_package

# ===========================


def launch_arguments():
    return [
        # DeclareLaunchArgument("", default_value=""),
    ]
# ==========================


def launch_setup(context, *args, **kwargs):
    tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        name="tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
    )

    keyboard_control = Node(
        package="keyboard_control",
        executable="keyboard_control_plus",
        prefix="xterm -e",
        parameters=[
                {"linear_v_inc": 0.1},
                {"angular_v_inc": 0.1},
                {"publish_topic": "/giraff/cmd_vel"}
        ],
    )

    # send command to unity so that the correct house model is loaded
    loadEnvironment = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub ',
            ' /load_environment',
            ' std_msgs/msg/Int32',
            ' "{data: ' + LaunchConfiguration("VGRHouse").perform(context) + '}"',
            ' -1'
        ]],
        shell=True
    )

    send_pose = Node(
        package="gsl_server",
        executable="send_pose",
        parameters=[
            {"x": parse_substitution("$(var start_pos_x)")},
            {"y": parse_substitution("$(var start_pos_y)")},
            {"z": parse_substitution("$(var start_pos_z)")},
            {"topic": "/giraff/resetPose"}
        ]
    )
    return [
        tcp_endpoint,
        send_pose,
        loadEnvironment,
        # keyboard_control,
    ]


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description)
