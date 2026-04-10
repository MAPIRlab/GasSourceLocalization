import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from ros2launch.api import get_share_file_path_from_package

# ===========================


def launch_arguments():
    return [
        DeclareLaunchArgument("", default_value=""),
    ]
# ==========================


def launch_setup(context, *args, **kwargs):
    graph_creator = Node(
        package="gsl_server",
        executable="graph_creator",
        name="creator",
        parameters=[
            {
                "map_yaml": os.path.join(get_package_share_directory("graphgsl_env"), "data", "graph3", "occupancy.yaml"),
                "root_directory": os.path.join(get_package_share_directory("graphgsl_env"), "data", "graph3")
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" +
                os.path.join(get_package_share_directory("graphgsl_env"), "config", "launch", "graph.rviz")
        ],
    )

    return [
        graph_creator,
        rviz
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
