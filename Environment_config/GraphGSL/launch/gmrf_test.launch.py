import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription,
                            SetEnvironmentVariable, OpaqueFunction, GroupAction, Shutdown, ExecuteProcess)
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

# ===========================


def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="A"),
        DeclareLaunchArgument("simulation", default_value="A1"),
        DeclareLaunchArgument("method",	default_value=["GraphGSL"]),
    ]
# ==========================


def launch_setup(context, *args, **kwargs):
    windMapCreator = Node(
        package="wind_map_creator",
        executable="gui_pub",
        prefix="xterm -hold -e",
        parameters=[
                {"listenTopic": "initialpose"},
                {"publishTopic": "Anemometer/WindSensor_reading"},
                {"poseTopic": "amcl_pose"}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("graphgsl_env"), "launch", "gmrf.rviz")
        ],
    )

    gmrf_wind = Node(
        package="gmrf_wind_mapping",
        executable="gmrf_wind_mapping_node",
        name="gmrf",
        parameters=[
            {"sensor_topic": "/Anemometer/WindSensor_reading"},
            {"map_yaml_file": os.path.join(get_package_share_directory("graphgsl_env"), "test_graph", "occupancy.yaml")},
            {"cell_size": 0.25},
            {"exec_freq": 10.0},
            
            # Monroy branch parameters
            {"GMRF_lambdaPrior_advection": 100.0},             # Advection constraint -> neighboring cells should have similar wind values in the direction of the wind
            {"GMRF_lambdaPrior_mass_conservation": 1000.0},    # Mass conservation law -> divergence of the wind field is zero
            {"GMRF_lambdaPrior_diffusion": 0.0001},            # Diffusion constraint -> neighboring cells should have similar wind values in all directions
            {"GMRF_lambdaPrior_obstacles": 2000.0},            # Obstacles --> cells close to obstacles has only tangencial wind
            {"num_iterations_MAP": 20},                        # Maximum number of iterations for the MAP estimation optimization
        ]
    )

    actions = []
    actions.append(rviz)
    actions.append(windMapCreator)
    actions.append(gmrf_wind)

    return actions


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description)
