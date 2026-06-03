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
        DeclareLaunchArgument("scenario", default_value="graph4"),
        DeclareLaunchArgument("config", default_value="config1"),
        DeclareLaunchArgument("simulation", default_value="sim1"),
        DeclareLaunchArgument("method",	default_value=["GraphGSL"]),
    ]
# ==========================


def launch_setup(context, *args, **kwargs):
    gsl_call = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_call",
                name="gsl_call",
                parameters=[
                    {"method": parse_substitution("$(var method)")},
                ],
            ),
        ])
    ]
    gsl_node = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_node",
                name="GSL",
                # prefix="xterm -hold -e gdb -ex run --args",
                # prefix="xterm -hold -e",
                parameters=[
                    {"graph_path": os.path.join(get_package_share_directory(
                        "graphgsl_env"), "data", "environments", "graph4", "graph")},
                    {"sim_measurements_path": os.path.join(get_package_share_directory(
                        "graphgsl_env"), "data", "test_data", "data_graph_4")},
                    {"cell_size": 0.15},
                    {"node_separation_mult": 1.0},
                    {"robot_location_topic": "/PioneerP3DX/ground_truth"},

                    # Advection constraint -> neighboring cells should have similar wind values in the direction of the wind
                    {"GMRF_lambdaPrior_advection": 100.0},
                    # Mass conservation law -> divergence of the wind field is zero
                    {"GMRF_lambdaPrior_mass_conservation": 1000.0},
                    # Diffusion constraint -> neighboring cells should have similar wind values in all directions
                    {"GMRF_lambdaPrior_diffusion": 100.0},
                    # Obstacles --> cells close to obstacles has only tangencial wind
                    {"GMRF_lambdaPrior_obstacles": 2000.0},
                ],
                on_exit=Shutdown()
            ),
        ])
    ]

    basic_sim = Node(
        package="basic_sim",
        executable="basic_sim",
        parameters=[
            {"deltaTime": 0.05},
            {"speed": 5.0},
            {"worldFile": parse_substitution(
                "$(find-pkg-share graphgsl_env)/data/environments/$(var scenario)/gaden/environment_configurations/$(var config)/BasicSimScene.yaml")}
        ],
    )

    gaden_player = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("graphgsl_env"),
                    "config",
                    "launch",
                    "gaden_player_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_rviz": "False",
            "scenario": LaunchConfiguration("scenario").perform(context),
            "config": LaunchConfiguration("config").perform(context),
            "simulation": LaunchConfiguration("simulation").perform(context)
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("graphgsl_env"),
                "config/navigation_config/nav2_launch.py",
            )
        ),
        launch_arguments={
            "scenario": LaunchConfiguration("scenario"),
            "namespace": LaunchConfiguration("robot_name")
        }.items(),
    )

    anemometer = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),
            Node(
                package="simulated_anemometer",
                executable="simulated_anemometer",
                name="Anemometer",
                parameters=[
                    {"sensor_frame": parse_substitution(
                        "$(var robot_name)_anemometer_frame")},
                    {"fixed_frame": "map"},
                    {"noise_std": 0.2},
                    {"use_map_ref_system": False},
                    {'use_sim_time': True},
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='anemometer_tf_pub',
                arguments=['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution(
                    '$(var robot_name)_base_link'), parse_substitution('$(var robot_name)_anemometer_frame')],
                parameters=[{'use_sim_time': True}]
            ),
        ])
    ]

    PID = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),
            Node(
                package="simulated_gas_sensor",
                executable="simulated_gas_sensor",
                name="PID",
                parameters=[
                    {"sensor_model": 30},
                    {"sensor_frame": parse_substitution(
                        "$(var robot_name)_pid_frame")},
                    {"fixed_frame": "map"},
                    {"noise_std": 20.1},
                    {'use_sim_time': True},
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='pid_tf_pub',
                arguments=['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution(
                    '$(var robot_name)_base_link'), parse_substitution('$(var robot_name)_pid_frame')],
                parameters=[{'use_sim_time': True}]
            ),
        ])
    ]

    windMapCreator = Node(
        package="wind_map_creator",
        executable="gui_pub",
        parameters=[
                {"listenTopic": parse_substitution(
                    "$(var robot_name)/initialpose")},
                {"publishTopic": parse_substitution(
                    "$(var robot_name)/Anemometer/WindSensor_reading")},
                {"poseTopic": parse_substitution(
                    "$(var robot_name)/amcl_pose")}
        ],
    )

    observationRecorder = Node(
        package="gsl_server",
        executable="observation_recorder",
        name="obs",
        prefix="xterm -hold -e",
        parameters=[
                {"pose_topic": "/measured_localization"},
                {"wind_topic": "/measured_wind"},
                {"gas_topic": "/measured_gas"},
                {"file_path": os.path.join(get_package_share_directory(
                    "graphgsl_env"), "data", "test_data", "data_graph_4")},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("graphgsl_env"),
                                "config", "launch", "graph.rviz")
        ],
    )
    actions = []
    actions.append(gaden_player)
    actions.extend(anemometer)
    actions.extend(PID)
    actions.append(nav2)
    actions.append(basic_sim)
    actions.extend(gsl_node)
    actions.extend(gsl_call)
    actions.append(rviz)
    # actions.append(windMapCreator)
    actions.append(observationRecorder)

    return actions


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("graphgsl_env")],
        ),
        SetLaunchConfiguration(
            name="nav_params_yaml",
            value=[PathJoinSubstitution(
                [LaunchConfiguration("pkg_dir"),
                 "config", "navigation_config", "nav2_params.yaml"]
            )],
        ),

        SetLaunchConfiguration(
            name="robot_name",
            value="PioneerP3DX"
        ),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description)
