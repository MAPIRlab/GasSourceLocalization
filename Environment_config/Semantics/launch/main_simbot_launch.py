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

# Internal gaden utilities
import sys
sys.path.append(get_package_share_directory('gaden_common'))
from gaden_internal_py.utils import read_sim_yaml  # NOQA # type: ignore


# ===========================


def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="B"),
        DeclareLaunchArgument("simulation", default_value="B1"),
        DeclareLaunchArgument("method",	default_value=["SemanticPMFS"]),
    ]
# ==========================


def launch_setup(context, *args, **kwargs):
    read_sim_yaml(context)
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

    scenario_folder = os.path.join(
        get_package_share_directory("semantic_gsl_env"),
        "scenarios",
        LaunchConfiguration("scenario").perform(context))
    gsl_node = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_node",
                name="GSL",
                # prefix="xterm -hold -e gdb -ex run --args",
                # prefix="xterm -hold -e gdb --args",
                prefix="xterm -hold -e",
                parameters=[
                    # Common
                    {'robot_location_topic': '/giraff/pose'},
                    {'enose_topic': '/giraff/pid'},
                    {'anemometer_topic': '/giraff/anemometer'},

                    {'use_sim_time': False},
                    {"maxSearchTime": 1000.0},
                    {"stop_and_measure_time": 0.4},
                    {"th_gas_present": parse_substitution("$(var th_gas_present)")},
                    {"th_wind_present": parse_substitution("$(var th_wind_present)")},
                    {"ground_truth_x": parse_substitution("$(var source_x)")},
                    {"ground_truth_y": parse_substitution("$(var source_y)")},
                    {"resultsFile": parse_substitution("Results/$(var simulation)/$(var method).csv")},

                    {"scale": parse_substitution("$(var scale)")},
                    {"markers_height": parse_substitution("$(var markers_height)")},

                    {"anemometer_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
                    {"openMoveSetExpasion": 5},
                    {"explorationProbability": 0.05},
                    {"convergence_thr": 0.0},

                    # GrGSL
                    # -----------------------------
                    {"useDiffusionTerm": True},
                    {"stdevHit": 1.0},
                    {"stdevMiss": 1.2},
                    {"infoTaxis": False},

                    # PMFS
                    # -----------------------------
                    {"headless": False},
                    {"distanceWeight": 0.25},

                    # Hit probabilities
                    {"maxUpdatesPerStop": 5},
                    {"kernelSigma": 1.5},
                    {"kernelStretchConstant": 1.5},
                    {"hitPriorProbability": 0.3},
                    {"confidenceSigmaSpatial": 1.2},
                    {"confidenceMeasurementWeight": 0.7},
                    {"initialExplorationMoves": parse_substitution("$(var initialExplorationMoves)")},

                    # Filament simulation
                    {"useWindGroundTruth": False},
                    {"stepsSourceUpdate": -1},
                    {"maxRegionSize": 5},
                    {"sourceDiscriminationPower": parse_substitution("$(var sourceDiscriminationPower)")},
                    {"refineFraction": 0.2},
                    {"deltaTime": parse_substitution("$(var filamentDeltaTime)")},
                    {"noiseSTDev": parse_substitution("$(var filament_movement_stdev)")},
                    {"iterationsToRecord": parse_substitution("$(var iterationsToRecord)")},
                    {"minWarmupIterations": parse_substitution("$(var minWarmupIterations)")},
                    {"maxWarmupIterations": parse_substitution("$(var maxWarmupIterations)")},
                    {"blurSigmaX": 0.7},
                    {"blurSigmaY": 0.7},

                    # Semantics
                    # -----------------------------
                    {"progressionFileName": parse_substitution("progression_$(var simulation).csv")},
                    {"semanticsType": "ClassMapVoxeland"},
                    {"wallsOccupancyFile": os.path.join(scenario_folder, "_occupancy_walls.pgm")},
                    {"detectionsTopic": "/semantic_instances_3D"},
                    {"ontologyPath": os.path.join(get_package_share_directory("gsl_server"), "resources", "ontology.yaml")},
                    {"targetGas": parse_substitution("$(var targetGas)")},
                    {"masksYAMLPath": os.path.join(scenario_folder, "room_categories", "roomMasks.yaml")},
                    {"roomOntologyPath": os.path.join(get_package_share_directory("gsl_server"), "resources", "ObjectProbByRoom.yaml")},

                    # ClassMap2D
                    {"zMin": -0.7},
                    {"zMax": 1.0},
                ],
                on_exit=Shutdown()
            ),
        ])
    ]

    gmrf_wind = Node(
        package="gmrf_wind_mapping",
        executable="gmrf_wind_mapping_node",
        name="gmrf",
        prefix="xterm -hold -T GMRF -e",
        parameters=[
            {"sensor_topic": parse_substitution("$(var robot_name)/anemometer")},
            {"map_topic": parse_substitution("$(var robot_name)/map")},
            {"cell_size": 0.25},
            {"exec_freq": 5.0},
            {"map_file": os.path.join(scenario_folder, "_occupancy_gmrf.pgm")}
        ]
    )

    gaden_player = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("semantic_gsl_env"),
                    "launch",
                    "gaden_player_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_rviz": "False",
            "scenario": LaunchConfiguration("scenario").perform(context),
            "simulation": LaunchConfiguration("simulation").perform(context)
        }.items(),
    )

    unity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("semantic_gsl_env"),
                    "launch",
                    "unity_launch.py",
                )
            ]
        ),
        launch_arguments={
        }.items(),
    )

    nav2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' launch',
            ' semantic_gsl_env',
            ' nav2_launch.py',
            ' scenario:='+LaunchConfiguration("scenario").perform(context),
            ' namespace:=giraff',
        ]],
        prefix="xterm -hold -T nav2 -e",
    )

    anemometer = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),
            Node(
                package="simulated_anemometer",
                executable="simulated_anemometer",
                name="Anemometer",
                parameters=[
                    {"sensor_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
                    {"fixed_frame": "map"},
                    {"noise_std": 0.3},
                    {"use_map_ref_system": False},
                    {'use_sim_time': True},
                    {'topic': '/giraff/anemometer'}
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='anemometer_tf_pub',
                arguments=['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution('$(var robot_name)_base_link'), parse_substitution('$(var robot_name)_anemometer_frame')],
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
                    {"sensor_frame": parse_substitution("$(var robot_name)_pid_frame")},
                    {"fixed_frame": "map"},
                    {"noise_std": 20.1},
                    {'use_sim_time': True},
                    {'topic': '/giraff/pid'}
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='pid_tf_pub',
                arguments=['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution('$(var robot_name)_base_link'), parse_substitution('$(var robot_name)_pid_frame')],
                parameters=[{'use_sim_time': True}]
            ),
        ])
    ]

    rvizHit = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("semantic_gsl_env"), "launch", "hit.rviz")
        ],
    )

    rvizSource = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("semantic_gsl_env"), "launch", "source.rviz")
        ],
    )

    semantics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_share_file_path_from_package(package_name="semantic_gsl_env", file_name="semantics_launch.py")),
        launch_arguments={
            "Mode": "Sim",
        }.items()
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

    map_server = Node(
        package="mapir_map_server",
        executable="mapir_map_server",
        parameters=[
            {"topic": "/giraff/map"},
            {"yaml_filename": os.path.join(scenario_folder, "_occupancy_walls.yaml")},
        ]
    )

    rosbag = ExecuteProcess(
        cmd=[[
            'xterm -T rosbag -e ',
            FindExecutable(name='ros2'),
            ' bag play ',
            ' /mnt/HDD/rosbags/',
            'attempt3',
            ' --start-offset 30',
            ' --rate 2'
        ]],
        shell=True
    )

    wind_map_creator = Node(
        package="wind_map_creator",
        executable="gui_pub",
        name="windMap",
        # prefix="xterm -T windMapCreator -hold -e ",
        parameters=[
            {"listenTopic": "/giraff/initialpose"},
            {"publishTopic": "/giraff/Anemometer/WindSensor_reading"}
        ]
    )

    fakeSensors = [
        Node(
            package="fake_sensors",
            executable="anemometer",
            name="fake_anemometer",
            prefix="xterm -hold -e",
            parameters=[
                {"mapTopic": "/giraff/map"},
                {"poseTopic": "/giraff/pose"},
                {"pubTopic": "/giraff/anemometer"},
                {"frequency": 5.0},
                {"mapScale": 0.2},
                {"noiseScale": 0.05},
                {"windImagePath": os.path.join(
                    scenario_folder, "simulations", "Pepe1A.png")},
            ]
        ),
        Node(
            package="fake_sensors",
            executable="gasSensor",
            name="fake_gasSensor",
            prefix="xterm -hold -e",
            parameters=[
                {"mapTopic": "/giraff/map"},
                {"poseTopic": "/giraff/pose"},
                {"pubTopic": "/giraff/pid"},
                {"frequency": 5.0},
                {"gasImagePath": os.path.join(
                    scenario_folder, "simulations", LaunchConfiguration("gasImage").perform(context))},
            ]
        )
    ]

    actions = []
    actions.append(gaden_player)
    actions.extend(anemometer)
    actions.extend(PID)
    actions.append(unity)
    actions.append(nav2)
    actions.append(keyboard_control)

    # actions.append(wind_map_creator)
    # actions.extend(fakeSensors)

    # actions.append(map_server)
    # actions.append(rosbag)

    actions.append(gmrf_wind)
    actions.extend(gsl_node)
    actions.extend(gsl_call)
    actions.append(semantics)

    actions.append(rvizHit)
    actions.append(rvizSource)

    return actions


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("semantic_gsl_env")],
        ),
        SetLaunchConfiguration(
            name="nav_params_yaml",
            value=[PathJoinSubstitution(
                [LaunchConfiguration("pkg_dir"), "navigation_config", "nav2_params.yaml"]
            )],
        ),

        SetLaunchConfiguration(
            name="robot_name",
            value="giraff"
        ),

        # GSL params (overwritable in each YAML)
        ##############################################
        SetLaunchConfiguration(
            name="th_gas_present",
            value="0.1"
        ),
        SetLaunchConfiguration(
            name="th_wind_present",
            value="0.02"
        ),

        SetLaunchConfiguration(
            name="filament_movement_stdev",
            value="0.15"
        ),
        SetLaunchConfiguration(
            name="sourceDiscriminationPower",
            value="0.1"
        ),
        SetLaunchConfiguration(
            name="iterationsToRecord",
            value="200"
        ),
        SetLaunchConfiguration(
            name="minWarmupIterations",
            value="500"
        ),
        SetLaunchConfiguration(
            name="maxWarmupIterations",
            value="800"
        ),
        SetLaunchConfiguration(
            name="initialExplorationMoves",
            value="3"
        ),
        SetLaunchConfiguration(
            name="filamentDeltaTime",
            value="0.2"
        ),

        SetLaunchConfiguration(
            name="targetGas",
            value="smoke"
        ),

        SetLaunchConfiguration(
            name="scale",
            value="25"
        ),
        SetLaunchConfiguration(
            name="markers_height",
            value="0.1"
        ),
        SetLaunchConfiguration(
            name="VGRHouse",
            value="1"
        ),
        SetLaunchConfiguration(
            name="gasImage",
            value=""
        ),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description)
