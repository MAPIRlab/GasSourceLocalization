import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction,Shutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
import xacro

# TODO:
# move all the scenario-specific things to a yaml
# implement the simulation file structure

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="A"),
        DeclareLaunchArgument("simulation", default_value="A1"),
        DeclareLaunchArgument("method",	default_value=["PMFS"]),
        DeclareLaunchArgument("use_infotaxis", default_value=["False"]),
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration("method").perform(context)
    gsl = [
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
            Node(
                package="gsl_server",
                executable="gsl_actionserver_node",
                name="gsl_node",
                parameters=[
                    # Common
                    {'use_sim_time': False},	
                    {"max_search_time": 300.0},
                    {"robot_location_topic": "ground_truth"},
                    {"stop_and_measure_time": 0.4 if method == "PMFS" else 2.0},
                    {"th_gas_present": 0.1},
                    {"th_wind_present": 0.02},
                    {"ground_truth_x": parse_substitution("$(var source_x)")},
                    {"ground_truth_y": parse_substitution("$(var source_y)")},
                    {"results_file": parse_substitution("Results/$(var simulation)/$(var method).csv")},
                    
                    {"scale": 25},
                    {"markers_height": 0.2},

                    {"anemometer_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
                    {"allowMovementRepetition": True},
                    {"openMoveSetExpasion": 5},
                    {"explorationProbability": 0.1},
                    {"convergence_thr": 1.5},
                    {"convergence_steps": 5},
                    
                    #GrGSL
                    {"useDiffusionTerm": True},
                    {"stdev_hit": 1.0},
                    {"stdev_miss": 1.2},
                    {"infoTaxis": parse_substitution("$(var use_infotaxis)")},

                    #PMFS
                        # Hit probabilities
                    {"headless": True},
                    {"max_updates_per_stop": 5},
                    {"kernel_sigma": 1.5},
                    {"kernel_stretch_constant": 1.5},
                    {"hitPriorProbability": 0.3},
                    {"confidence_sigma_spatial": 1.0},
                    {"confidence_measurement_weight": 1.0},
                        #Filament simulation
                    {"useWindGroundTruth": True},
                    {"stepsSourceUpdate": 3},
                    {"maxRegionSize": 5},
                    {"sourceDiscriminationPower": 0.2},
                    {"refineFraction": 0.25},
                    {"deltaTime": 0.1},
                    {"noiseSTDev": parse_substitution("$(var filament_movement_stdev)")},
                    {"iterationsToRecord": 100},
                    {"maxWarmupIterations": 500},

                    
                ],
                on_exit=Shutdown()
            ),
        ])
    ]

    gmrf_wind = Node(
        package="gmrf_wind_mapping",
        executable="gmrf_wind_mapping_node",
        name="gmrf",
        parameters=[
            {"sensor_topic": parse_substitution("$(var robot_name)/Anemometer/WindSensor_reading")},
            {"map_topic": parse_substitution("$(var robot_name)/map")},
            {"cell_size": 0.25},
        ]
    )

    basic_sim = Node(
        package="basic_sim",
        executable="basic_sim",
        parameters=[
            {"deltaTime": 0.1},
            {"speed": 5.0},
            {"worldFile": parse_substitution("$(find-pkg-share pmfs_env)/scenarios/$(var scenario)/basicSim/$(var simulation).yaml")}
            ],
    )

    gaden_player = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("pmfs_env"),
                    "launch",
                    "gaden_player_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_rviz": "True",
            "scenario": LaunchConfiguration("scenario").perform(context),
            "simulation": LaunchConfiguration("simulation").perform(context)
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("pmfs_env"),
                    "navigation_config/nav2_launch.py",
                )
            ),
            launch_arguments={
                "scenario": LaunchConfiguration("scenario"),
                "namespace" : LaunchConfiguration("robot_name")
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
                    {"sensor_frame" : parse_substitution("$(var robot_name)_anemometer_frame") },
                    {"fixed_frame" : "map"},
                    {"noise_std" : 0.3},
                    {"use_map_ref_system" : False},
                    {'use_sim_time': True},
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='anemometer_tf_pub',
                arguments = ['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution('$(var robot_name)_base_link'), parse_substitution('$(var robot_name)_anemometer_frame')],
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
                    {"sensor_model" : 30 },
                    {"sensor_frame" : parse_substitution("$(var robot_name)_pid_frame") },
                    {"fixed_frame" : "map"},
                    {"noise_std" : 20.1},
                    {'use_sim_time': True},
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='pid_tf_pub',
                arguments = ['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution('$(var robot_name)_base_link'), parse_substitution('$(var robot_name)_pid_frame')],
                parameters=[{'use_sim_time': True}]
            ),
        ])
    ]

    actions = []
    actions.append(gaden_player)
    actions.extend(anemometer)
    actions.extend(PID)
    actions.append(nav2)
    actions.append(gmrf_wind)
    actions.append(basic_sim)
    actions.extend(gsl)

    return actions


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("pmfs_env")],
        ),
        SetLaunchConfiguration(
            name="nav_params_yaml",
            value=[PathJoinSubstitution(
                [LaunchConfiguration("pkg_dir"), "navigation_config", "nav2_params.yaml"]
            )],
        ),

        SetLaunchConfiguration(
            name="robot_name", 
            value="PioneerP3DX"
        ),

        SetLaunchConfiguration(
            name="filament_movement_stdev", 
            value="0.5"
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)