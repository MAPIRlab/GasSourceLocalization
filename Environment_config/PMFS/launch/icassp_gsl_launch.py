import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction,Shutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="ICASSP"),
        DeclareLaunchArgument("simulation", default_value="A1"),
        DeclareLaunchArgument("method",	default_value=["PMFS"]),
        DeclareLaunchArgument("use_infotaxis", default_value=["True"]),
        DeclareLaunchArgument("map_height", default_value="0.32"), #[0.025, 0.105, 0.185,    0.265, 0.345, 0.425,    0.505, 0.585, 0.665, 0.745, 0.825, 0.905]
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration("method").perform(context)
    map_file = os.path.join(
		get_package_share_directory("pmfs_env"),
        "scenarios",
        "ICASSP",
		LaunchConfiguration("map_height").perform(context),
		"occupancy.yaml",
	)
    map_height = float(LaunchConfiguration("map_height").perform(context))


    map_server = [
        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration("robot_name")),            
            # map_server
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                #arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                parameters=[
                    {"use_sim_time": False},
                    {"yaml_filename": map_file},
                    {"frame_id": "map"},
                ],
            ),

            # LIFECYCLE MANAGER
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                #arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                parameters=[
                    {"use_sim_time": False},
                    {"autostart": True},
                    {"node_names": [
                        "map_server",
                        ]
                    },
                ],
            ),
        ])        
    ]

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
                prefix="xterm -hold -e",
                name="GSL",
                parameters=[
                    # Common
                    {'use_sim_time': False},	
                    {"maxSearchTime": 300.0},
                    {"robot_location_topic": "ground_truth"},
                    {"stop_and_measure_time": 0.4},
                    {"th_gas_present": parse_substitution("$(var th_gas_present)")},
                    {"th_wind_present": parse_substitution("$(var th_wind_present)")},
                    #{"ground_truth_x": parse_substitution("$(var source_x)")},
                    #{"ground_truth_y": parse_substitution("$(var source_y)")},
                    {"resultsFile": parse_substitution("Results/$(var simulation)/$(var method).csv")},
                    
                    {"scale": 70},
                    {"markers_height": 0.2},

                    {"anemometer_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
                    {"openMoveSetExpasion": 5},
                    {"explorationProbability": 0.05},
                    {"convergence_thr": 1.5},
                    
                    #GrGSL
                    {"useDiffusionTerm": True},
                    {"stdevHit": 1.0},
                    {"stdevMiss": 1.2},
                    {"infoTaxis": parse_substitution("$(var use_infotaxis)")},
                    {"allowMovementRepetition": parse_substitution("$(var use_infotaxis)")},

                    #PMFS
                    {"headless": False},
                    {"proportionExpectedValue": 1.0}, #How many of the candidate source positions to include when calculating the expected value, as a proportion


                        # Hit probabilities
                    {"maxUpdatesPerStop": 5},
                    {"kernelSigma": 1.5},
                    {"kernelStretchConstant": 1.2},
                    {"hitPriorProbability": 0.05},
                    
                    {"confidencePrior": 0.0},
                    {"confidenceSigmaSpatial": 1.0},
                    {"confidenceMeasurementWeight": 0.5},
                    {"initialExplorationMoves" : parse_substitution("$(var initialExplorationMoves)")},
                    {"pHit" : 0.6}, # p(H | hit) -- needs to be > than the prior
                    {"pMiss" : 0.02}, # p(H | miss) -- needs to be < than the prior
                    {"confidenceGainOverTime": 0.002},
                        #Filament simulation
                    {"useWindGroundTruth": True},
                    {"stepsSourceUpdate": 3},
                    {"maxRegionSize": 7},
                    {"maxSimulationLevels": 0},
                    {"sourceDiscriminationPower": parse_substitution("$(var sourceDiscriminationPower)")},
                    {"refineFraction": 0.7},
                    {"deltaTime": parse_substitution("$(var filamentDeltaTime)")},
                    {"noiseSTDev": parse_substitution("$(var filament_movement_stdev)")},
                    {"iterationsToRecord": parse_substitution("$(var iterationsToRecord)")},
                    {"maxWarmupIterations": parse_substitution("$(var maxWarmupIterations)")},

                    # ICASSP
                    {"test_folder": "/home/pepe/Documents/test"}, #"/mnt/d/Projects/2024_GSL_Challenge_IEEE_ICASSP/train/test"
                    {"zMin": 0.26},
                    {"zMax": 0.43},


                    
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
            {"cell_size": 0.05},
        ]
    )

    rvizHit = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        #prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("pmfs_env"), "launch", "hit.rviz")
        ],
    )

    rvizSource = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        #prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("pmfs_env"), "launch", "source.rviz")
        ],
    )

    actions = []
    actions.extend(map_server)
    actions.append(gmrf_wind)
    actions.extend(gsl_node)
    actions.extend(gsl_call)
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


        # GSL params (overwritable in each YAML)
        ##############################################
        SetLaunchConfiguration(
            name="th_gas_present", 
            value="0.134"
        ),
        SetLaunchConfiguration(
            name="th_wind_present", 
            value="0.02"
        ),

        SetLaunchConfiguration(
            name="filament_movement_stdev", 
            value="0.4"
        ),
        SetLaunchConfiguration(
            name="sourceDiscriminationPower", 
            value="0.2"
        ),
        SetLaunchConfiguration(
            name="iterationsToRecord", 
            value="200"
        ),
        SetLaunchConfiguration(
            name="maxWarmupIterations", 
            value="500"
        ),
        SetLaunchConfiguration(
            name="initialExplorationMoves", 
            value="2"
        ),
        SetLaunchConfiguration(
            name="filamentDeltaTime", 
            value="0.1"
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)