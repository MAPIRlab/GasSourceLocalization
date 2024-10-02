import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction,Shutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from ros2launch.api import get_share_file_path_from_package

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="B"),
        DeclareLaunchArgument("simulation", default_value="B1"),
        DeclareLaunchArgument("method",	default_value=["SemanticPMFS"]),
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration("method").perform(context)
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
                #prefix="xterm -hold -e gdb --args",
                parameters=[
                    # Common
                    {'use_sim_time': False},	
                    {"maxSearchTime": 300.0},
                    {"robot_location_topic": "ground_truth"},
                    {"stop_and_measure_time": 0.4},
                    {"th_gas_present": parse_substitution("$(var th_gas_present)")},
                    {"th_wind_present": parse_substitution("$(var th_wind_present)")},
                    {"ground_truth_x": parse_substitution("$(var source_x)")},
                    {"ground_truth_y": parse_substitution("$(var source_y)")},
                    {"resultsFile": parse_substitution("Results/$(var simulation)/$(var method).csv")},
                    
                    {"scale": 25},
                    {"markers_height": 0.2},

                    {"anemometer_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
                    {"openMoveSetExpasion": 5},
                    {"explorationProbability": 0.05},
                    {"convergence_thr": 1.5},
                    
                    #GrGSL
                    {"useDiffusionTerm": True},
                    {"stdevHit": 1.0},
                    {"stdevMiss": 1.2},
                    {"infoTaxis": False},

                    #PMFS
                        # Hit probabilities
                    {"headless": False},
                    {"maxUpdatesPerStop": 5},
                    {"kernelSigma": 1.5},
                    {"kernelStretchConstant": 1.5},
                    {"hitPriorProbability": 0.3},
                    {"confidenceSigmaSpatial": 1.0},
                    {"confidenceMeasurementWeight": 1.0},
                    {"initialExplorationMoves" : parse_substitution("$(var initialExplorationMoves)")},
                        #Filament simulation
                    {"useWindGroundTruth": True},
                    {"stepsSourceUpdate": 3},
                    {"maxRegionSize": 5},
                    {"sourceDiscriminationPower": parse_substitution("$(var sourceDiscriminationPower)")},
                    {"refineFraction": 0.1},
                    {"deltaTime": parse_substitution("$(var filamentDeltaTime)")},
                    {"noiseSTDev": parse_substitution("$(var filament_movement_stdev)")},
                    {"iterationsToRecord": parse_substitution("$(var iterationsToRecord)")},
                    {"maxWarmupIterations": parse_substitution("$(var maxWarmupIterations)")},

                    #Semantics
                    {"semanticsType" : "ClassMapVoxeland"},
                    {"wallsOccupancyFile": os.path.join(scenario_folder, "_occupancy_walls.pgm" )},
                    {"detectionsTopic": "/semantic_instances_3D"},
                    {"ontologyPath": os.path.join(get_package_share_directory("gsl_server"), "resources", "ontology.yaml")},
                    {"targetGas" : "smoke"},
                    {"masksYAMLPath" : os.path.join(scenario_folder, "room_categories", "roomMasks.yaml" )},
                    {"roomOntologyPath" : os.path.join(get_package_share_directory("gsl_server"), "resources", "ObjectProbByRoom.yaml")},
                    #ClassMap2D
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
        parameters=[
            {"sensor_topic": parse_substitution("$(var robot_name)/Anemometer/WindSensor_reading")},
            {"map_topic": parse_substitution("$(var robot_name)/map")},
            {"cell_size": 0.25},
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("semantic_gsl_env"),
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        #prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("semantic_gsl_env"), "launch", "gaden.rviz")
        ],
    )

    send_pose = Node(
        package="gsl_server",
        executable="send_pose",
        parameters=[
            {"x":-3.4},
            {"y":-3.5},
            {"z":-0.73},
            {"topic":"/giraff/resetPose"}
        ]
    )

    semantics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_share_file_path_from_package(package_name="semantic_gsl_env", file_name="semantics_launch.py"))
    )

    actions = []
    actions.append(gaden_player)
    actions.extend(anemometer)
    actions.extend(PID)
    actions.append(nav2)
    actions.append(gmrf_wind)
    actions.extend(gsl_node)
    actions.extend(gsl_call)
    actions.append(rviz)
    actions.append(send_pose)
    actions.append(semantics)

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
            value="0.5"
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
            value="5"
        ),
        SetLaunchConfiguration(
            name="filamentDeltaTime", 
            value="0.1"
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)