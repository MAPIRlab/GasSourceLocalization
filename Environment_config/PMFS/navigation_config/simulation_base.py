"""
	Launch file to run the GADEN-preprocessing node.
	This is mandatory before running GADEN simulator.
	From the CAD models in the selected scenario, it generates a 3D and 2D occupancy gridmap
	that will be employed by GADEN and nav2 to simulate dispersion and navigation. Moreover parses
	the 3D cloud of wind-vectors obtained from CFD, to a grid-based format.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.frontend.parse_substitution import parse_substitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def launch_arguments():
	return [
		# ==================
			DeclareLaunchArgument(
				"scenario",	default_value=["A"],
			),
	]

def launch_setup(context, *args, **kwargs):

	basic_sim = Node(
		package="basic_sim",
		executable="basic_sim",
		#prefix = "xterm -e gdb --args",
		parameters=[
			{"deltaTime": 0.1},
			{"speed": 5.0},
			{"worldFile": parse_substitution("$(find-pkg-share pmfs_env)/B/sim.yaml")}
			],
	)

	coppelia = [
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
					os.path.join(
						get_package_share_directory("pmfs_env"),
						"navigation_config/conditional_coppelia_launch.py",
					)
				),
				launch_arguments={
					"launchCoppelia":"True",
					"scenePath" : parse_substitution("$(find-pkg-share pmfs_env)/$(var scenario)/coppeliaScene.ttt"),
					"autoplay" : "True",
					"headless" : "True"
				}.items(),
		),
		Node(
			package="gaden_preprocessing",
			executable="configureCoppeliaSim",
			name="configureCoppeliaSim",
			condition=IfCondition(LaunchConfiguration("launchCoppelia")),
			parameters=[
				{"permanentChange" : False},
				{"robotName" : LaunchConfiguration("robot_name")},
				{"simulationSpeed" : 5.0}
            ],
		)
	]
	
	rviz = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz",
		#prefix="xterm -e",
		arguments=[
			"-d" + os.path.join(get_package_share_directory("pmfs_env"), "launch", "gaden.rviz")
		],
	)
	

	returnList = []
	#returnList.extend(coppelia)
	#returnList.append(basic_sim)
	returnList.append(rviz)

	return returnList



def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        
		SetLaunchConfiguration(
            name="robot_name",
            value=["PioneerP3DX"],
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)