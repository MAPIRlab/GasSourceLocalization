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
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
	launchCoppelia = (
		LaunchConfiguration("launchCoppelia").perform(context) == "True"
	)

	returnList = []
	
	if launchCoppelia:
		returnList.append(
			IncludeLaunchDescription(
				FrontendLaunchDescriptionSource(
					os.path.join(
						get_package_share_directory("coppelia_ros2_pkg"),
						"launch/coppeliaSim.launch",
					)
				),
				launch_arguments={
					"coppelia_scene_path": LaunchConfiguration("scenePath").perform(context),
					"coppelia_headless": LaunchConfiguration("headless").perform(context),
					"autoplay": LaunchConfiguration("autoplay").perform(context),
				}.items(),
			)
		)

	return returnList


def generate_launch_description():
	pkg_dir = get_package_share_directory("pmfs_env")

	return LaunchDescription(
		[
			# Set env var to print messages to stdout immediately
			SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
			SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
			
			# ==================
			DeclareLaunchArgument(
				"launchCoppelia",
				default_value=["True"],  # debug, info
				description="Whether to launch the node",
			),
			DeclareLaunchArgument(
				"autoplay",
				default_value=["True"],  # debug, info
				description="Should the simulation start playing on launch?",
			),
			DeclareLaunchArgument(
				"headless",
				default_value=["True"],  # debug, info
				description="Run coppelia in headless mode",
			),
			DeclareLaunchArgument(
				"scenePath",
				default_value=[os.path.join( pkg_dir,
								"navigation_config",
								"resources",
								"default_coppelia_scene.ttt")],
				description="Path to the coppelia scene",
			),
			OpaqueFunction(function=launch_setup),
		]
	)
