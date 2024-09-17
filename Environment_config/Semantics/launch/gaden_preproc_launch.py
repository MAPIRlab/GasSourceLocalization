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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

# Internal gaden utilities
import sys
sys.path.append(get_package_share_directory('gaden_common'))
from gaden_internal_py.utils import read_sim_yaml



#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument(
            "scenario",
            default_value=["A"],
            description="scenario to preprocess",
        ),
        DeclareLaunchArgument(
            "simulation",
            default_value=["A1"],
            description="name of the simulation yaml file",
        )
    ]
#==========================


def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    pkg_dir = LaunchConfiguration("pkg_dir").perform(context)

    params_yaml_file = os.path.join(
        pkg_dir, "scenarios", scenario, "params", "preproc_params.yaml"
    )
    
    read_sim_yaml(context)
    

    ## NODES
    preprocessing = Node(
            package="gaden_preprocessing",
            executable="preprocessing",
            name="gaden_preprocessing",
            output="screen",
            prefix="",
            parameters=[
                ParameterFile(params_yaml_file, allow_substs=True),
                {"generateCoppeliaScene": False},
            ],
    )
  
    returnList = [preprocessing]

    return returnList


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),

        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("semantic_gsl_env")],
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)