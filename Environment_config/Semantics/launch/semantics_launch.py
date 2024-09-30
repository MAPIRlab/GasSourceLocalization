import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from ros2launch.api import get_share_file_path_from_package

#===========================
def launch_arguments():
    return [
        #DeclareLaunchArgument("", default_value=""),
   ]
#==========================

def launch_setup(context, *args, **kwargs):
    voxeland_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file_path_from_package(package_name="voxeland", file_name="voxeland_server.launch.py")
        ),
        launch_arguments= {
            "resolution" : "0.25"
        }.items()
    )

    voxeland_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file_path_from_package(package_name="voxeland_robot_perception", file_name="semantic_mapping.launch.py")
        ),
        launch_arguments={
            "dataset" : "ROS-Unity",

            "topic_camera_info" : "/rgbd/info",
            "topic_rgb_image" : "/rgbd/color/raw",
            "topic_depth_image" : "/rgbd/depth/raw",
            "topic_localization" : "/giraff/ground_truth",
            
            "map_frame_id" : "map",
            "robot_frame_id" : "giraff_base_link",
            "camera_frame_id" : "RGBD",
        }.items()
    )

    unity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file_path_from_package(package_name="semantic_gsl_env", file_name="unity_launch.py")
        ),
        launch_arguments={}.items()
    )

    # For the 2D ClassMap
    projectTo3D = Node(
            package="instance_segmentation_utils",
            executable="projectTo3D",
            name="projectTo3D",
            #prefix = "xterm -e",
            parameters=[
                {"color_topic": "/rgbd/color/raw"},
                {"depth_topic": "/rgbd/depth/raw"},
                {"info_topic": "/rgbd/info"},
                {"depth_format": "mono16"}
                ],
        )
        
    return [
        #unity, 
        voxeland_server,
        voxeland_robot
    ]

#==========================





def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)