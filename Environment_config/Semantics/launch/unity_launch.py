import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

#===========================
def launch_arguments():
    return [
        #DeclareLaunchArgument("", default_value=""),
   ]
#==========================

def launch_setup(context, *args, **kwargs):
    tcp_endpoint = Node(
                package="ros_tcp_endpoint",
                name="tcp_endpoint",
                executable="default_server_endpoint",
                emulate_tty=True,
                parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
            )
    
    keyboard_control = Node(
            package="keyboard_control",
            executable="keyboard_control_plus",
            prefix = "xterm -e",
            parameters=[
                {"linear_v_inc": 0.1},
                {"angular_v_inc": 0.1},
                {"publish_topic": "/giraff/cmd_vel"}
                ],
        )

    
    detectron = Node(
            package="detectron_ros",
            executable="detectron_ros_node",
            prefix = "xterm -hold -e",
            parameters=[],
        )
        
    return [
        tcp_endpoint,
        keyboard_control,
        detectron
    ]


def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)