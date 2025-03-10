import os
from enum import Enum
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

class SegmentationNN(Enum):
    YOLO = 1
    DETECTRON = 2

def toName(nnEnum):
    if nnEnum == SegmentationNN.YOLO:
        return "yolo"
    elif nnEnum == SegmentationNN.DETECTRON:
        return "detectron"
    

def launch_setup(context, *args, **kwargs):

    # Select the segmentation network you want to use here! From here: https://github.com/MAPIRlab/instance_segmentation
    ######################################################
    segmentation_net = SegmentationNN.YOLO 


    voxeland_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file_path_from_package(package_name="voxeland", file_name="voxeland_server.launch.py")
        ),
        launch_arguments= {
            "resolution" : "0.25",
            "pHit" : "0.6",
            "pMiss" : "0.4",
            "clampOccupancyMax" : "0.97",
            "clampOccupancyMin" : "0.12",
        }.items()
    )

    voxeland_robot_simulation = IncludeLaunchDescription(
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

            "service_name": f"/{toName(segmentation_net)}/segment",
        }.items()
    )

    voxeland_robot_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file_path_from_package(package_name="voxeland_robot_perception", file_name="semantic_mapping.launch.py")
        ),
        launch_arguments={
            "dataset" : "ROS-Unity", #TODO

            "topic_camera_info" : "/giraff/camera/color/camera_info",
            "topic_rgb_image" : "/giraff/camera/color/image_compressed",
            "topic_depth_image" : "/giraff/camera/depth/image_compressed",
            "topic_localization" : "/giraff/amcl_pose",
            
            "map_frame_id" : "map",
            "robot_frame_id" : "giraff_base_link",
            "camera_frame_id" : "camera_link",

            "service_name": f"/{toName(segmentation_net)}/segment",
        }.items()
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
    
    detectron = Node(
            package="detectron_ros",
            executable="detectron_ros_node",
            prefix = "xterm -hold -e",
            parameters=[],
        )
    

    yolo = Node(
            package="yolo_ros",
            name="yolo",
            executable="yolo_ros.py",
            prefix = "xterm -hold -e",
            parameters=[
                {"model_file": "yolo11x-seg.pt"},
            ],
        )
    
    segmentationNode = None
    if segmentation_net == SegmentationNN.YOLO:
        segmentationNode = yolo
    elif segmentation_net == SegmentationNN.DETECTRON:
        segmentationNode = detectron

    return [
        voxeland_server,
        voxeland_robot_simulation,
        segmentationNode
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