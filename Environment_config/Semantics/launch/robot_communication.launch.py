import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

# ===========================


def launch_arguments():
    return [
        DeclareLaunchArgument("", default_value=""),
    ]
# ==========================


def launch_setup(context, *args, **kwargs):
    tf = Node(
        package="tf_transfer",
        executable="tf_server",
        name="server",
        prefix="xterm -T tf -hold -e",
        parameters=[
            {"protocol": "UDP"},
            {"port": 15760},
            {"topic": "/tf"},
            {"isServerSocket": True},
        ],
    )

    map = Node(
        package="nav2_transfer",
        executable="mapServer",
        prefix="xterm -hold -e",
        parameters=[
            {"protocol": "TCP"},
            {"port": 15770},
            {"topic": "/giraff/map"},
            {"isServerSocket": True},
        ],
    )

    nav2 = Node(
        package="nav2_transfer",
        executable="navToPoseClient",
        prefix="xterm -hold -e",
        parameters=[
            {"protocol": "TCP"},
            {"port": 15780},
            {"actionServer": "/giraff/navigate_to_pose"},
            {"isServerSocket": True},
        ],
    )

    rvizTest = Node(
        package="nav2_transfer",
        executable="rvizTest",
        prefix="xterm -hold -e ",
        parameters=[
            {"topic": "/giraff/goal_pose"},
            {"actionServer": "/giraff/navigate_to_pose"}
        ],
    )

    initialPose = Node(
        package="tf_transfer",
        executable="poseWithCovarianceStamped_client",
        prefix="xterm -hold -e ",
        parameters=[
            {"protocol": "TCP"},
            {"port": 15790},
            {"topic": "/giraff/initialpose"},
            {"isServerSocket": True},
        ],
    )

    laser = Node(
        package="laser_scan_transfer",
        executable="server",
        prefix="xterm -hold -T laser -e",
        parameters=[
            {"protocol": "UDP"},
            {"port": 15800},
            {"topic": "/giraff/laser_scan"},
            {"isServerSocket": True},
        ],
    )

    camera = [
        Node(
            package="image_transfer",
            executable="server_info",
            prefix="xterm -hold -T cameraInfo -e",
            parameters=[
                {"protocol": "UDP"},
                {"port": 15801},
                {"topic": "/giraff/camera/color/camera_info"},
                {"isServerSocket": True},
            ],
        ),
        Node(
            package="image_transfer",
            executable="server_compressed",
            prefix="xterm -hold -T rgb -e",
            parameters=[
                {"protocol": "UDP"},
                {"port": 15802},
                {"topic": "/giraff/camera/color/image_compressed"},
                {"isServerSocket": True},
            ],
        ),
        Node(
            package="image_transfer",
            executable="server_compressed",
            prefix="xterm -hold -T depth -e",
            parameters=[
                {"protocol": "UDP"},
                {"port": 15803},
                {"topic": "/giraff/camera/depth/image_compressed"},
                {"isServerSocket": True},
            ],
        )
    ]

    amcl = Node(
        package="tf_transfer",
        executable="poseWithCovarianceStamped_server",
        prefix="xterm -hold -T amcl -e ",
        parameters=[
            {"protocol": "TCP"},
            {"port": 15791},
            {"topic": "/giraff/amcl_pose"},
            {"isServerSocket": True},
        ],
    )


    nodes = []
    nodes.append(tf)
    nodes.append(map)
    nodes.append(nav2)
    nodes.append(rvizTest)
    nodes.append(initialPose)
    nodes.append(laser)
    nodes.extend(camera)
    nodes.append(amcl)

    return nodes


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description)
