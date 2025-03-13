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
            {"port": 15761},
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
            {"port": 15762},
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
            {"port": 15763},
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
            {"port": 15764},
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
                {"port": 15765},
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
                {"port": 15766},
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
                {"port": 15767},
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
            {"port": 15768},
            {"topic": "/giraff/amcl_pose"},
            {"isServerSocket": True},
        ],
    )

    cmd_vel = Node(
        package="nav2_transfer",
        executable="twistClient",
        prefix="xterm -hold -e ",
        parameters=[
            {"protocol": "UDP"},
            {"port": 15769},
            {"topic": "/giraff/cmd_vel"},
            {"isServerSocket": True},
        ],
    )

    rvizHit = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("semantic_gsl_env"), "launch", "hit.rviz")
        ],
    )

    rvizSource = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        # prefix="xterm -e",
        arguments=[
            "-d" + os.path.join(get_package_share_directory("semantic_gsl_env"), "launch", "source.rviz")
        ],
    )

    pose_publisher = Node(
        package="pose_from_tf",
        executable="pose_with_covariance",
        prefix="xterm -hold -e ",
        parameters=[
                {"frame_id": "giraff_base_link"},
                {"topic": "/giraff/pose"},
                {"frequency": 20.0},
        ],
    )

    olfaction_sensors = [
        Node(
            package="olfaction_msgs_transfer",
            executable="gas_sensor_server",
            prefix="xterm -hold -T pid -e ",
            parameters=[
                {"protocol": "UDP"},
                {"port": 15770},
                {"topic": "/giraff/PID/Sensor_reading"},
                {"isServerSocket": True},
            ],
        ),
        Node(
            package="olfaction_msgs_transfer",
            executable="anemometer_server",
            prefix="xterm -hold -T anemometer -e ",
            parameters=[
                {"protocol": "UDP"},
                {"port": 15771},
                {"topic": "/giraff/Anemometer/WindSensor_reading"},
                {"isServerSocket": True},
            ],
        )
    ]



    nodes = []
    nodes.append(tf)
    nodes.append(map)
    nodes.append(nav2)
    nodes.append(rvizTest)
    nodes.append(initialPose)
    nodes.append(laser)
    nodes.extend(camera)
    nodes.append(amcl)
    nodes.append(cmd_vel)
    nodes.append(rvizHit)
    nodes.append(pose_publisher)
    nodes.extend(olfaction_sensors)
    # nodes.append(rvizSource)

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
