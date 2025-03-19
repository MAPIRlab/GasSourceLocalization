#ament packages
set(COMMON_AMENT_DEPENDENCIES 
    rclcpp
    rclcpp_action
    tf2
    tf2_ros
    tf2_geometry_msgs
    angles
    olfaction_msgs
    visualization_msgs
    geometry_msgs
    std_msgs
    nav2_msgs
    gsl_actions
)

foreach(Dependency IN ITEMS ${COMMON_AMENT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

#other common stuff
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)