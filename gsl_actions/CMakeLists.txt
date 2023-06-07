cmake_minimum_required(VERSION 3.8)
project(gsl_actions)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)


# services and actions
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    action/DoGSL.action
    DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)



ament_package()
