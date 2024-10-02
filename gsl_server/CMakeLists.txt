cmake_minimum_required(VERSION 3.11)
project(gsl_server) 
set(CMAKE_CXX_FLAGS "-g -Wno-narrowing ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_VERBOSE_MAKEFILE ON) #Outputs the compiler command to the build log file

#########################
#OPTIONS
#########################
#enabling debug mode disables optimizations and activates a bunch of assertions in the code.
set(GSL_DEBUG ON) 
#extra logging to tell you what is being executed
set(GSL_TRACING ON) 
 
#if ON, the code is compiled with support for directly querying gaden simulations to obtain the wind ground truth. 
#If you dont have gaden in this workspace, you need to set this to OFF or it wont compile
set(USE_GADEN ON)

# if ON uses the nav_assistant package rather than directly communicating with nav_2. 
#It adds some extra features, but currently it has a relatively high performance cost due to async inter-process communication  
set(USE_NAV_ASSISTANT OFF) 

# Compile graphical interface with DearImGUI. Currently only affects the PMFS algorithm. Not required to function, it is mostly for debugging purposes
set(USE_GUI ON)


# Tracy is a profiler. ON enables some instrumentation macros (and adds a dependency to tracy). To do sampling-based profiling you also need to run the node as sudo
set(USE_TRACY OFF) 
#disable to do sampling-only profiling
set(USE_TRACY_INSTRUMENTATION OFF) 
##############################################################################


include(CMake/HandleConditionals.cmake)
include(CMake/CommonDependencies.cmake)


#########################
## INCLUDES
#########################

include_directories(
  src
  third_party/xxHash/include
  third_party/DDA/include
  third_party/DDA/third_party/glm
  third_party/StateMachine/include
)

##########################
#EXECUTABLES
##########################

#### Server node
##############
add_executable(gsl_actionserver_node src/gsl_server/gsl_server.cpp)
ament_target_dependencies( gsl_actionserver_node
    rclcpp
    rclcpp_action
    gsl_actions
)
# You can disable the compilation of any of these algorithms by commenting them out here (useful if you don't want to install their dependencies)
include(src/gsl_server/algorithms/Common/CMakeLists.txt)
include(src/gsl_server/algorithms/PlumeTracking/CMakeLists.txt)
include(src/gsl_server/algorithms/Spiral/CMakeLists.txt)
include(src/gsl_server/algorithms/ParticleFilter/CMakeLists.txt)
include(src/gsl_server/algorithms/GrGSL/CMakeLists.txt)
include(src/gsl_server/algorithms/PMFS/CMakeLists.txt)
include(src/gsl_server/algorithms/Semantics/CMakeLists.txt)


##### Call node
add_executable(gsl_actionserver_call src/aux_nodes/gsl_server_call.cpp)
ament_target_dependencies( gsl_actionserver_call
    rclcpp
    rclcpp_action
    gsl_actions
)

#### Send pose node
add_executable(send_pose src/aux_nodes/send_pose.cpp)
ament_target_dependencies( send_pose
    rclcpp
    geometry_msgs
)


##########################
# INSTALLATION
##########################
install(
  TARGETS gsl_actionserver_node  gsl_actionserver_call send_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY resources
	DESTINATION share/${PROJECT_NAME}
)
ament_package()