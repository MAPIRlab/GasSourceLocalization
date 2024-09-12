cmake_minimum_required(VERSION 3.11)
project(gsl_server) 
set(CMAKE_CXX_FLAGS "-std=c++20 -g -Wno-narrowing ${CMAKE_CXX_FLAGS}")



#OPTIONS
##############################################################################
set(GSL_DEBUG ON) #enabling debug mode disables optimizations and activates a bunch of assertions in the code.
set(GSL_TRACING ON) #extra logging to tell you what is being executed

set(USE_GADEN ON) #if ON, the code is compiled with support for directly querying gaden simulations to obtain the wind ground truth. If you dont have gaden in this workspace, you need to set this to OFF or it wont compile (duh)

set(USE_NAV_ASSISTANT OFF) # if ON uses the nav_assistant package rather than directly communicating with nav_2. It adds some extra features, but currently it has a relatively high performance cost due to async inter-process communication  

# Compile graphical interface with DearImGUI. Currently only affects the PMFS algorithm. Not required to function, it is mostly for debugging purposes
set(USE_GUI ON)

set(USE_TRACY OFF) # Tracy is a profiler. ON enables some instrumentation macros (and adds a dependency to tracy). To do sampling-based profiling you also need to run the node as sudo
set(USE_TRACY_INSTRUMENTATION OFF) #disable to do sampling-only profiling
##############################################################################




 











if(GSL_DEBUG)
  set(CMAKE_BUILD_TYPE "Debug")
  add_definitions(-DGSL_DEBUG=1)
  set(CMAKE_CXX_FLAGS "-O0 ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(GSL_DEBUG)



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

#other stuff
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)



if(GSL_TRACING)
    add_definitions(-DGSL_TRACING=1)
endif(GSL_TRACING)


if(USE_NAV_ASSISTANT)
    add_definitions(-DUSE_NAV_ASSISTANT=1)
	find_package(nav_assistant_msgs REQUIRED)
    set(CONDITIONAL_NAV_ASSISTANT "nav_assistant_msgs")  
endif()


if(USE_TRACY)
    find_package(Tracy)
    add_definitions(-DTRACY_ENABLE)
    if(USE_TRACY_INSTRUMENTATION)
        add_definitions(-DTRACY_INSTRUMENTATION)
    endif()
    set(CONDITIONAL_TRACY "Tracy::TracyClient")
else()
endif(USE_TRACY)


if(USE_GUI)
	add_definitions(-DUSE_GUI=1)
    find_package(ament_imgui REQUIRED)
    set(CONDITIONAL_IMGUI "ament_imgui")
endif(USE_GUI)



###########
## INCLUDES
###########

include_directories(
  src
  third_party/xxHash/include
  third_party/DDA/include
  third_party/DDA/third_party/glm
  third_party/StateMachine/include
)

###########
#EXECUTABLES
###########

#### Server node
##############
add_executable(gsl_actionserver_node src/gsl_server/gsl_server.cpp)
ament_target_dependencies( gsl_actionserver_node
    rclcpp
    rclcpp_action
    gsl_actions
)

add_subdirectory(src/gsl_server/algorithms/Common)
add_subdirectory(src/gsl_server/algorithms/PlumeTracking)
add_subdirectory(src/gsl_server/algorithms/Spiral)
add_subdirectory(src/gsl_server/algorithms/ParticleFilter)
add_subdirectory(src/gsl_server/algorithms/GrGSL)
add_subdirectory(src/gsl_server/algorithms/PMFS)

target_link_libraries(gsl_actionserver_node
    PlumeTracking
    Spiral
    ParticleFilter
    GrGSL
    PMFS
)



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


install(
  TARGETS gsl_actionserver_node  gsl_actionserver_call send_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY resources
	DESTINATION share/${PROJECT_NAME}
)
ament_package()