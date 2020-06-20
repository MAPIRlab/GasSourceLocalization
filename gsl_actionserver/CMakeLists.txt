cmake_minimum_required(VERSION 2.8.3)
project(gsl_actionserver)
set (CMAKE_CXX_STANDARD 14)  # Require C++14
set(CMAKE_CXX_FLAGS "-g -std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## Find actionlib required stuff
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    genmsg
    actionlib_msgs
    actionlib
    nav_msgs
    move_base
    tf
    )

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES
   gsl_action_msg.action
)

## Generate services
#add_service_files(
#  DIRECTORY srv
#  FILES
#  MDPInference.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs  std_msgs sensor_msgs # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(DEPENDS CATKIN)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autodocking
#   CATKIN_DEPENDS actionlib_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gsl_actionserver_node src/gsl_server.cpp src/gsl_algorithm.cpp src/gsl_server.cpp src/gsl_spiral.cpp src/gsl_plume_tracking.cpp src/gsl_surge_cast.cpp src/gsl_surge_spiral.cpp src/gsl_particle_filter.cpp src/gsl_grid.cpp)
add_executable(gsl_actionserver_call src/gsl_server_call.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(gsl_actionserver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gsl_actionserver_call ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gsl_actionserver_node  
  ${catkin_LIBRARIES}
)
target_link_libraries(gsl_actionserver_call
  ${catkin_LIBRARIES}
)

