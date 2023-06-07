cmake_minimum_required(VERSION 3.11)
project(gsl_server)
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp -g ${CMAKE_CXX_FLAGS}")



#OPTIONS
##############################################################################
set(GSL_DEBUG OFF) #debug mode disables optimizations and activates a bunch of assertions in the code.

set(USE_GADEN OFF) #if ON, the code is compiled with support for directly querying gaden simulations to obtain the wind ground truth. If you dont have gaden in this workspace, you need to set this to OFF or it wont compile (duh)

# Compile graphical interface with DearImGUI. Currently only affects the PMFS algorithm. Not required to function, it is mostly for debugging purposes
set(USE_GUI ON)

##############################################################################



















if(USE_GUI)
  # You can change these two lines to use a different rendering API and window manager combo for imgui
  # but you will also need to provide the boilerplate code in the gsl_imgui.cpp file, which only implements this combo
  set(IMGUI_BACKEND 
  "third_party/imgui/backends/imgui_impl_glfw.cpp")
  list(APPEND IMGUI_BACKEND 
  "third_party/imgui/backends/imgui_impl_opengl3.cpp")
  add_definitions(-DUSE_GUI)
  add_definitions(-DIMGUI_INI_PATH="${CMAKE_CURRENT_SOURCE_DIR}/imgui.ini") #imgui.ini file in the same directory as this CMakeLists.txt
endif(USE_GUI)

if(GSL_DEBUG)
  add_definitions(-DGSL_DEBUG)
  set(CMAKE_CXX_FLAGS "-Og ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(GSL_DEBUG)



#ament packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(gmrf_wind_mapping REQUIRED)
find_package(navigation_assistant REQUIRED)
find_package(gsl_actions REQUIRED)



if(USE_GADEN)
add_definitions(-DUSE_GADEN)
find_package(gaden_player REQUIRED)
endif(USE_GADEN)
set(CONDITIONAL_GADEN_PLAYER "")  




#other stuff
find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED)

if(USE_GUI)
  add_subdirectory(third_party/glfw)
  set(CONDITIONAL_GLFW "glfw")
endif(USE_GUI)



###########
## INCLUDES
###########

include_directories(
  include
  Utils
  
  third_party/imgui
  third_party/imgui/backends

  third_party/implot
)

###########
## SOURCES
###########

if(USE_GUI)  
  SET(IMGUI_FILES
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/imgui.cpp
    
    ${IMGUI_BACKEND}
  )
    
  SET(IMPLOT_FILES
    third_party/implot/implot.cpp
    third_party/implot/implot_items.cpp
  )
endif(USE_GUI)
  
set(SOURCES_node 
  Utils/Utils/Utils.cpp
  Utils/NQA/NQAQuadtree.cpp

  src/gsl_server.cpp 
  src/gsl_imgui.cpp

  src/algorithms/gsl_algorithm.cpp 
  src/algorithms/gsl_spiral.cpp 
  src/algorithms/gsl_plume_tracking.cpp 
  src/algorithms/gsl_surge_cast.cpp 
  src/algorithms/gsl_surge_spiral.cpp 
  src/algorithms/gsl_particle_filter.cpp 
  src/algorithms/gsl_GrGSL.cpp 
  
  src/algorithms/PMFS/pmfs_simulations.cpp
  src/algorithms/PMFS/pmfs_UI.cpp
    src/algorithms/PMFS/PMFS_GSL/pmfs_core.cpp
    src/algorithms/PMFS/PMFS_GSL/pmfs_callbacks.cpp
    src/algorithms/PMFS/PMFS_GSL/pmfs_initialization.cpp
    src/algorithms/PMFS/PMFS_GSL/pmfs_movement.cpp
    src/algorithms/PMFS/PMFS_GSL/pmfs_visualization.cpp
    src/algorithms/PMFS/PMFS_GSL/pmfs_utils.cpp

  ${IMGUI_FILES}  
  ${IMPLOT_FILES}
)

###########
#DECLARE EXECUTABLES
###########

add_executable(gsl_actionserver_node ${SOURCES_node})
add_executable(gsl_actionserver_call src/gsl_server_call.cpp)

add_dependencies(gsl_actionserver_node ${CONDITIONAL_GLFW})

ament_target_dependencies( gsl_actionserver_node
    rclcpp
    rclcpp_action
    tf2
    tf2_ros
    tf2_geometry_msgs
    olfaction_msgs
    gmrf_wind_mapping
    navigation_assistant
    gsl_actions
)

ament_target_dependencies( gsl_actionserver_call
    rclcpp
    rclcpp_action
    gsl_actions
)

###########
#LINK
###########
target_link_libraries(gsl_actionserver_node  
  ${OpenCV_LIBS}
  spdlog::spdlog
  ${CMAKE_DL_LIBS}
)


#optional link libs
if(USE_GUI)
  target_link_libraries(gsl_actionserver_node
  glfw
  OpenGL
  )
endif(USE_GUI)


install(
  TARGETS gsl_actionserver_node  gsl_actionserver_call
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()