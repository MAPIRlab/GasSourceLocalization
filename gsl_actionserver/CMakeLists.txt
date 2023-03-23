cmake_minimum_required(VERSION 3.11)
project(gsl_actionserver)
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp ${CMAKE_CXX_FLAGS}")



#OPTIONS
##############################################################################
set(GSL_DEBUG OFF) #debug mode disables all optimizations and activates a bunch of assertions in the code.

set(USE_GADEN OFF) #if ON, the code is compiled with support for directly querying gaden simulations to obtain the wind ground truth. If you dont have gaden in this workspace, you need to set this to OFF or it wont compile (duh)

#profiler (tracy)
set(TRACY_ENABLE OFF) # compiles and links with support for the profiler Tracy. If you disable this option, you dont need to install tracy at all
set(TRACY_SAMPLING OFF) # if you want to use sample profiling, you'll need to run as root. Use sudo -s -E before running roslaunch


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
  set(CMAKE_CXX_FLAGS "-g -Og ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(GSL_DEBUG)


if(TRACY_ENABLE)
  add_definitions(-DTRACY_ENABLE)

  if(TRACY_SAMPLING)
    add_definitions(-DTRACY_CALLSTACK)
  else()
    add_definitions(-DTRACY_NO_CALLSTACK)
    add_definitions(-DTRACY_NO_SAMPLING)
    set(CMAKE_CXX_FLAGS "-g -fno-omit-frame-pointer ${CMAKE_CXX_FLAGS}")
  endif(TRACY_SAMPLING)

endif(TRACY_ENABLE)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    genmsg
    actionlib_msgs
    actionlib
    nav_msgs
    move_base
    tf
    )
    
## is used, also find other catkin packages
## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  gsl_action_msg.action
)
  
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs sensor_msgs # Or other packages containing msgs
)


if(USE_GADEN)
  add_definitions(-DUSE_GADEN)
  set(CONDITIONAL_GADEN_PLAYER "gaden_player")  
endif(USE_GADEN)
  set(CONDITIONAL_GADEN_PLAYER "")  


catkin_package(
  DEPENDS CATKIN
  CATKIN_DEPENDS gmrf_wind_mapping olfaction_msgs ${CONDITIONAL_GADEN_PLAYER}
)








## Find packages
find_package(OpenCV)
add_subdirectory(third_party/spdlog)

if(USE_GUI)
  add_subdirectory(third_party/glfw)
  set(CONDITIONAL_GLFW)
endif(USE_GUI)



###########
## INCLUDES
###########

include_directories(
  include
  include/algorithms
  Utils
  Utils/segmentation/src
  Utils/Quadtree
  include/algorithms/PMFS
  ${catkin_INCLUDE_DIRS}
  
  third_party/imgui
  third_party/imgui/backends

  third_party/implot
  third_party/spdlog/include

  third_party/StateMachines/include
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
  Utils/Quadtree/NQAQuadtree.cpp

  src/gsl_server.cpp 
  src/gsl_imgui.cpp

  src/algorithms/gsl_algorithm.cpp 
  src/algorithms/gsl_spiral.cpp 
  src/algorithms/gsl_plume_tracking.cpp 
  src/algorithms/gsl_surge_cast.cpp 
  src/algorithms/gsl_surge_spiral.cpp 
  src/algorithms/gsl_particle_filter.cpp 
  src/algorithms/gsl_grid.cpp 
  
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
add_dependencies(gsl_actionserver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} spdlog ${CONDITIONAL_GLFW})
add_dependencies(gsl_actionserver_call ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


###########
#LINK
###########
target_link_libraries(gsl_actionserver_call
  ${catkin_LIBRARIES}
)

target_link_libraries(gsl_actionserver_node  
  ${catkin_LIBRARIES}
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

if(TRACY_ENABLE)
  target_link_libraries(gsl_actionserver_node 
    TracyClient
  )
endif(TRACY_ENABLE)

