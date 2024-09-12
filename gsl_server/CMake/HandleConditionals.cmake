# Check the options and apply them


if(GSL_DEBUG)
  set(CMAKE_BUILD_TYPE "Debug")
  add_definitions(-DGSL_DEBUG=1)
  set(CMAKE_CXX_FLAGS "-O0 ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(GSL_DEBUG)


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

if(USE_GADEN)
    add_definitions(-DUSE_GADEN=1)
    find_package(gaden_player REQUIRED)
    set(CONDITIONAL_GADEN_PLAYER "gaden_player")  
else()
    set(CONDITIONAL_GADEN_PLAYER "")  
endif(USE_GADEN)