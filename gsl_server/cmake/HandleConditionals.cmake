# Check the options and apply them

add_compile_definitions("GSL_DEBUG=$<CONFIG:Debug>")

if(GSL_TRACING)
    add_compile_definitions(GSL_TRACING=1)
endif(GSL_TRACING)


if(USE_NAV_ASSISTANT)
    add_compile_definitions(USE_NAV_ASSISTANT=1)
	find_package(nav_assistant_msgs REQUIRED)
    set(CONDITIONAL_NAV_ASSISTANT "nav_assistant_msgs")  
endif()


if(USE_TRACY)
    find_package(Tracy)
    add_compile_definitions(TRACY_ENABLE)
    if(USE_TRACY_INSTRUMENTATION)
        add_compile_definitions(TRACY_INSTRUMENTATION)
    endif()
    set(CONDITIONAL_TRACY "Tracy::TracyClient")
else()
endif(USE_TRACY)


if(USE_GUI)
	add_compile_definitions(USE_GUI=1)
    include(cmake/ament_imgui.cmake)
    set(CONDITIONAL_IMGUI "imgui_gl")
endif(USE_GUI)

if(USE_GADEN)
    add_compile_definitions(USE_GADEN=1)
    find_package(gaden_msgs REQUIRED)
    set(CONDITIONAL_GADEN_MSGS "gaden_msgs")  
else()
    set(CONDITIONAL_GADEN_MSGS "")  
endif(USE_GADEN)

if(DISABLE_NAVIGATION)
    add_compile_definitions(DISABLE_NAVIGATION=1)
endif()