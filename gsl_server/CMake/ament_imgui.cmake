# if building through ROS, we might already have ament_imgui somewhere in the workspace
# if so, just create an interface target for easy linking
# otherwise, download from git
find_package(ament_imgui QUIET)
if(ament_imgui_FOUND)

    add_library(imgui_gl INTERFACE)
    target_include_directories(imgui_gl INTERFACE ${ament_imgui_INCLUDE_DIRS})
    target_link_libraries(imgui_gl INTERFACE ${ament_imgui_LIBRARIES})

else()
    include(FetchContent)

    FetchContent_Declare(
        ament_imgui
        GIT_REPOSITORY git@github.com:PepeOjeda/ament_imgui.git
        GIT_TAG FullyStatic
    )
    FetchContent_GetProperties(ament_imgui)
    if(NOT ament_imgui_POPULATED)
        FetchContent_Populate(ament_imgui)
        add_subdirectory(${ament_imgui_SOURCE_DIR}/imgui_gl ${ament_imgui_BINARY_DIR})
    endif()
    
endif()
