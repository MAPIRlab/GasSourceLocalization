
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

find_package(magic_enum REQUIRED)
find_package(vision_msgs REQUIRED)

# Create the library
add_library(SemanticsCommon ${SOURCES})

# Link ament dependencies
ament_target_dependencies(SemanticsCommon
    ${COMMON_AMENT_DEPENDENCIES}
    vision_msgs
)

# Link other dependencies
target_link_libraries(SemanticsCommon
    OpenMP::OpenMP_CXX
    fmt
    ${CONDITIONAL_TRACY}
    magic_enum::magic_enum
    GSL_common
    )
