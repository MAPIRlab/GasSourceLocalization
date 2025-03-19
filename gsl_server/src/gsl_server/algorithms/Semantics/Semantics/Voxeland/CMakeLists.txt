
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

find_package(voxeland_msgs)

# Create the library
add_library(ClassMapVoxeland ${SOURCES})

# Link ament dependencies
ament_target_dependencies(ClassMapVoxeland
    ${COMMON_AMENT_DEPENDENCIES}
    voxeland_msgs
)

# Link other dependencies
target_link_libraries(ClassMapVoxeland
    SemanticsCommon
    )
add_compile_definitions(ENABLE_CLASSMAP_VOXELAND=1)
list(APPEND SEMANTIC_TYPES ClassMapVoxeland)
