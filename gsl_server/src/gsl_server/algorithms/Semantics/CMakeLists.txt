
file(GLOB_RECURSE SOURCES "./*.cpp") 

#Semantics
find_package(magic_enum REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yaml_cpp_vendor)
find_package(voxeland)

# Create target and add dependencies
add_library(Semantics ${SOURCES})

# Link ament dependencies
ament_target_dependencies(Semantics
    ${COMMON_AMENT_DEPENDENCIES}
    vision_msgs
    yaml_cpp_vendor
    voxeland
)

# Link other dependencies
target_link_libraries(Semantics  
    OpenMP::OpenMP_CXX
    fmt
    ${CONDITIONAL_TRACY}
    magic_enum::magic_enum
    GSL_common
    PMFS
    )
add_compile_definitions(ENABLE_SEMANTICS=1)