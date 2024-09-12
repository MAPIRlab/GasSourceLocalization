project(Spiral)

file(GLOB_RECURSE SOURCES "./*.cpp") 


# Create target and add dependencies
add_library(Spiral ${SOURCES})

# Link ament dependencies
ament_target_dependencies(Spiral
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(Spiral  
    OpenMP::OpenMP_CXX
    fmt
    ${CONDITIONAL_TRACY}
)
add_compile_definitions(ENABLE_SPIRAL=1)