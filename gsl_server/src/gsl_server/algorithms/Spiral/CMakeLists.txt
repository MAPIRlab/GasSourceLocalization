file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 


# Create the library
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


target_link_libraries(gsl_actionserver_node
    Spiral
)
