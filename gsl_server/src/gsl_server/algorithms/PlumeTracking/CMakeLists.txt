
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 


# Create the library
add_library(PlumeTracking ${SOURCES})

# Link ament dependencies
ament_target_dependencies(PlumeTracking
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(PlumeTracking  
    OpenMP::OpenMP_CXX
    fmt
    ${CONDITIONAL_TRACY}
    GSL_common
)

add_compile_definitions(ENABLE_PLUME_TRACKING=1)

target_link_libraries(gsl_actionserver_node
    PlumeTracking
)
