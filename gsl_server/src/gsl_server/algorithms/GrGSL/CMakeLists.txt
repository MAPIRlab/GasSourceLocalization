
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

find_package(gmrf_msgs REQUIRED)

# Create the library
add_library(GrGSL ${SOURCES})

# Link ament dependencies
ament_target_dependencies(GrGSL
    ${COMMON_AMENT_DEPENDENCIES}
    gmrf_msgs
    ${CONDITIONAL_GADEN_MSGS}
)

# Link other dependencies
target_link_libraries(GrGSL  
    GSL_common
)
add_compile_definitions(ENABLE_GrGSL=1)


target_link_libraries(gsl_actionserver_node
    GrGSL
)
