
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 


# Create the library
add_library(ParticleFilter ${SOURCES})

# Link ament dependencies
ament_target_dependencies(ParticleFilter
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(ParticleFilter  
    GSL_common
)
add_compile_definitions(ENABLE_PARTICLE_FILTER=1)

target_link_libraries(gsl_actionserver_node
    ParticleFilter
)
