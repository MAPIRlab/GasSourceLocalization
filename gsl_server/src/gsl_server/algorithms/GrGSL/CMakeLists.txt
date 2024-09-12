
file(GLOB_RECURSE SOURCES "./*.cpp") 

find_package(gmrf_wind_mapping REQUIRED)
if(USE_GADEN)
    add_definitions(-DUSE_GADEN=1)
    find_package(gaden_player REQUIRED)
    set(CONDITIONAL_GADEN_PLAYER "gaden_player")  
else()
    set(CONDITIONAL_GADEN_PLAYER "")  
endif(USE_GADEN)


# Create target and add dependencies
add_library(GrGSL ${SOURCES})

# Link ament dependencies
ament_target_dependencies(GrGSL
    ${COMMON_AMENT_DEPENDENCIES}
    gmrf_wind_mapping
    ${CONDITIONAL_GADEN_PLAYER}
)

# Link other dependencies
target_link_libraries(GrGSL  
    OpenMP::OpenMP_CXX
    fmt
    ${CONDITIONAL_TRACY}
    GSL_common
)
add_compile_definitions(ENABLE_GrGSL=1)