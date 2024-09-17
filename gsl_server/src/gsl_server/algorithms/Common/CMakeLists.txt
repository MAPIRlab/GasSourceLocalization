
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

find_package(OpenCV REQUIRED)

# Create the library
add_library(GSL_common ${SOURCES})
target_include_directories(GSL_common PUBLIC ${OpenCV_INCLUDE_DIRS})

# Link ament dependencies
ament_target_dependencies(GSL_common
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(GSL_common  
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
    fmt
    ${CONDITIONAL_TRACY}
)