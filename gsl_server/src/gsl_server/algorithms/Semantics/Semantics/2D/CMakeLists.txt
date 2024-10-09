
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

# Create the library
add_library(ClassMap2D ${SOURCES})

# Link ament dependencies
ament_target_dependencies(ClassMap2D
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(ClassMap2D
    SemanticsCommon
    )
add_compile_definitions(ENABLE_CLASSMAP2D=1)
list(APPEND SEMANTIC_TYPES ClassMap2D)
