
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

# Create the library
add_library(SemanticGrGSL ${SOURCES})

# Link ament dependencies
ament_target_dependencies(SemanticGrGSL
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(SemanticGrGSL
    SemanticsCommon
    GrGSL
    ${SEMANTIC_TYPES}
    )
add_compile_definitions(ENABLE_SEMANTIC_GrGSL=1)


target_link_libraries(gsl_actionserver_node
    SemanticGrGSL
)
