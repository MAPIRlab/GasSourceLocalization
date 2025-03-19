
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp") 

# Create the library
add_library(SemanticPMFS ${SOURCES})

# Link ament dependencies
ament_target_dependencies(SemanticPMFS
    ${COMMON_AMENT_DEPENDENCIES}
)

# Link other dependencies
target_link_libraries(SemanticPMFS
    SemanticsCommon
    PMFS
    ${SEMANTIC_TYPES}
    )
add_compile_definitions(ENABLE_SEMANTIC_PMFS=1)


target_link_libraries(gsl_actionserver_node
    SemanticPMFS
)
