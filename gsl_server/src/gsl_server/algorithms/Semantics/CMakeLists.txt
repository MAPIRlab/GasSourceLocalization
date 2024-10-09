# Semantic types
include(${CMAKE_CURRENT_LIST_DIR}/Semantics/Common/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Semantics/2D/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Semantics/Voxeland/CMakeLists.txt)

# GSL Algorithms with semantic integration
include(${CMAKE_CURRENT_LIST_DIR}/SemanticGrGSL/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SemanticPMFS/CMakeLists.txt)