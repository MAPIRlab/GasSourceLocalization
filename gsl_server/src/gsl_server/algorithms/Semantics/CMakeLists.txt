# Comment out any of these lines to disable compilation of the corresponding semantics type or GSL algorithm
# "Common" is required by the rest of the semantics stuff, but everything else is entirely modular
# You can technically even compile the GSL algorithms without any semantic types enabled, but you obviously won't be able to run them

# Semantic types
include(${CMAKE_CURRENT_LIST_DIR}/Semantics/Common/CMakeLists.txt) # Required
include(${CMAKE_CURRENT_LIST_DIR}/Semantics/2D/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Semantics/Voxeland/CMakeLists.txt)

# GSL Algorithms with semantic integration
include(${CMAKE_CURRENT_LIST_DIR}/SemanticGrGSL/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SemanticPMFS/CMakeLists.txt)