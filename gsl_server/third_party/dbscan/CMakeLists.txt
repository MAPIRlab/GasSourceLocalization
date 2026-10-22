cmake_minimum_required(VERSION 3.10)
project(dbscan)
set(CMAKE_CXX_STANDARD 20)

add_library(dbscan SHARED dbscan.cpp)
target_include_directories(dbscan PUBLIC vendor include)

# add_executable(dbscan_example example.cpp)
# target_link_libraries(dbscan_example dbscan)