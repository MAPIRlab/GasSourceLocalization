cmake_minimum_required(VERSION 3.5)
project(pmfs_env)
find_package(ament_cmake REQUIRED)


install(DIRECTORY .
DESTINATION share/${PROJECT_NAME}
)
ament_package()