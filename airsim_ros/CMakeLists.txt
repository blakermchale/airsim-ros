cmake_minimum_required(VERSION 3.5)
project(airsim_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
