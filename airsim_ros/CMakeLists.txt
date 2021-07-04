cmake_minimum_required(VERSION 3.5)
project(airsim_ros)

set(CMAKE_CXX_STANDARD 17)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/libs/AirSim)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(airsim_ros_interfaces REQUIRED)

include_directories(
  include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

set(dependencies
  rclcpp
  image_transport
  geometry_msgs
  sensor_msgs
  airsim_ros_interfaces
)

############################
## Libraries and executables
############################
set(settings_lib settings_parser)
add_library(${settings_lib} SHARED
  src/settings_parser.cpp
)
target_link_libraries(${settings_lib} AirLib)

set(wrapper_lib ros_wrapper_core)
add_library(${wrapper_lib} SHARED
  src/ros_wrapper.cpp
)
target_link_libraries(${wrapper_lib} ${settings_lib} yaml-cpp AirLib)
ament_target_dependencies(${wrapper_lib} ${dependencies})

set(node_exe airsim_node)
add_executable(${node_exe}
  src/airsim_node.cpp
)
target_link_libraries(${node_exe} ${wrapper_lib})
ament_target_dependencies(${node_exe}
  ${dependencies}
)

######################
## Testing
######################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

######################
## Install
######################
install(TARGETS ${wrapper_lib}
                ${settings_lib}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${node_exe}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_python_install_package(${PROJECT_NAME})

ament_package()
