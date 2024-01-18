cmake_minimum_required(VERSION 3.8)
project(ur5_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_subdirectory(ur5_description)
add_subdirectory(ur5_moveit_config)
add_subdirectory(ur5_gazebo)

ament_package()
