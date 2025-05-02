cmake_minimum_required(VERSION 3.8)
project(sim_carto_l1_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)


install(
 DIRECTORY bag description param launch mesh map
 DESTINATION share/${PROJECT_NAME}
)


ament_package()
