cmake_minimum_required(VERSION 3.8)
project(arc_lab2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wheel_watchdog src/wheel_watchdog.cpp)
ament_target_dependencies(wheel_watchdog rclcpp sensor_msgs)
install(TARGETS wheel_watchdog DESTINATION lib/${PROJECT_NAME})



ament_package()
