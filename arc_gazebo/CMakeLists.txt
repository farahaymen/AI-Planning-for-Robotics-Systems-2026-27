cmake_minimum_required(VERSION 3.8)
project(arc_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# cmd_vel_relay is a Python node in an ament_cmake package. install(PROGRAMS)
# puts it on the lib path so `ros2 run arc_gazebo cmd_vel_relay.py` resolves,
# without turning this package into ament_python or adding a package solely for
# one node.
install(PROGRAMS
  scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
