cmake_minimum_required(VERSION 3.5)
project(intraprocess_chain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(msgs REQUIRED)

add_executable(intraprocess_chain src/multiprocess_executor.cpp)

ament_target_dependencies(intraprocess_chain
  rclcpp
  std_msgs
  builtin_interfaces
  msgs
)

install(TARGETS
  intraprocess_chain
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
