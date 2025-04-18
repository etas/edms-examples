cmake_minimum_required(VERSION 3.5)
project(msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # <- Diese Zeile hinzufügen

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
