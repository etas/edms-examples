cmake_minimum_required(VERSION 3.5)
project(intraprocess_chain)

# Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Executable erstellen
add_executable(intraprocess_chain src/multiprocess_executor.cpp)

# Mit Abhängigkeiten verknüpfen
ament_target_dependencies(intraprocess_chain
  rclcpp
  std_msgs
  builtin_interfaces
)

# Installationsanweisungen
install(TARGETS
  intraprocess_chain
  DESTINATION lib/${PROJECT_NAME}
)

# Paketexport
ament_package()
