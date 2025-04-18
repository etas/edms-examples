cmake_minimum_required(VERSION 3.8)
project(hello_world)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

# Install targets
install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
