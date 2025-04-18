cmake_minimum_required(VERSION 3.5)
project(interprocess_chain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(callback1_node src/callback1_node.cpp)
ament_target_dependencies(callback1_node rclcpp std_msgs)

add_executable(callback2_node src/callback2_node.cpp)
ament_target_dependencies(callback2_node rclcpp std_msgs)

add_executable(callback3_node src/callback3_node.cpp)
ament_target_dependencies(callback3_node rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

install(
    TARGETS
    publisher
    callback1_node
    callback2_node
    callback3_node
    subscriber
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
      
ament_package()
