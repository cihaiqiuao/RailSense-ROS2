cmake_minimum_required(VERSION 3.8)
project(trainros_fusion)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trainros_interfaces REQUIRED)

add_executable(fusion_node src/fusion_node.cpp)
target_compile_features(fusion_node PUBLIC cxx_std_17)
ament_target_dependencies(fusion_node diagnostic_msgs message_filters rclcpp sensor_msgs trainros_interfaces)

install(TARGETS fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
