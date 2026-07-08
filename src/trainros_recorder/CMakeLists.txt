cmake_minimum_required(VERSION 3.8)
project(trainros_recorder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trainros_interfaces REQUIRED)

add_executable(recorder_node src/recorder_node.cpp)
target_compile_features(recorder_node PUBLIC cxx_std_17)
ament_target_dependencies(recorder_node rclcpp rclcpp_action trainros_interfaces)

install(TARGETS recorder_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
