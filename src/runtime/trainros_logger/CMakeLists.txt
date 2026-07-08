cmake_minimum_required(VERSION 3.8)
project(trainros_logger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(trainros_interfaces REQUIRED)

add_executable(logger_node src/logger_node.cpp)
target_compile_features(logger_node PUBLIC cxx_std_17)
ament_target_dependencies(logger_node rclcpp diagnostic_msgs trainros_interfaces)

install(TARGETS logger_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
