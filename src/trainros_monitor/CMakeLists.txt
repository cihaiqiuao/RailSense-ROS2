cmake_minimum_required(VERSION 3.8)
project(trainros_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trainros_interfaces REQUIRED)

add_executable(monitor_node src/monitor_node.cpp)
target_compile_features(monitor_node PUBLIC cxx_std_17)
ament_target_dependencies(monitor_node rclcpp diagnostic_msgs sensor_msgs trainros_interfaces)

install(TARGETS monitor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
