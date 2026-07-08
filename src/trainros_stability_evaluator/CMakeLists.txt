cmake_minimum_required(VERSION 3.8)
project(trainros_stability_evaluator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trainros_interfaces REQUIRED)

add_executable(stability_evaluator_node src/stability_evaluator_node.cpp)
target_compile_features(stability_evaluator_node PUBLIC cxx_std_17)
ament_target_dependencies(stability_evaluator_node rclcpp trainros_interfaces)

install(TARGETS stability_evaluator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
