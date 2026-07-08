cmake_minimum_required(VERSION 3.8)
project(trainros_camera_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_driver_node src/camera_driver_node.cpp)
target_compile_features(camera_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(camera_driver_node rclcpp sensor_msgs)

install(TARGETS camera_driver_node DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS scripts/video_file_publisher.py
  RENAME video_file_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch pipelines DESTINATION share/${PROJECT_NAME})

ament_package()
