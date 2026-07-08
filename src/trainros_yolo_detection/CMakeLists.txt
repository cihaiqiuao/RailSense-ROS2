cmake_minimum_required(VERSION 3.8)
project(trainros_yolo_detection)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/yolo_detection_node.py
  RENAME yolo_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

ament_package()
