cmake_minimum_required(VERSION 3.8)
project(trainros_yolo_detection)

find_package(ament_cmake REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

install(
  PROGRAMS scripts/yolo_detection_node.py
  RENAME yolo_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_yolo_degraded test/test_yolo_degraded.py TIMEOUT 30)
endif()

ament_package()
