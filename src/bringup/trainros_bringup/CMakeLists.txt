cmake_minimum_required(VERSION 3.8)
project(trainros_bringup)

find_package(ament_cmake REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

install(
  DIRECTORY launch params rviz config systemd
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fake_serial_integration test/test_fake_serial_integration.py TIMEOUT 60)
  ament_add_pytest_test(test_launch_params test/test_launch_params.py TIMEOUT 20)
endif()

ament_package()
