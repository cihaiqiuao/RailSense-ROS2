cmake_minimum_required(VERSION 3.8)
project(trainros_camera_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

function(trainros_enable_sanitizer target)
  if(ENABLE_SANITIZER)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_executable(camera_driver_node src/camera_driver_node.cpp)
target_compile_features(camera_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(camera_driver_node rclcpp sensor_msgs)
trainros_enable_sanitizer(camera_driver_node)

install(TARGETS camera_driver_node DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS scripts/video_file_publisher.py
  RENAME video_file_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch pipelines DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_video_file_degraded test/test_video_file_degraded.py TIMEOUT 30)
endif()

ament_package()
