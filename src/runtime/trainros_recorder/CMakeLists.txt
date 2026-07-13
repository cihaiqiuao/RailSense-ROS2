cmake_minimum_required(VERSION 3.8)
project(trainros_recorder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trainros_interfaces REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

function(trainros_enable_sanitizer target)
  if(ENABLE_SANITIZER)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_executable(recorder_node src/recorder_node.cpp)
target_compile_features(recorder_node PUBLIC cxx_std_17)
ament_target_dependencies(recorder_node rclcpp rclcpp_action trainros_interfaces)
trainros_enable_sanitizer(recorder_node)

install(TARGETS recorder_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_recorder_action test/test_recorder_action.py TIMEOUT 45)
endif()

ament_package()
