cmake_minimum_required(VERSION 3.8)
project(trainros_logger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(trainros_interfaces REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

function(trainros_enable_sanitizer target)
  if(ENABLE_SANITIZER)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_executable(logger_node src/logger_node.cpp)
target_compile_features(logger_node PUBLIC cxx_std_17)
ament_target_dependencies(logger_node rclcpp diagnostic_msgs trainros_interfaces)
trainros_enable_sanitizer(logger_node)

install(TARGETS logger_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_logger_events test/test_logger_events.py TIMEOUT 30)
endif()

ament_package()
