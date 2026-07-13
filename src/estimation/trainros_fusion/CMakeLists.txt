cmake_minimum_required(VERSION 3.8)
project(trainros_fusion)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trainros_interfaces REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

function(trainros_enable_sanitizer target)
  if(ENABLE_SANITIZER)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_executable(fusion_node src/fusion_node.cpp)
target_compile_features(fusion_node PUBLIC cxx_std_17)
target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fusion_node diagnostic_msgs message_filters rclcpp sensor_msgs trainros_interfaces)
trainros_enable_sanitizer(fusion_node)

install(TARGETS fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_kalman_filter test/test_motion_kalman_filter.cpp)
  target_compile_features(test_motion_kalman_filter PUBLIC cxx_std_17)
  target_include_directories(test_motion_kalman_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  trainros_enable_sanitizer(test_motion_kalman_filter)
endif()

ament_package()
