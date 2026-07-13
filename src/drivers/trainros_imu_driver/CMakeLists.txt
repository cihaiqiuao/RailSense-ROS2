cmake_minimum_required(VERSION 3.8)
project(trainros_imu_driver)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

function(trainros_enable_sanitizer target)
  if(ENABLE_SANITIZER)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_executable(imu_driver_node src/imu_driver_node.cpp)
target_compile_features(imu_driver_node PUBLIC cxx_std_17)
target_include_directories(imu_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_driver_node diagnostic_msgs rclcpp sensor_msgs)
trainros_enable_sanitizer(imu_driver_node)

install(TARGETS imu_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_parser test/test_imu_parser.cpp)
  target_compile_features(test_imu_parser PUBLIC cxx_std_17)
  target_include_directories(test_imu_parser PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  trainros_enable_sanitizer(test_imu_parser)
endif()

ament_package()
