cmake_minimum_required(VERSION 3.8)
project(trainros_gps_driver)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gps_driver_node src/gps_driver_node.cpp)
target_compile_features(gps_driver_node PUBLIC cxx_std_17)
target_include_directories(gps_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gps_driver_node diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS gps_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gps_parser test/test_gps_parser.cpp)
  target_compile_features(test_gps_parser PUBLIC cxx_std_17)
  target_include_directories(test_gps_parser PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
