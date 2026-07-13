cmake_minimum_required(VERSION 3.8)
project(trainros_stability_evaluator)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trainros_interfaces REQUIRED)

option(ENABLE_SANITIZER "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

function(trainros_enable_sanitizer target)
  if(ENABLE_SANITIZER)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

add_executable(stability_evaluator_node src/stability_evaluator_node.cpp)
target_compile_features(stability_evaluator_node PUBLIC cxx_std_17)
target_include_directories(stability_evaluator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stability_evaluator_node diagnostic_msgs rclcpp trainros_interfaces)
trainros_enable_sanitizer(stability_evaluator_node)

install(TARGETS stability_evaluator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stability_metrics test/test_stability_metrics.cpp)
  target_compile_features(test_stability_metrics PUBLIC cxx_std_17)
  target_include_directories(test_stability_metrics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  trainros_enable_sanitizer(test_stability_metrics)
endif()

ament_package()
