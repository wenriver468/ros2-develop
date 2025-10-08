cmake_minimum_required(VERSION 3.8)
project(signal_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(signal_generator src/signal_generator.cpp)
ament_target_dependencies(signal_generator rclcpp std_msgs)

add_executable(signal_processor src/signal_processor.cpp)
ament_target_dependencies(signal_processor rclcpp std_msgs)

install(TARGETS
  signal_generator
  signal_processor
  DESTINATION lib/${PROJECT_NAME})

ament_package()