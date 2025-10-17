cmake_minimum_required(VERSION 3.8)
project(filter_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(data_generator src/data_generator.cpp)
ament_target_dependencies(data_generator rclcpp std_msgs)

add_executable(median_filter src/median_filter.cpp)
ament_target_dependencies(median_filter rclcpp std_msgs)

add_executable(low_pass_filter src/low_pass_filter.cpp)
ament_target_dependencies(low_pass_filter rclcpp std_msgs)

install(TARGETS
  data_generator
  median_filter
  low_pass_filter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()