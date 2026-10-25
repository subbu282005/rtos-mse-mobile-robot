
cmake_minimum_required(VERSION 3.8)
project(rtos_mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright checks until copyright headers are added.
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint until copyright headers are added.
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Install the Python obstacle-avoidance controller
install(
  PROGRAMS
    scripts/obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

