cmake_minimum_required(VERSION 3.8)
project(wheel_nav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Waypoint.srv"
  "msg/StepData.msg"
  "msg/GoalPosition.msg"
  "srv/GoalPosition.srv"
  DEPENDENCIES geometry_msgs
)

# Install the config directory
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
