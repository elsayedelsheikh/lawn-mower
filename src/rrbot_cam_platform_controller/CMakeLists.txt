cmake_minimum_required(VERSION 3.8)
project(rrbot_cam_platform_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rrbot_cam_msgs REQUIRED)
find_package(control_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rrbot_cam_msgs
  control_msgs
)

include_directories(include)

add_executable(platform_controller 
  src/rrbot_cam_platform_controller/PIDController.cpp
  src/rrbot_cam_platform_controller/PlatformController.cpp
  src/controller_main.cpp
)

ament_target_dependencies(platform_controller ${dependencies})

install(TARGETS
  platform_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

## Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/joint_trajectory_controller_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
