cmake_minimum_required(VERSION 3.16)
project(fwsbot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(dependencies
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/pca9685_system.cpp
  hardware/src/pca9685_comm.cpp
  hardware/src/I2CPeripheral.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} 
  PUBLIC
  ${dependencies}
)

# link i2c 
target_link_libraries(${PROJECT_NAME} PUBLIC i2c)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PCA9685_HARDWARE_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface fwsbot_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz description/meshes
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
