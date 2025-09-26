cmake_minimum_required(VERSION 3.8)
project(bothoven_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiodcxx)
include_directories(${LIBGPIOD_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME} SHARED
  src/cl42t/cl42t_comm.cpp
  src/cl42t/cl42t_system.cpp
  src/i2c/I2CPeripheral.cpp
  src/pca9685/pca9685_comm.cpp
  src/pca9685/pca9685_system.cpp
  src/mcp23017/mcp23017_comm.cpp
  src/mcp23017/mcp23017_system.cpp
  src/ads7138/ads7138_comm.cpp 
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

find_library(
  I2C_LIB
  NAMES i2c
  HINTS
  /usr/lib
  /usr/lib/x86_64-linux-gnu
)

target_link_libraries(
  ${PROJECT_NAME}
  ${LIBGPIOD_LIBRARIES}
  ${I2C_LIB}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
