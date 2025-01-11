cmake_minimum_required(VERSION 3.8)
project(hardware)

# Add compile options
# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set Core dependencies
# Set Core dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager
)

# Find dependencies
# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find libgpiod
find_package(PkgConfig REQUIRED) # Required for system libraries
pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiodcxx) # Finds libgpiod
include_directories(${LIBGPIOD_INCLUDE_DIRS}) # Adds libgpiod headers to include path

# Find libi2c
pkg_check_modules(LIBI2C REQUIRED IMPORTED_TARGET i2c)
include_directories(${LIBI2C_INCLUDE_DIRS})

# Add a library to the project using the specified source files
add_library(
  ${PROJECT_NAME} SHARED
  src/bothoven_hardware.cpp
  src/cl42t.cpp
  src/i2c/I2CPeripheral.cpp
  src/pca9685/pca9685_comm.cpp
  src/pca9685/pca9685_system.cpp
  src/mcp23017/mcp23017_comm.cpp
  src/mcp23017/mcp23017_system.cpp
)

# Include directories
# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBGPIOD_INCLUDE_DIRS}
  ${LIBI2C_INCLUDE_DIRS}
)

# Link dependencies
# Link dependencies
ament_target_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${LIBGPIOD_LIBRARIES}
  ${LIBI2C_LIBRARIES}
)

# Export plugin
# Export plugin
pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

# Linting
# Linting
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

# Add an executable to the project using the specified source files
# Add an executable to the project using the specified source files
add_executable(example_jtc examples/example_jtc.cpp)
ament_target_dependencies(example_jtc
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

# Specify rules to run at install time
# Specify rules to run at install time
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    example_jtc
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  FILES hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Export dependencies and targets
# Export dependencies and targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Mark the package for export

# Mark the package for export
ament_package()