# cmake_minimum_required(VERSION 3.8)
# project(hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(THIS_PACKAGE_INCLUDE_DEPENDS
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   controller_manager
# )

# # find dependencies
# find_package(ament_cmake REQUIRED)
# foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# add_library(
#   ${PROJECT_NAME} SHARED
#   src/bothoven_hardware.cpp
# )

# target_include_directories(${PROJECT_NAME} PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(
#   ${PROJECT_NAME} PUBLIC
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# target_link_libraries(${PROJECT_NAME}
  
# )

# pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

# add_library(
#   pca9685_hardware_interface
#   SHARED
#   src/pca9685/pca9685_system.cpp
#   src/pca9685/pca9685_comm.cpp
#   src/i2c/I2CPeripheral.cpp
# )

# target_compile_features(pca9685_hardware_interface PUBLIC cxx_std_17)
# target_include_directories(pca9685_hardware_interface PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include/pca9685_hardware_interface>
# )
# target_link_libraries(pca9685_hardware_interface PUBLIC i2c)
# ament_target_dependencies(
#   pca9685_hardware_interface PUBLIC
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# # Causes the visibility macros to use dllexport rather than dllimport,
# # which is appropriate when building the dll but not consuming it.
# target_compile_definitions(${PROJECT_NAME} PRIVATE "PCA9685_HARDWARE_INTERFACE_BUILDING_DLL")

# # Export hardware plugins
# pluginlib_export_plugin_description_file(hardware_interface pca9685_hardware_interface.xml)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(
#   DIRECTORY launch config urdf
#   DESTINATION share/${PROJECT_NAME}
# )

# # install(
# #   TARGETS ${PROJECT_NAME}
# #   DESTINATION lib
# # )

# install(TARGETS 
#   ${PROJECT_NAME}
#   pca9685_hardware_interface
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(
#   FILES 
#   hardware.xml
#   pca9685_hardware_interface.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_libraries(
#   ${PROJECT_NAME}
#   pca9685_hardware_interface
# )

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
# ament_package()


cmake_minimum_required(VERSION 3.8)
project(hardware)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Bothoven hardware library
add_library(bothoven_hardware SHARED
  src/bothoven_hardware.cpp
)
target_include_directories(bothoven_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bothoven_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# PCA9685 hardware interface library
add_library(pca9685_hardware_interface SHARED
  src/pca9685/pca9685_system.cpp
  src/pca9685/pca9685_comm.cpp
  src/i2c/I2CPeripheral.cpp
)
target_include_directories(pca9685_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(pca9685_hardware_interface PUBLIC cxx_std_17)
ament_target_dependencies(pca9685_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export plugin XMLs for each library
pluginlib_export_plugin_description_file(
  hardware_interface hardware.xml
)
pluginlib_export_plugin_description_file(
  hardware_interface pca9685_hardware_interface.xml
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install other resources
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  bothoven_hardware
  pca9685_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML files
install(FILES
  hardware.xml
  pca9685_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export libraries and targets
ament_export_libraries(
  bothoven_hardware
  pca9685_hardware_interface
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
