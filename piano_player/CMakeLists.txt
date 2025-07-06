cmake_minimum_required(VERSION 3.8)
project(piano_player)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Add executables
add_executable(configurable_song_player src/configurable_song_player.cpp)
ament_target_dependencies(configurable_song_player
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
  control_msgs
)

# Install executables
install(TARGETS
  configurable_song_player
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install songs files
install(DIRECTORY
  songs
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
  control_msgs
)

ament_package()