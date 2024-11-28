import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory of the 'description' package
    description_share_dir = get_package_share_directory('description')

    # Path to the URDF file
    urdf_file = os.path.join(description_share_dir, 'urdf', 'bothoven.urdf.xacro')

    robot_description_config = Command(['xacro ', urdf_file])

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
        ),
        # Spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'description', '-topic', '/description'],
            output='screen'
        ),
    ])
