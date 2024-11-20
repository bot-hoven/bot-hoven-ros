import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, Shutdown
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get the share directory of the 'description' package
    description_share_dir = get_package_share_directory('description')

    # Path to the URDF file
    urdf_file = os.path.join(description_share_dir, 'urdf', 'bothoven.urdf.xacro')

    controller_config = os.path.join(
        get_package_share_directory('hardware'),
        'config',
        'controllers.yaml'
    )

    robot_description_config = Command(['xacro ', urdf_file])

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
    )
    
    left_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_hand_controller'],
    )

    right_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_hand_controller'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        left_hand_controller,
        right_hand_controller,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ros2_control_node,
                on_exit=[Shutdown()],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_hand_controller,
                on_exit=[Shutdown()],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_hand_controller,
                on_exit=[Shutdown()],
            )
        ),
    ])
