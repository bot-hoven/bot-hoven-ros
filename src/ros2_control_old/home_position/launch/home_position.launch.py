from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for home position nodes.
    This function creates two ROS 2 nodes for controlling the home position on both
    the left and right hands. Each node is configured with a specific namespace.
    Returns:
        LaunchDescription: A ROS 2 LaunchDescription object containing the 
        configured nodes.
    """
    nodes = []
    for hand in ['left_hand', 'right_hand']:
        nodes.append(
            Node(
                package='home_position',
                executable='home_position_node',
                name='home_position',
                namespace=hand,
                output='screen',
                parameters=[],
                remappings=[
                    ('/home_status', f'/{hand}/home_status'),
                    ('/calibrate_home', f'/{hand}/calibrate_home')
                ]
            )
        )

    return LaunchDescription(nodes)