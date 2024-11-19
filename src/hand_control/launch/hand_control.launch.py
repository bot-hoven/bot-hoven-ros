from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for hand control nodes.
    This function creates two ROS 2 nodes for controlling the hand on both
    the left and right sides. Each node is configured with a specific namespace,
    parameters, and remappings.
    Returns:
        LaunchDescription: A ROS 2 LaunchDescription object containing the 
        configured nodes.
    """
    nodes = []
    for i in range(2):
        hand = 'left_hand' if i < 1 else 'right_hand'
        namespace=f'{hand}'
        nodes.append(
            Node(
                package='hand_control',
                executable='hand_control_node',
                name='hand_control',
                namespace=namespace,
                output='screen',
                parameters=[],
                remappings=[
                    ('/hand_position_cmd', f'/{namespace}/hand_position_cmd'),
                    ('/hand_position_state', f'/{namespace}/hand_position_state'),
                    ('/set_hand_home', f'/{namespace}/set_hand_home')
                ]
            )
        )

    return LaunchDescription(nodes)