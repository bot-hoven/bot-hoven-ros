import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for finger control nodes.
    This function creates a list of ROS 2 nodes for controlling fingers on both
    hands. It generates 10 nodes in total, 5 for each hand. Each node is 
    configured with a specific namespace, parameters, and remappings.
    Returns:
        LaunchDescription: A ROS 2 LaunchDescription object containing the 
        configured nodes.
    """
    # Generate 10 finger nodes (5 per each hand)
    nodes = []
    for i in range(10):
        hand = 'left' if i < 5 else 'right'
        finger_index = i % 5 + 1  # Finger index: 1 to 5
        namespace = f'{hand}_hand/finger_{finger_index}'
        
        nodes.append(
            Node(
                package='finger_control',
                executable='finger_control_node',
                name=f'finger_control_{hand}_{finger_index}',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'finger_index': finger_index,
                    'hand': hand,
                }],
                remappings=[
                    (f'/{namespace}/finger_position_cmd', '/finger_position_cmd'),
                    (f'/{namespace}/key_press_cmd', '/key_press_cmd'),
                ]
            )
        )

    return LaunchDescription(nodes)
