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
    finger_names = ['thumb', 'index_finger', 'middle_finger', 'ring_finger', 'pinky']
    nodes = []
    for i in range(10):
        hand = 'left_hand' if i < 5 else 'right_hand'
        finger_name = finger_names[i % 5]
        namespace = f'{hand}/{finger_name}'
        
        nodes.append(
            Node(
                package='finger_control',
                executable='finger_control_node',
                name='finger_control',
                namespace=namespace,
                output='screen',
                parameters=[{ 
                    'finger_press_duration': 1.0
                }],
                remappings=[ # Remap topics to the namespace
                    ('/finger_position_cmd', f'/{namespace}/finger_position_cmd'),
                    ('/key_press_cmd', f'/{namespace}/key_press_cmd'),
                ]
            )
        )

    return LaunchDescription(nodes)
