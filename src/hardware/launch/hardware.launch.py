import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Declare Arguments
    declare_use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Start robot with mock hardware mirroring command to its states."
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ROS 2 control if true'
    )

    # Get URDF via xacro
    pkg_path_description = os.path.join(get_package_share_directory('description'))
    xacro_file = os.path.join(pkg_path_description,'urdf','bothoven.urdf.xacro')

    robot_description_content = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hardware"), 
            "config", 
            "controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[("~/robot_description", "/robot_description"),]
    )

    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn additional nodes for control and visualization
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Add spawner node for each hand controller
    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "-c", "/controller_manager"],
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "-c", "/controller_manager"],
    )

    # List all arguments that we want to declare
    declared_arguments = [
        # declare_use_mock_hardware,
        declare_use_sim_time,
        declare_use_ros2_control
    ]

    # List all nodes that we want to start
    nodes = [
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        right_hand_controller_spawner,
        left_hand_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
