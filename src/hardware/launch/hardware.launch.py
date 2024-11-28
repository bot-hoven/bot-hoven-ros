from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare Arguments
    declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_mock_hardware",
    #         default_value="false",
    #         description="Start robot with mock hardware mirroring command to its states."
    #     )
    # )

    #Initialize Arguments
    # use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # control = LaunchConfiguration("control")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hardware"),
                    "urdf",
                    "ros2_control.xacro",
                ]
            ), 
            # " ",
            # "use_mock_hardware:=", use_mock_hardware, 
            # " ",
            # "control:=", control
        ]
    )

    robot_description = {"robot_description": robot_description_content}

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
    # hand_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[control, "-c", "/controller_manager"],
    # )

        # List all nodes that we want to start
    nodes = [
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
