import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_args",
            default_value="",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [gz_args, ' -r -v4 empty.sdf'])]
    )

    # gazebo_headless = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     ),
    #     launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
    #     condition=UnlessCondition(gui),
    # )

    gazebo_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "bothoven",
            "-allow_renaming",
            "true",
            "-pose",
            "0 0 0.1 0 0 0"
        ],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bothoven_description"), "urdf", "bothoven.urdf.xacro"]
            ),
            " ",
            "use_sim:=true",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bothoven_description"), "rviz", "bothoven.rviz"]
    )

    rsp_params = {"robot_description": robot_description_content, "use_sim_time": True}
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[rsp_params],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    left_stepper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_stepper_controller", "-c", "/controller_manager"],
    )

    left_servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_servo_controller", "-c", "/controller_manager"],
    )

    left_solenoid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_solenoid_controller", "-c", "/controller_manager"],
    )

    right_stepper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_stepper_controller", "-c", "/controller_manager"],
    )

    right_servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_servo_controller", "-c", "/controller_manager"],
    )

    right_solenoid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_solenoid_controller", "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_left_stepper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_stepper_controller_spawner],
        )
    )

    delayed_left_servo_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_servo_controller_spawner],
        )
    )

    delayed_left_solenoid_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_solenoid_controller_spawner],
        )
    )

    delayed_right_stepper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_stepper_controller_spawner],
        )
    )

    delayed_right_servo_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_servo_controller_spawner],
        )
    )

    delayed_right_solenoid_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_solenoid_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        # gazebo_headless,
        gazebo_bridge,
        node_robot_state_publisher,
        gazebo_spawn_entity,
        delayed_joint_state_broadcaster_spawner,
        delayed_left_stepper_controller_spawner,
        delayed_left_servo_controller_spawner,
        delayed_left_solenoid_controller_spawner,
        delayed_right_stepper_controller_spawner,
        delayed_right_servo_controller_spawner,
        delayed_right_solenoid_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
