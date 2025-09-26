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
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Start robot in Gazebo."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_viz",
            default_value="false",
            description="Starts rviz."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_args",
            default_value="",
        )
    )

    use_sim = LaunchConfiguration("use_sim")
    enable_viz = LaunchConfiguration("enable_viz")
    gz_args = LaunchConfiguration("gz_args")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [gz_args, ' -r -v4 empty.sdf'])],
        condition=IfCondition(use_sim),
    )

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
            "0 0 0.2 0 0 0"
        ],
        condition=IfCondition(use_sim),
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_sim),
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bothoven_description"), "urdf", "bothoven.urdf.xacro"]
            ),
            " ",
            "use_sim:=",
            use_sim
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bothoven_description"), "rviz", "bothoven.rviz"]
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("bothoven_bringup"), 'config', "controllers.yaml"]
    )
    
    rsp_params = {"robot_description": robot_description_content, "use_sim_time": use_sim}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ParameterFile(robot_controllers, allow_substs=True)],
        remappings=[("~/robot_description", "robot_description"),],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        condition=UnlessCondition(use_sim),

    )

    robot_state_publisher_node = Node(
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
        condition=IfCondition(enable_viz),
    )

    # delayed_joint_state_broadcaster_spawner_sim = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=gazebo_spawn_entity,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     ),
    #     condition=IfCondition(use_sim)
    # )

    # delayed_joint_state_broadcaster_spawner_hw = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=control_node,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     ),
    #     condition=UnlessCondition(use_sim)
    # )

    # delayed_controllers_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[
    #             left_stepper_controller_spawner,
    #             left_servo_controller_spawner,
    #             left_solenoid_controller_spawner,
    #             right_stepper_controller_spawner,
    #             right_servo_controller_spawner,
    #             right_solenoid_controller_spawner
    #         ],
    #     )
    # )

    nodes = [
        gazebo,
        gazebo_bridge,
        control_node,
        robot_state_publisher_node,
        gazebo_spawn_entity,
        # delayed_joint_state_broadcaster_spawner_sim,
        # delayed_joint_state_broadcaster_spawner_hw,
        # delayed_controllers_spawner,
        joint_state_broadcaster_spawner,
        left_stepper_controller_spawner,
        left_servo_controller_spawner,
        left_solenoid_controller_spawner,
        right_stepper_controller_spawner,
        right_servo_controller_spawner,
        right_solenoid_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
