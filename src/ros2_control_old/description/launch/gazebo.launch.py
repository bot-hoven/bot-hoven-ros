import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_project_description = get_package_share_directory('description')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_ros2_control = LaunchConfiguration('use_ros2_control', default=False)

    # Get URDF via xacro
    xacro_file = os.path.join(pkg_project_description,'urdf','bothoven.urdf.xacro')

    robot_description_content = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
   
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    # Generate URDF from xacro
    generate_urdf = ExecuteProcess(
        cmd=['ros2', 'run', 'xacro', 'xacro', os.path.join(pkg_project_description, 'urdf', 'bothoven.urdf.xacro'), '-o', '/tmp/bothoven.urdf'],
        output='screen'
    )

    # Generate SDF from URDF
    generate_model_sdf = ExecuteProcess(
        cmd=['gz', 'sdf', '-p', '/tmp/bothoven.urdf', '>', os.path.join(pkg_project_description, 'models', 'bothoven', 'model.sdf')],
        shell=True,
        output='screen',
        on_exit=[generate_urdf]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('hardware'),
            'config',
            'controllers.yaml',
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ' '.join([
                '--physics-engine gz-physics-bullet-featherstone-plugin -r -v 4',
                '--gui-config ' + os.path.join(pkg_project_description, 'resource', 'gazebo_config.config'),
                os.path.join(pkg_project_description, 'worlds', 'bothoven_world.sdf')
            ])
        }.items()
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    right_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_hand_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    left_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_hand_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_description, 'resource', 'rviz2_config.rviz')],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name',
                   'bothoven', '-allow_renaming', 'true'],
        output='screen'
    )

    return LaunchDescription([
        generate_model_sdf,
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[right_hand_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_hand_controller_spawner],
            )
        ),
        bridge,
        robot_state_publisher,
        spawn_entity,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value=use_ros2_control,
            description='If true, use ros2_control'
        )
    ])
