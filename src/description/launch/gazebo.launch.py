import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_project_description = get_package_share_directory('description')

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

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_project_description,'launch','rsp.launch.py')]
        ), 
        launch_arguments={
            'use_sim_time': 'true', 
            'use_ros2_control': 'false'
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': os.path.join(
                pkg_project_description, 
                'worlds', 
                'bothoven_world.sdf'
            )
        }.items()
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
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/joint_states', 'joint_states'),
            ('/cmd_vel', 'cmd_vel'),
            ('/odom', 'odom')
        ],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_bothoven',
        parameters=[{'name': 'bothoven',
                    'topic': '/robot_description'}],
        output='screen'
    )

    return LaunchDescription([
        generate_model_sdf,
        rsp,
        gazebo,
        bridge,
        rviz
    ])
