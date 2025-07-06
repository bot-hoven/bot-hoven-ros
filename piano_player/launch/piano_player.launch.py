from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Parameters
    songs_dir = LaunchConfiguration('songs_dir')
    song_file = LaunchConfiguration('song_file')
    
    # Declare launch arguments
    declare_songs_dir = DeclareLaunchArgument(
        'songs_dir',
        default_value=os.path.join(get_package_share_directory('piano_player'), 'songs'),
        description='Directory containing song files'
    )
    
    declare_song_file = DeclareLaunchArgument(
        'song_file',
        default_value='mary_had_a_little_lamb.csv',
        description='Song file to play'
    )
    
    # Start the configurable song player node
    song_player_node = Node(
        package='piano_player',
        executable='configurable_song_player',
        name='song_player',
        parameters=[{
            'songs_directory': songs_dir
        }],
        arguments=[song_file]
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_songs_dir,
        declare_song_file,
        song_player_node
    ])