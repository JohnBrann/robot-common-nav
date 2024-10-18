# File: ~/ros2_turtlebot_ws/src/turtlebot_controller/launch/turtlebot_world_launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the gazebo_worlds package and world launch file
    gazebo_worlds_pkg = get_package_share_directory('gazebo_worlds')
    world_launch_file = PathJoinSubstitution([gazebo_worlds_pkg, 'launch', 'my_world.launch.py'])  # Replace with actual world launch file

    # Path to the TurtleBot model SDF file
    turtlebot_model = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf'  # Adjust the model type if needed
    ])

    # Include the world launch file from the gazebo_worlds package
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_file)
    )

    # Command to spawn the TurtleBot in the world
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', turtlebot_model,
            '-x', '0', '-y', '0', '-z', '0.1'  # Adjust position if needed
        ],
        output='screen'
    )

    # Combine everything into the launch description
    return LaunchDescription([
        world_launch,
        spawn_turtlebot
    ])
