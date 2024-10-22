import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the TurtleBot3 model from the environment variable
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Path to the SDF file for the TurtleBot3 model
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_' + TURTLEBOT3_MODEL,
        'model.sdf'
    )

    # Launch configuration variables for the robot's spawn position
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare launch arguments for the spawn position
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot'
    )

    # Include your Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot-common-sim'), 'launch', 'gazebo_sim.launch.py') 
        )
    )

    # Node to spawn the TurtleBot3 in Gazebo
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Define the launch description
    ld = LaunchDescription()

    # Add launch options and actions
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(gazebo_launch)  # Include your Gazebo launch file
    ld.add_action(spawn_turtlebot_cmd)  # Then spawn the TurtleBot3

    return ld
