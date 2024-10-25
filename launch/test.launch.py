from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set the path to the TurtleBot Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('robot-common-nav'),  # Replace with your TurtleBot package name
        'urdf',
        'turtlebot3_burger.urdf.xacro'  # Adjust for your TurtleBot model (e.g., burger, waffle, etc.)
    )

    # Declare arguments for position
    x_pos = DeclareLaunchArgument('x_pos', default_value='0.0', description="X position of the robot")
    y_pos = DeclareLaunchArgument('y_pos', default_value='0.0', description="Y position of the robot")
    z_pos = DeclareLaunchArgument('z_pos', default_value='0.0', description="Z position of the robot")

    # Process the Xacro file to generate URDF XML
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # Node to publish robot description to robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Use Gazebo’s spawn_entity.py script to spawn the TurtleBot in the world
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos'),
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        x_pos,
        y_pos,
        z_pos,
        robot_state_publisher,
        spawn_entity
    ])
