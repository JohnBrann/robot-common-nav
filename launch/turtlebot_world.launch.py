from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the TurtleBot3 model from the environment variable
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    urdf_path = os.path.join(
        get_package_share_directory('robot-common-nav'),
        'urdf',
        'turtlebot3_burger.urdf.xacro'
    )
    
    # Create the robot state publisher node first
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]),  # Removed extra space
                value_type=str
            )
        }]
    )
    
    # Then create the spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],  # Removed leading slash
        output='screen'
    )
    
    # Return launch description with correct order
    return LaunchDescription([
        robot_state_publisher_node,  # Start this first
        spawn_entity  # Then spawn the entity
    ])