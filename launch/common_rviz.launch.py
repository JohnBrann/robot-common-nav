import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory for RViz
    rviz_config_dir = os.path.join(get_package_share_directory('robot-common-nav'), 'rviz2', 'rviz2_turtlebot3_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Start RViz with the specified configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map']
        ),
    ])
