import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_robot_common_sim = get_package_share_directory('robot-common-sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')

    # Declare launch argument for the world name
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_robot_common_sim, 'worlds', '4x4m_empty.world'),
    )

    sim_time = DeclareLaunchArgument(
       'use_sim_time', default_value='true',
       description='use_sim_time or not'
    )

    # Declare launch arguments for the spawn position
    x_position = DeclareLaunchArgument(
        'x_pose', default_value='-1.8',
        description='X position of the robot'
    )
    y_position = DeclareLaunchArgument(
        'y_pose', default_value='-1.8',
        description='Y position of the robot'
    )

    # Launch file to bringup Gazebo and spawn in Turtlebot3
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    # Include your Gazebo launch file
    rviz_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_cartographer, 'launch', 'cartographer.launch.py') 
        )
    )

    # Define the launch description
    ld = LaunchDescription()

    # Add launch options and actions
    ld.add_action(world_arg)
    ld.add_action(sim_time)
    ld.add_action(x_position)
    ld.add_action(y_position)
    ld.add_action(gazebo_launch)
    ld.add_action(rviz_cartographer_launch)

    return ld