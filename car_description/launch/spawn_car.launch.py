from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')

    return LaunchDescription([

        # 1. Start Gazebo WITH factory plugin
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # 2. Publish robot_description (xacro → URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file])
            }],
            output='screen'
        ),

        # 3. Spawn entity from robot_description
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'car',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])

