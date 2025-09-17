from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hw2_2_pkg',
            executable='odom_broadcaster',
            name='odom_broadcaster',
            output='screen'
        )
    ])

