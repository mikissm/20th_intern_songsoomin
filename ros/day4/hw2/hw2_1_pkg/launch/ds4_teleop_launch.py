from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node"
        ),

        Node(
            package="hw2_1_pkg",
            executable="ds4_teleop",
            name="ds4_teleop"
        )
    ])
