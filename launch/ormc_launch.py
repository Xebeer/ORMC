from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ormc',
            namespace="ORMC",
            executable='direct_kinematics',
        ),
        Node(
            package='ormc',
            namespace="ORMC",
            executable='trajectory_generator',
        ),
        Node(
            package='ormc',
            namespace="ORMC",
            executable='program_parser',
        )
    ])
