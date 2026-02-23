from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            executable='Depth_processe',
            name='depth_processor',
            output='screen'
        ),
        Node(
            package='obstacle_avoidance',
            executable='obsavoid',
            name='obstacle_avoider',
            output='screen'
        ),
    ])
