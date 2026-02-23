from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_saver = Node(
        package='util',
        executable='waypointSaver.py',
        name='waypointSaver',
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen'
    )

    return LaunchDescription([waypoint_saver])
