from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Python nodes

        Node(
            package='path_planning',
            executable='helper.py',
            name='helper',
            output='screen'
        ),
        Node(
            package='path_planning',
            executable='led_controller.py',
            name='led_controller',
            output='screen'
        ),
    ])
