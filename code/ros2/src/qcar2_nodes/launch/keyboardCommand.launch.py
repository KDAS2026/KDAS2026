from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qcar2_nodes',
            executable='keyboardCommand',
            name='keyboardCommand',
            output='screen',
            emulate_tty=True,
            prefix='xterm -e',
        )
    ])
