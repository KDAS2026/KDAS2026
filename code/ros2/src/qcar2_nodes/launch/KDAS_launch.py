import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # Include the Cartographer localization launch file

    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('qcar2_nodes'),
                'launch',
                'localization_launch.py'
            )
        )
    )

    util_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('util'),
                'launch',
                'util_launch.py'
            )
        )
    )

    lane_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lane_detection'),
                'launch',
                'lane_detection_launch.py'
            )
        )
    )

    path_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('path_planning'),
                'launch',
                'path_planning_launch.py'
            )
        )
    )

    obstacle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('obstacle_avoidance'),
                'launch',
                'obstacle_avoidance_launch.py'
            )
        )
    )

    yoloNode = Node(
        package='yolo_detection',  
        executable='yolo_node.py',
        name='yolo_node',
        output='screen'
    )

    controller = Node(
        package='kdas_mat',
        executable='KDAS_mat',
        name='KDAS_mat',
        output='screen'
    )



    return LaunchDescription([
        #carto_launch,
        util_launch,
        path_launch,
        obstacle_launch,
        yoloNode,
        controller,
        #lane_launch,
    ])
