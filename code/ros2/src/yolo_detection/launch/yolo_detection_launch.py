import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 설치 경로에서 모델 파일 위치 찾기
    pkg_share = get_package_share_directory('yolo_detection')
    model_path = os.path.join(pkg_share, 'models', '0126yolo.pt')

    # 2. Launch Argument 설정 (실행 시 경로 변경 가능하도록)
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=model_path,
        description='Path to the YOLO .pt file'
    )

    # 3. Node 설정
    yolo_node = Node(
        package='yolo_detection',
        executable='yolo_node.py',  # CMakeLists.txt에서 install한 파일명
        name='qcar_detection_node',
        output='screen',
        parameters=[
            {'model_path': LaunchConfiguration('model')}
        ]
    )

    return LaunchDescription([
        model_arg,
        yolo_node
    ])
