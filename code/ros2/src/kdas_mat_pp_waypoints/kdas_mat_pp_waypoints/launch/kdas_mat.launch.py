from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MATLAB/Simulink 기반 컨트롤러 노드 실행
        Node(
            package='kdas_mat_pp_waypoints',
            executable='KDAS_mat_pp_waypoints',
            name='kdas_controller',
            output='screen',
            parameters=[
                # 필요 시 파라미터 추가 가능
            ],
            remappings=[
                # 토픽 이름을 변경해야 할 경우 여기에 작성
                # ('/location', '/current_pose'),
            ]
        ),
        
        # (선택 사항) 시각화 노드나 센서 노드가 있다면 아래에 추가 가능
    ])