# nav2.launch.py (최종 수정본)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 파라미터 파일의 경로를 설정합니다.
    params_file = os.path.join(
        get_package_share_directory('rl_sar'),
        'config',
        'nav2_params.yaml'
    )

    # Nav2의 lifecycle manager를 통해 우리가 필요한 노드들만 명시적으로 실행합니다.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                # ★★★★★ 지도 없는 주행에 필요한 노드 목록만 지정
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]}
            ]
        ),

        # 각 서버 노드들을 실행합니다. 이 노드들은 lifecycle_manager가 관리합니다.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])