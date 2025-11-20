# Copyright (c) 2024-2025 Ziqi Fan (Modified by Sanghyun Kim & Gemini)
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration("rname")

    wname = "stairs"
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    # ------------------------------
    # 1단계: Robot State Publisher
    # ------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ------------------------------
    # 2단계: Gazebo 시작
    # ------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "true",
            "pause": "false",
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    # ------------------------------
    # 3단계: Robot Spawn
    # ------------------------------
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robot_model",
            "-z", "1.0",
            "-timeout", "60",
        ],
        output="screen",
    )

    # dron 제거 → delay spawn에는 로봇만 포함
    delayed_spawn = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="Starting robot spawn after 15 second delay..."),
            spawn_entity,
        ]
    )

    # ------------------------------
    # 4단계: Controller Manager
    # ------------------------------
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "30",
        ],
        output="screen",
    )

    delayed_controller = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg="Starting joint state broadcaster with extended timeout..."),
            joint_state_broadcaster_node
        ]
    )

    # 기타 노드
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 0.0}],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{"robot_name": robot_name, "gazebo_model_name": gazebo_model_name}],
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare([rname, '_description']),
                'launch',
                'display.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    odom_to_tf_node = Node(
        package="odom_tf_broadcaster",
        executable="odom_to_tf_node",
        name="odom_to_tf_node",
        output="screen",
    )

    static_map_to_odom_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )# map -> odom -> base -> 나머지 센서들

    delayed_other_nodes = TimerAction(
        period=35.0,
        actions=[
            LogInfo(msg="Starting additional nodes..."),
            joy_node,
            param_node,
            display_launch,
            odom_to_tf_node,
            static_map_to_odom_publisher,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text="go2"),
        ),
        LogInfo(msg="Step 1: Starting robot state publisher..."),
        robot_state_publisher_node,
        LogInfo(msg="Step 2: Starting Gazebo with large world file..."),
        gazebo,
        delayed_spawn,
        delayed_controller,
        delayed_other_nodes,
    ])
