#!/usr/bin/env python3
"""
完整的 MoveIt + Gazebo 力矩控制启动文件
架构: MoveIt → torque_controller_node → Gazebo 物理仿真
使用方法: ros2 launch ARV_V1_MOVEIT moveit_gazebo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    
    # 获取包路径
    pkg_moveit = get_package_share_directory('ARV_V1_MOVEIT')
    pkg_model = get_package_share_directory('ARV_V1_MODEL')
    
    # 使用 xacro 处理 URDF，传递绝对路径给 initial_positions_file
    xacro_file = os.path.join(pkg_moveit, 'config', 'ARV_V1_MODEL.urdf.xacro')
    initial_positions_file = os.path.join(pkg_moveit, 'config', 'initial_positions.yaml')
    
    # 使用 Command() 延迟执行 xacro，传递绝对路径
    robot_description_content = Command([
        'xacro', ' ', xacro_file,
        ' ', 'initial_positions_file:=', initial_positions_file
    ])
    
    # MoveIt 配置
    moveit_config = MoveItConfigsBuilder("ARV_V1_MODEL", package_name="ARV_V1_MOVEIT").to_moveit_configs()
    
    # 统一的 robot_description 参数
    unified_robot_description = {'robot_description': robot_description_content}
    
    # ros2_controllers 配置
    ros2_controllers_path = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')
    
    # ========== Gazebo 环境变量 ==========
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_model)
    )
    
    # ========== Gazebo 仿真节点 ==========
    gazebo_node = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[unified_robot_description]
    )
    
    # ========== Spawn 机器人到 Gazebo ==========
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'arv_v1',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',  # 抬高避免穿地
            '-allow_renaming', 'false'
        ],
        output='screen'
    )
    
    # ========== Gazebo Bridge（ROS2 ↔ Gazebo）==========
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_moveit, 'config', 'gz_bridge.yaml')
        }],
        output='screen'
    )
    
    # ========== 力矩转换节点（Float64MultiArray → 单个关节力矩）==========
    effort_to_gazebo = Node(
        package='ARV_V1_MOVEIT',
        executable='effort_to_gazebo.py',
        name='effort_to_gazebo',
        output='screen'
    )
    
    # controller_manager（使用 mock 硬件）
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            unified_robot_description,
            ros2_controllers_path
        ],
        output='screen',
        emulate_tty=True
    )
    
    # MoveIt move_group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            unified_robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.moveit_cpp,
            moveit_config.planning_scene_monitor,
            moveit_config.joint_limits,
            moveit_config.pilz_cartesian_limits
        ]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[
            unified_robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ],
        output='screen',
        emulate_tty=True
    )
    
    # 力矩控制器 Action Server
    torque_controller_node = Node(
        package='ARV_V1_MOVEIT',
        executable='torque_controller_node',
        name='torque_controller_action_server',
        output='screen',
        emulate_tty=True
    )
    
    # Spawn controllers
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    spawn_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        # ========== Gazebo 仿真环境 ==========
        gz_resource_path,
        gazebo_node,
        
        # ========== 机器人描述和控制 ==========
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot, gz_bridge, effort_to_gazebo]),  # 等 Gazebo 启动
        controller_manager_node,
        TimerAction(period=5.0, actions=[spawn_joint_state_broadcaster, spawn_effort_controller]),
        
        # ========== 力矩控制器 ==========
        TimerAction(period=6.0, actions=[torque_controller_node]),
        
        # ========== MoveIt 和可视化 ==========
        TimerAction(period=7.0, actions=[move_group_node]),
        TimerAction(period=9.0, actions=[rviz_node]),
    ])
