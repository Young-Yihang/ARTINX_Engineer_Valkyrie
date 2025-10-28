#!/usr/bin/env python3
# ============================================================
# Gazebo Harmonic 仿真 + 力矩控制器启动文件
# ============================================================

import os
import re
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def resolve_package_uris(urdf_content):
    """
    将 URDF 中的 package:// URI 替换为绝对路径
    Gazebo Harmonic 不能直接解析 package:// URI
    """
    def replace_uri(match):
        package_name = match.group(1)
        resource_path = match.group(2)
        try:
            pkg_path = get_package_share_directory(package_name)
            return f'file://{pkg_path}/{resource_path}'
        except Exception as e:
            print(f"警告: 无法解析包 {package_name}: {e}")
            return match.group(0)
    
    # 替换 package://PACKAGE_NAME/path 为 file:///absolute/path
    pattern = r'package://([^/]+)/(.+?)"'
    return re.sub(pattern, lambda m: replace_uri(m) + '"', urdf_content)

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('ARV_V1_MOVEIT')
    model_pkg_share = get_package_share_directory('ARV_V1_MODEL')
    
    # URDF 文件路径
    urdf_path = os.path.join(model_pkg_share, 'urdf', 'ARV_V1_MODEL.urdf')
    
    # 读取原始 URDF（不做 URI 转换，让 Gazebo 通过环境变量自己解析）
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # 注意：不再使用 resolve_package_uris 转换
    # Gazebo 会通过 GZ_SIM_RESOURCE_PATH 自动将 package:// 解析为 model://
    
    # 设置 Gazebo 资源路径
    # model_pkg_share = .../install/ARV_V1_MODEL/share/ARV_V1_MODEL
    # Gazebo 需要 .../install/ARV_V1_MODEL/share 才能解析 model://ARV_V1_MODEL/meshes/
    resource_path = os.path.dirname(model_pkg_share)  # 返回 .../share 目录
    
    print(f"\n{'='*60}")
    print(f"Gazebo 资源路径配置:")
    print(f"GZ_SIM_RESOURCE_PATH = {resource_path}")
    print(f"模型包路径 = {model_pkg_share}")
    print(f"Meshes 路径 = {os.path.join(model_pkg_share, 'meshes')}")
    print(f"{'='*60}\n")
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )
    
    return LaunchDescription([
        # ========== 0. 设置环境变量 ==========
        gz_resource_path,
        
        # ========== 1. 启动 Gazebo Harmonic（新版）==========
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],  # ← 修改:使用 gz sim
            output='screen'
        ),
        
        # ========== 2. 发布机器人描述（URDF）==========
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        
        # ========== 3. 等待 Gazebo 启动后再生成机器人 ==========
        TimerAction(
            period=3.0,  # 等待 3 秒
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-topic', '/robot_description',
                        '-name', 'arv_v1',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '20.0'
                    ],
                    output='screen'
                )
            ]
        ),
        
        # ========== 4. Gazebo 到 ROS 2 的桥接（关节状态）==========
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/world/empty/model/arv_v1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen',
            remappings=[
                ('/world/empty/model/arv_v1/joint_state', '/joint_states')
            ]
        ),
        
        # ========== 5. 启动力矩控制器节点 ==========
        TimerAction(
            period=5.0,  # 等待 5 秒（确保 Gazebo 和桥接都启动）
            actions=[
                Node(
                    package='ARV_V1_MOVEIT',
                    executable='torque_controller_node',
                    name='torque_controller_node',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),
    ])