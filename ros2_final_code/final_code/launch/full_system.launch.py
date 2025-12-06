#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 시작 메시지
        LogInfo(msg="========================================"),
        LogInfo(msg="  Starting Full Robot System"),
        LogInfo(msg="========================================"),
        
        # 1. Tool Grip Node
        Node(
            package='dsr_rokey2',
            executable='tool_grip.py',
            name='tool_grip',
            namespace='dsr01',
            output='screen',
            emulate_tty=True,
            parameters=[],
            respawn=False,
            respawn_delay=2.0,
        ),
        
        # 2. Water Floor Action Server
        Node(
            package='dsr_rokey2',
            executable='water_floor.py',
            name='water_floor_server',
            namespace='dsr01',
            output='screen',
            emulate_tty=True,
            parameters=[],
            respawn=False,
            respawn_delay=2.0,
        ),
        
        # 3. Wipe Floor Action Server
        Node(
            package='dsr_rokey2',
            executable='wipe_floor.py',
            name='wipe_floor_server',
            namespace='dsr01',
            output='screen',
            emulate_tty=True,
            parameters=[],
            respawn=False,
            respawn_delay=2.0,
        ),
        
        # 4. Main Controller (Orchestrator)
        Node(
            package='dsr_rokey2',
            executable='main_controller.py',
            name='main_controller',
            namespace='dsr01',
            output='screen',
            emulate_tty=True,
            parameters=[],
            respawn=False,
            respawn_delay=2.0,
        ),
        
        
        # 완료 메시지
        LogInfo(msg="========================================"),
        LogInfo(msg="  All Nodes Started Successfully!"),
        LogInfo(msg="  Waiting for /tool_select command..."),
        LogInfo(msg="========================================"),
    ])
