#!/usr/bin/env python3
"""mcu_bridge 启动文件。

    ros2 launch mcu_bridge mcu_bridge.launch.py                    # 用 config 默认值
    ros2 launch mcu_bridge mcu_bridge.launch.py dry_run:=true      # 只打日志不写串口
    ros2 launch mcu_bridge mcu_bridge.launch.py port:=/dev/ttyUSB1 output_frame:=map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mcu_bridge')
    default_params = os.path.join(pkg_share, 'config', 'mcu_bridge.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    output_frame = LaunchConfiguration('output_frame')
    base_frame = LaunchConfiguration('base_frame')
    dry_run = LaunchConfiguration('dry_run')
    log_level = LaunchConfiguration('log_level')

    declare = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='参数文件（默认包内 config/mcu_bridge.yaml）'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='串口设备'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('output_frame', default_value='odom',
                              description='发给 MCU 的坐标系（odom / map）'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='当前位姿取 output_frame <- base_frame；'
                                          '机器人上跑 SCURM 的 fast_lio 时是 imu_link'
                                          '（那套只发 odom->imu_link）'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = 不开串口，只打第一帧 hex'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    node = Node(
        package='mcu_bridge',
        executable='mcu_bridge_node',
        name='mcu_bridge',
        output='screen',
        parameters=[params_file, {
            'port': port,
            'baudrate': baudrate,
            'output_frame': output_frame,
            'base_frame': base_frame,
            'dry_run': dry_run,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription(declare + [node])
