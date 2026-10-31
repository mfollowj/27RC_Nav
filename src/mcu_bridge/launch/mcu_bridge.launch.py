#!/usr/bin/env python3
"""mcu_bridge 启动文件。

    ros2 launch mcu_bridge mcu_bridge.launch.py                    # 用 config 默认值
    ros2 launch mcu_bridge mcu_bridge.launch.py dry_run:=true      # 只打日志不写串口
    ros2 launch mcu_bridge mcu_bridge.launch.py port:=/dev/ttyUSB1 output_frame:=map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_bridge(context):
    # 仅将显式提供的启动参数覆盖到 YAML，保留自定义配置中的串口与坐标系。
    overrides = {}
    for key, value_type in [('port', str), ('baudrate', int),
                            ('output_frame', str), ('base_frame', str),
                            ('dry_run', bool)]:
        value = LaunchConfiguration(key).perform(context)
        if value:
            overrides[key] = ParameterValue(value, value_type=value_type)
    return [Node(
        package='mcu_bridge',
        executable='mcu_bridge_node',
        name='mcu_bridge',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('mcu_bridge')
    default_params = os.path.join(pkg_share, 'config', 'mcu_bridge.yaml')

    declare = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='参数文件（默认包内 config/mcu_bridge.yaml）'),
        DeclareLaunchArgument('port', default_value='',
                              description='串口设备；未指定时使用 YAML'),
        DeclareLaunchArgument('baudrate', default_value='',
                              description='串口波特率；未指定时使用 YAML'),
        DeclareLaunchArgument('output_frame', default_value='',
                              description='发给 MCU 的坐标系；未指定时使用 YAML'),
        DeclareLaunchArgument('base_frame', default_value='',
                              description='当前位姿取 output_frame <- base_frame；'
                                          '机器人上跑 SCURM 的 fast_lio 时是 imu_link'
                                          '（那套只发 odom->imu_link）'),
        DeclareLaunchArgument('dry_run', default_value='',
                              description='true = 不开串口，只打第一帧 hex；未指定时使用 YAML'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    return LaunchDescription(declare + [OpaqueFunction(function=launch_bridge)])
