#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只起 Nav2：先验 2D 代价地图（map_server）+ 规划/控制/BT。

它被 real_localization.launch.py 通过 nav2_watchdog.py 拉起（守护命令是纯字面量，
所以参数全部走默认值 / 环境变量，不靠 ":=" 传参）：

    环境变量 REAL_TEST_MAP        先验代价地图 yaml（默认 maps/prior_costmap.yaml）
    环境变量 REAL_TEST_NAV_PARAMS Nav2 参数文件（默认 config/nav2_real.yaml）
    环境变量 REAL_TEST_SIM_TIME   1 则 use_sim_time=true（默认 false）

也可以单独手动跑（定位已经在别的终端跑着的时候）：
    ros2 launch <repo>/real/launch/real_nav2_only.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REAL_TEST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REAL_TEST_DIR, 'config')
MAPS = os.path.join(REAL_TEST_DIR, 'maps')

DEFAULT_MAP = os.environ.get('REAL_TEST_MAP', os.path.join(MAPS, 'prior_costmap.yaml'))
DEFAULT_PARAMS = os.environ.get('REAL_TEST_NAV_PARAMS',
                                os.path.join(CONFIG, 'nav2_real.yaml'))
DEFAULT_SIM = 'true' if os.environ.get('REAL_TEST_SIM_TIME') == '1' else 'false'


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare = [
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP,
                              description='先验 2D 代价地图 yaml'),
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value=DEFAULT_SIM),
    ]

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )
    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server']}],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                         'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': 'False'}.items(),
    )

    ld = LaunchDescription()
    for a in declare:
        ld.add_action(a)
    ld.add_action(map_server)
    ld.add_action(lifecycle_map)
    ld.add_action(navigation)
    return ld
