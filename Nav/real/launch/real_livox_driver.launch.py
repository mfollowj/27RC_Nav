#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Livox Mid-360 驱动（自动适配本机网卡 IP）

为什么不直接用厂商的 msg_MID360_launch.py：
  1) 它的 user_config_path 里的 host_net_info 是**别人电脑的 IP**，
     驱动会因为绑不上地址 / 收不到数据而启动即崩；
  2) 雷达 IP 也是写死的；
  3) 这里把 xfer_format 固定成 1（CustomMsg，带逐点 offset_time），
     FAST-LIVO2 的运动去畸变必须靠它；publish_freq 默认 15 Hz，
     与机器人上 SCURM 那套驱动的实际值、以及 Nav/config/livo_mid360.yaml
     里的 scan_rate 保持一致（改一处就要三处一起改）。

本文件会读 `config/MID360_config.json`，然后：
  * 用 `lidar_ip`（默认取 json 里的值）所在子网，自动找到本机对应网卡的 IP，
    覆盖 `host_net_info` 的四个 IP —— 换机器（192.168.1.50 / .100 …）不用改文件；
  * 生成一份临时 json 交给驱动，原文件不动。

用法：
    ros2 launch <repo>/Nav/real/launch/real_livox_driver.launch.py
    ros2 launch ... host_ip:=192.168.1.100 lidar_ip:=192.168.1.126
    ros2 launch ... config:=/path/to/MID360_config.json
"""
import json
import os
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REAL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CFG = os.path.join(REAL_DIR, 'config', 'MID360_config.json')


def _detect_host_ip(lidar_ip):
    """找和雷达同一 /24 的本机网卡地址。"""
    try:
        out = subprocess.check_output(['ip', '-4', '-o', 'addr', 'show'],
                                      text=True, timeout=3)
    except Exception:
        return None
    net = '.'.join(lidar_ip.split('.')[:3]) + '.'
    for line in out.splitlines():
        parts = line.split()
        if 'inet' not in parts:
            continue
        addr = parts[parts.index('inet') + 1].split('/')[0]
        if addr.startswith(net):
            return addr
    return None


def _runtime_config(base, lidar_ip, host_ip):
    with open(base, 'r', encoding='utf-8') as f:
        cfg = json.load(f)
    mid = cfg.get('MID360', {})
    if lidar_ip:
        for item in cfg.get('lidar_configs', []):
            item['ip'] = lidar_ip
    if host_ip:
        for k in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            if k in mid.get('host_net_info', {}):
                mid['host_net_info'][k] = host_ip
    fd, tmp = tempfile.mkstemp(prefix='mid360_', suffix='.json')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        json.dump(cfg, f, indent=2)
    return tmp


def _setup(context, *args, **kwargs):
    base = LaunchConfiguration('config').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    publish_freq = LaunchConfiguration('publish_freq').perform(context)
    driver_rviz = LaunchConfiguration('driver_rviz').perform(context)
    host_arg = LaunchConfiguration('host_ip').perform(context).strip()
    lidar_arg = LaunchConfiguration('lidar_ip').perform(context).strip()

    with open(base, 'r', encoding='utf-8') as f:
        base_lidar = (json.load(f).get('lidar_configs') or [{}])[0].get('ip', '')
    lidar_ip = lidar_arg or base_lidar
    host_ip = host_arg or _detect_host_ip(lidar_ip) or ''

    if host_ip:
        cfg_path = _runtime_config(base, lidar_ip, host_ip)
    else:
        cfg_path = base
        print('[livox] 警告：没找到和雷达 %s 同网段的本机网卡，直接用原配置（可能绑不上）'
              % lidar_ip)
    print('[livox] user_config_path=%s  host=%s  lidar=%s'
          % (cfg_path, host_ip or '(未检测到)', lidar_ip))

    driver = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen',
        parameters=[{'xfer_format': 1,            # 1 = CustomMsg（带 offset_time）
                     'multi_topic': 0,
                     'data_src': 0,
                     'publish_freq': float(publish_freq),
                     'output_data_type': 0,
                     'frame_id': frame_id,
                     'lvx_file_path': '/tmp/livox_test.lvx',
                     'user_config_path': cfg_path,
                     'cmdline_input_bd_code': 'livox0000000001'}],
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(driver_rviz),
        arguments=['-d', os.path.join(REAL_DIR, 'rviz', 'real_nav.rviz')],
    )
    return [driver, rviz_node]


def generate_launch_description():
    declare = [
        DeclareLaunchArgument('config', default_value=DEFAULT_CFG,
                              description='雷达网络配置 json（host IP 会自动按网卡改写）'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('publish_freq', default_value='15.0',
                              description='点云发布频率；15 Hz 与机器人上实际跑的'
                                          '驱动 / LIO 的 scan_rate 一致'),
        DeclareLaunchArgument('host_ip', default_value='',
                              description='本机网卡 IP；留空则按雷达 IP 所在子网自动探测'),
        DeclareLaunchArgument('lidar_ip', default_value='',
                              description='雷达 IP；留空则用 json 里的值'),
        DeclareLaunchArgument('driver_rviz', default_value='false',
                              description='本文件自己再开一个 RViz（一般不需要）'),
    ]
    ld = LaunchDescription()
    for a in declare:
        ld.add_action(a)
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
