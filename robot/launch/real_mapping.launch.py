#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 A：真机 Mid-360 建图（采集先验地图）

一条命令拉起：
    livox_ros_driver2 -> /livox/lidar (CustomMsg)
      -> livox_preprocess.py   抽稀 + 滤波 -> /livox/lidar_filtered
      -> fastlivo_mapping (FAST-LIVO2, 无视觉, 只用 LiDAR+IMU)
           -> TF odom->base_link, /aft_mapped_to_init, /cloud_registered, /Laser_map
      -> traj_recorder.py      把轨迹写到 maps/prior_traj.txt

跑完之后**不要退出**，另开终端执行：
    ./robot/scripts/save_prior_map.sh
它会存先验点云、轨迹、2D 代价地图和 meta.json。

用法
----
    # 雷达驱动已经在别处跑着（推荐：驱动单独一个终端，方便看它的状态）
    ros2 launch <repo>/robot/launch/real_mapping.launch.py

    # 让本 launch 顺便把 livox 驱动也拉起来
    ros2 launch <repo>/robot/launch/real_mapping.launch.py livox_driver:=true

    # 抽稀到 6000 点/帧、开 RViz 看建图效果
    ros2 launch <repo>/robot/launch/real_mapping.launch.py points_per_frame:=6000 rviz:=true
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REAL_TEST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REAL_TEST_DIR, 'scripts')
CONFIG = os.path.join(REAL_TEST_DIR, 'config')
# ★ real / sim **共用**的 FAST-LIVO2 参数（M3：两边读同一份 yaml，改一处两边都变）
SHARED_LIVO_CONFIG = os.path.join(
    os.path.dirname(REAL_TEST_DIR), 'config', 'livo_mid360.yaml')
# 相机参数（img_en=0 时只用于满足 FAST-LIVO2 的参数声明）同样 real/sim 共用
SHARED_CAMERA_CONFIG = os.path.join(
    os.path.dirname(REAL_TEST_DIR), 'config', 'camera_pinhole.yaml')
MAPS = os.path.join(REAL_TEST_DIR, 'maps')
LAUNCH = os.path.join(REAL_TEST_DIR, 'launch')
RVIZ_CFG = os.path.join(REAL_TEST_DIR, 'rviz', 'real_nav.rviz')


def generate_launch_description():
    livox_driver = LaunchConfiguration('livox_driver')
    preprocess = LaunchConfiguration('preprocess')
    livo = LaunchConfiguration('livo')
    rviz = LaunchConfiguration('rviz')
    record_traj = LaunchConfiguration('record_traj')
    raw_topic = LaunchConfiguration('raw_topic')
    filtered_topic = LaunchConfiguration('filtered_topic')
    points_per_frame = LaunchConfiguration('points_per_frame')
    rate = LaunchConfiguration('rate')
    min_range = LaunchConfiguration('min_range')
    max_range = LaunchConfiguration('max_range')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')
    min_reflectivity = LaunchConfiguration('min_reflectivity')
    livo_config = LaunchConfiguration('livo_config')
    camera_config = LaunchConfiguration('camera_config')
    traj_out = LaunchConfiguration('traj_out')
    lio_delay = LaunchConfiguration('lio_delay')
    map_accum = LaunchConfiguration('map_accum')
    map_out = LaunchConfiguration('map_out')
    rviz_config = LaunchConfiguration('rviz_config')
    pub_livox_tf = LaunchConfiguration('pub_livox_tf')

    declare = [
        DeclareLaunchArgument('livox_driver', default_value='false',
                              description='是否由本 launch 同时拉起 livox_ros_driver2'),
        DeclareLaunchArgument('preprocess', default_value='true',
                              description='是否跑雷达抽稀/滤波节点'),
        DeclareLaunchArgument('livo', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(REAL_TEST_DIR, 'rviz', 'real_lio.rviz'),
            description='建图阶段用 Fixed Frame = odom 的那份 RViz 配置'
                        '（real_nav.rviz 是给定位/导航阶段用的，Fixed Frame = map）'),
        DeclareLaunchArgument('pub_livox_tf', default_value='true',
                              description='发 base_link->livox_frame 静态 TF，'
                                          '让 RViz 能显示雷达原始/抽稀点云'),
        DeclareLaunchArgument('record_traj', default_value='true',
                              description='是否记录 odom 轨迹（转 2D 图时要用）'),
        DeclareLaunchArgument('map_accum', default_value='true',
                              description='是否边建图边把 /cloud_registered 累加成整张先验地图'
                                          '（save_prior_map.sh 就靠它落盘）'),
        DeclareLaunchArgument('map_out', default_value=os.path.join(MAPS, 'prior_map.pcd')),
        DeclareLaunchArgument('raw_topic', default_value='/livox/lidar',
                              description='驱动原始话题（CustomMsg）'),
        DeclareLaunchArgument('filtered_topic', default_value='/livox/lidar_filtered',
                              description='抽稀后的话题（FAST-LIVO2 的 lid_topic）'),
        DeclareLaunchArgument('points_per_frame', default_value='4000',
                              description='每帧保留点数：3000~6000 之间最合适'),
        DeclareLaunchArgument('rate', default_value='15.0', description='输出帧率 Hz'),
        DeclareLaunchArgument('min_range', default_value='0.5'),
        DeclareLaunchArgument('max_range', default_value='60.0'),
        DeclareLaunchArgument('min_z', default_value='-3.0'),
        DeclareLaunchArgument('max_z', default_value='10.0'),
        DeclareLaunchArgument('min_reflectivity', default_value='0'),
        DeclareLaunchArgument('livo_config', default_value=SHARED_LIVO_CONFIG,
                              description='real/sim 共用参数（config/livo_mid360.yaml）'),
        DeclareLaunchArgument('camera_config',
                              default_value=SHARED_CAMERA_CONFIG,
                              description='FAST-LIVO2 无视觉模式也要求加载相机参数文件'),
        DeclareLaunchArgument('traj_out', default_value=os.path.join(MAPS, 'prior_traj.txt')),
        DeclareLaunchArgument('lio_delay', default_value='3.0',
                              description='等抽稀节点起来后再启动 LIO 的秒数'),
    ]

    # ---- 1) 雷达驱动（可选，默认由用户自己起） ----
    # ★ 用自己的驱动 launch（配置里 host_net_info 必须是本机网卡 IP、
    #   lidar_configs 里必须是这台雷达的真实 IP）——厂商那份 msg_MID360_launch.py
    #   用的是别人电脑的 IP，直接照抄会连不上雷达。见 README 第 9 节。
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH, 'real_livox_driver.launch.py')),
        condition=IfCondition(livox_driver),
    )

    # ---- 2) 雷达抽稀/滤波（本工程自定义节点，纯 Python，不需要 colcon 编译） ----
    pre = ExecuteProcess(
        output='screen',
        condition=IfCondition(preprocess),
        cmd=['python3', os.path.join(SCRIPTS, 'livox_preprocess.py'),
             '--in-topic', raw_topic,
             '--out-topic', filtered_topic,
             '--points-per-frame', points_per_frame,
             '--rate', rate,
             '--min-range', min_range,
             '--max-range', max_range,
             '--min-z', min_z,
             '--max-z', max_z,
             '--min-reflectivity', min_reflectivity],
    )

    # ---- 3) FAST-LIVO2（无视觉 / LiDAR-only 紧耦合里程计 + 建图） ----
    livo_node = Node(
        package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
        output='screen', condition=IfCondition(livo),
        parameters=[livo_config,
                    camera_config,
                    {'locate_in_prior_map': False},
                    # 话题由 launch 统一决定（覆盖 yaml 里的默认值）
                    {'common': {'lid_topic': filtered_topic}}],
    )

    # ---- 4) 轨迹记录（保存地图时要靠它推地面高度 + 生成空闲区） ----
    traj = ExecuteProcess(
        output='screen',
        condition=IfCondition(record_traj),
        cmd=['python3', os.path.join(SCRIPTS, 'traj_recorder.py'),
             '-t', '/aft_mapped_to_init', '-o', traj_out],
    )

    # ---- 5) 全局地图累积器：边跑边攒，收到 /save_prior_map 就写盘 ----
    # ★ /cloud_registered 只是"当前视野"，跑完再抓只能拿到最后一小片，
    #   所以必须从一开始就累积（内存只跟地图大小有关，不跟跑多久有关）。
    accum = ExecuteProcess(
        output='screen', condition=IfCondition(map_accum),
        cmd=['python3', os.path.join(SCRIPTS, 'save_cloud_pcd.py'),
             '--topic', '/cloud_registered', '-o', map_out,
             '--accumulate', '--trigger-topic', '/save_prior_map',
             '--voxel', '0.05', '--verbose'],
    )

    # ---- 6) 可选的静态 TF：base_link -> livox_frame ----
    # ★ 建图阶段只有 odom->base_link（LIO 发的），雷达点云却是 livox_frame 系，
    #   没有这一段 TF，RViz 里就**看不到原始/抽稀后的点云**（会一直丢消息）。
    #   数值就是 FAST-LIVO2 的 extrin_calib.extrinsic_T（LiDAR 在 IMU 系下的位置）。
    #   注意：只影响 RViz 显示，不参与 LIO 计算（LIO 用自己的 extrinsic 参数）。
    livox_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='livox_tf',
        output='log', condition=IfCondition(pub_livox_tf),
        arguments=['--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        parameters=[{'use_sim_time': False}],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(rviz), arguments=['-d', rviz_config],
    )

    ld = LaunchDescription()
    for a in declare:
        ld.add_action(a)
    ld.add_action(driver)
    ld.add_action(pre)
    ld.add_action(TimerAction(period=lio_delay, actions=[livo_node]))
    ld.add_action(TimerAction(period=2.0, actions=[traj, accum]))
    ld.add_action(livox_tf)
    ld.add_action(rviz_node)
    return ld
