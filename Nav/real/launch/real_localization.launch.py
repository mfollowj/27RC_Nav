#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 B：真机先验地图定位 + 重定位 + Nav2 导航

数据流（主路径，默认 track_prior:=false）：
    livox_ros_driver2 -> /livox/lidar -> livox_preprocess.py -> /livox/lidar_filtered
      -> fastlivo_mapping（纯 LIO：只发 odom->base_link，不碰先验图）
           -> /cloud_registered（odom 系点云）, /aft_mapped_to_init
      -> prior_map_localizer.py
           把 /cloud_registered 和先验点云地图做体素重叠度匹配 -> TF map->odom
           同时发 /localization_pose, /relocalization_result, /reloc_score, /reloc_status
      -> map_server（先验 2D 代价地图）+ Nav2（规划/控制/BT）-> /cmd_vel

可选路径（track_prior:=true）：FAST-LIVO2 自己把先验地图灌进体素地图做
    scan-to-prior-map 跟踪（抗漂移更强）。此时：
      * prior_map_localizer.py 以 --no-tf 只负责"开机喂一次位姿"到 /relocalization_result
      * reloc_pose_to_tf.py 负责发 map->odom 这一段 TF
    两条路径不要同时开，否则会打两套 map->odom（见 README 的"可选路径"）。

用法
----
    # 最常用：先建好图，有了 prior_map.pcd / prior_costmap.yaml 之后
    ros2 launch <repo>/real/launch/real_localization.launch.py \
        prior_map:=<repo>/real/maps/prior_map.pcd \
        map:=<repo>/real/maps/prior_costmap.yaml \
        meta:=<repo>/real/maps/prior_costmap.meta.json \
        imu_height:=0.30

    # 大致知道车停在先验图的 (2.0, 1.5)、朝 +x：给个初始猜测，收敛更快更稳
    ros2 launch ... initial_pose:="2.0,1.5,0"

    # LIO 已在别的终端跑着，只起定位 + Nav2
    ros2 launch ... front_end:=false

    # 只验证定位，不要 Nav2
    ros2 launch ... nav2:=false rviz:=true
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    front_end = LaunchConfiguration('front_end')
    livox_driver = LaunchConfiguration('livox_driver')
    preprocess = LaunchConfiguration('preprocess')
    track_prior = LaunchConfiguration('track_prior')
    nav2 = LaunchConfiguration('nav2')
    rviz = LaunchConfiguration('rviz')
    pub_livox_tf = LaunchConfiguration('pub_livox_tf')
    prior_map = LaunchConfiguration('prior_map')
    costmap = LaunchConfiguration('map')
    meta = LaunchConfiguration('meta')
    imu_height = LaunchConfiguration('imu_height')
    initial_pose = LaunchConfiguration('initial_pose')
    reloc_period = LaunchConfiguration('reloc_period')
    min_score = LaunchConfiguration('min_score')
    nav2_timeout = LaunchConfiguration('nav2_timeout')
    use_sim_time = LaunchConfiguration('use_sim_time')
    raw_topic = LaunchConfiguration('raw_topic')
    filtered_topic = LaunchConfiguration('filtered_topic')
    points_per_frame = LaunchConfiguration('points_per_frame')
    rate = LaunchConfiguration('rate')
    map_config = LaunchConfiguration('map_config')
    locate_config = LaunchConfiguration('locate_config')
    camera_config = LaunchConfiguration('camera_config')

    declare = [
        DeclareLaunchArgument('front_end', default_value='true',
                              description='true=本 launch 也起雷达抽稀和 LIO；false=LIO 已在别处跑'),
        DeclareLaunchArgument('livox_driver', default_value='false'),
        DeclareLaunchArgument('preprocess', default_value='true'),
        DeclareLaunchArgument('track_prior', default_value='false',
                              description='true=用 FAST-LIVO2 自带的先验图跟踪（可选路径）'),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('pub_livox_tf', default_value='true',
                              description='发 base_link->livox_frame 静态 TF（RViz 看点云用）'),
        DeclareLaunchArgument('prior_map',
                              default_value=os.path.join(MAPS, 'prior_map.pcd'),
                              description='先验 3D 点云地图（重定位用）'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(MAPS, 'prior_costmap.yaml'),
                              description='先验 2D 代价地图（Nav2 map_server 用）'),
        DeclareLaunchArgument('meta',
                              default_value=os.path.join(MAPS, 'prior_costmap.meta.json'),
                              description='建图阶段写出的 meta（读 ground_z）'),
        DeclareLaunchArgument('imu_height', default_value='0.30',
                              description='IMU/雷达离地高度；没有 meta 时用 -imu_height 当地面'),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description='可选初始猜测 "x,y,yaw_deg"（先验图坐标系）'),
        DeclareLaunchArgument('reloc_period', default_value='3.0',
                              description='重定位周期（秒）'),
        DeclareLaunchArgument('min_score', default_value='0.55',
                              description='体素重叠度阈值，低于该值的重定位结果丢弃'),
        DeclareLaunchArgument('nav2_timeout', default_value='75.0',
                              description='Nav2 单轮等待激活的最长时间（秒）'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='真机 false；跑仿真自测给 true'),
        DeclareLaunchArgument('raw_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('filtered_topic', default_value='/livox/lidar_filtered'),
        DeclareLaunchArgument('points_per_frame', default_value='4000'),
        DeclareLaunchArgument('rate', default_value='15.0'),
        DeclareLaunchArgument('map_config', default_value=SHARED_LIVO_CONFIG,
                              description='real/sim 共用参数（Nav/config/livo_mid360.yaml）'),
        DeclareLaunchArgument('locate_config', default_value=SHARED_LIVO_CONFIG,
                              description='同一份文件：locate 相关两行由参数覆盖，'
                                          '不再单独维护 livo_mid360_locate.yaml'),
        DeclareLaunchArgument('camera_config',
                              default_value=SHARED_CAMERA_CONFIG),
    ]

    # front_end 同时管住"抽稀 + LIO"两个节点：LIO 已经在别的终端跑的时候，
    # front_end:=false 不能又起第二个 fastlivo_mapping（两个节点抢同一个话题）。
    # 本 launch 用 ExecuteProcess 跑 rclpy 脚本，必须显式把 use_sim_time 传进去
    # （真机上是 false，跑仿真验证时必须给 true，否则时间戳一个是墙上时钟一个是 /clock）
    clk = ['--ros-args', '-p', ['use_sim_time:=', use_sim_time]]

    do_pre = PythonExpression(["'", front_end, "' == 'true' and '", preprocess, "' == 'true'"])
    do_lio = PythonExpression(["'", front_end, "' == 'true' and '", track_prior, "' == 'false'"])
    do_track = PythonExpression(["'", front_end, "' == 'true' and '", track_prior, "' == 'true'"])
    do_locate = PythonExpression(["'", track_prior, "' == 'false'"])
    do_seed = PythonExpression(["'", track_prior, "' == 'true'"])

    # ---------------- 前端：雷达驱动（可选）+ 抽稀 + LIO ----------------
    # 用自己的驱动 launch（host_net_info / lidar IP 都对得上本机与这台雷达）
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH, 'real_livox_driver.launch.py')),
        condition=IfCondition(livox_driver),
    )

    pre = ExecuteProcess(
        output='screen',
        condition=IfCondition(do_pre),
        cmd=['python3', os.path.join(SCRIPTS, 'livox_preprocess.py'),
             '--in-topic', raw_topic, '--out-topic', filtered_topic,
             '--points-per-frame', points_per_frame, '--rate', rate] + clk,
    )

    # 主路径：纯 LIO（不碰先验图），odom->base_link 由它负责
    livo_lio = Node(
        package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
        output='screen', condition=IfCondition(do_lio),
        parameters=[map_config, camera_config,
                    {'locate_in_prior_map': False},
                    {'common': {'lid_topic': filtered_topic}}],
    )

    # 可选路径：把先验点云地图灌进 FAST-LIVO2 的体素地图做跟踪
    livo_track = Node(
        package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
        output='screen', condition=IfCondition(do_track),
        parameters=[locate_config, camera_config,
                    {'locate_in_prior_map': True},
                    {'prior_map_path': prior_map},
                    {'common': {'lid_topic': filtered_topic}}],
    )

    # ---------------- 重定位：map->odom ----------------
    localizer = ExecuteProcess(
        output='screen', condition=IfCondition(do_locate),
        cmd=['python3', os.path.join(SCRIPTS, 'prior_map_localizer.py'),
             '--prior-map', prior_map,
             '--meta', meta,
             '--cloud-topic', '/cloud_registered',
             '--odom-topic', '/aft_mapped_to_init',
             '--map-frame', 'map', '--odom-frame', 'odom',
             '--imu-height', imu_height,
             '--period', reloc_period,
             '--min-score', min_score,
             ['--initial-pose=', initial_pose]] + clk,
    )

    # 可选路径专用：喂一次位姿给 FAST-LIVO2（--no-tf，不发 TF）
    seed = ExecuteProcess(
        output='screen', condition=IfCondition(do_seed),
        cmd=['python3', os.path.join(SCRIPTS, 'prior_map_localizer.py'),
             '--prior-map', prior_map,
             '--meta', meta,
             '--cloud-topic', '/cloud_registered',
             '--odom-topic', '/aft_mapped_to_init',
             '--map-frame', 'map', '--odom-frame', 'odom',
             '--imu-height', imu_height,
             '--period', '100000.0',
             '--no-tf',
             ['--initial-pose=', initial_pose]] + clk,
    )

    # 可选路径专用：把 /relocalization_result 变成 map->odom(camera_init/odom) 的 TF
    reloc_tf = ExecuteProcess(
        output='screen', condition=IfCondition(do_seed),
        cmd=['python3', os.path.join(SCRIPTS, 'reloc_pose_to_tf.py'),
             '--reloc-topic', '/relocalization_result',
             '--odom-topic', '/aft_mapped_to_init',
             '--map-frame', 'map', '--child-frame', 'odom'] + clk,
    )

    # ---------------- Nav2（map_server + 规划/控制/BT） ----------------
    # 用 nav2_watchdog.py 起：Fast DDS 下 lifecycle_manager 偶发卡死，守护进程整轮重启。
    # 命令保持**纯字面量** + real_nav2_only.launch.py（参数都有默认值），
    # 避免 ExecuteProcess 把 "name:=value" 拆成两个 argv（实测踩过）。
    nav = ExecuteProcess(
        output='screen', condition=IfCondition(nav2),
        cmd=['python3', os.path.join(SCRIPTS, 'nav2_watchdog.py'),
             '--node', 'bt_navigator',
             '--timeout', nav2_timeout,
             '--retries', '4',
             '--watch-timeout', '30.0',
             '--',
             'ros2', 'launch', os.path.join(LAUNCH, 'real_nav2_only.launch.py')],
    )

    # 静态 TF base_link -> livox_frame（只为 RViz 能显示雷达点云，不参与 LIO 计算）
    livox_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='livox_tf',
        output='log', condition=IfCondition(pub_livox_tf),
        arguments=['--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(rviz), arguments=['-d', RVIZ_CFG],
    )

    ld = LaunchDescription()
    for a in declare:
        ld.add_action(a)
    # 让被守护的 real_nav2_only.launch.py（纯字面量命令、不能带 ":=" 参数）
    # 也能拿到用户指定的先验代价地图
    ld.add_action(SetEnvironmentVariable('REAL_TEST_MAP', costmap))
    ld.add_action(driver)
    ld.add_action(pre)
    ld.add_action(TimerAction(period=3.0, actions=[livo_lio, livo_track]))
    ld.add_action(TimerAction(period=8.0, actions=[localizer, seed, reloc_tf]))
    ld.add_action(TimerAction(period=25.0, actions=[nav]))
    ld.add_action(livox_tf)
    ld.add_action(rviz_node)
    return ld
