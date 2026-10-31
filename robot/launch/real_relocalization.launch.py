#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机·3D 全局重定位链路（可在大范围漂移后恢复）

与 real_nav.sh（旧链路：4-DOF 体素重叠 + 纯 map→odom 修正）的区别：
    * 重定位用**原始雷达**做 TEASER++ 6-DOF 全局搜索 + GICP 精配准，不看 odom；
    * 结果经 reloc_bridge 换算成 map←base，喂给 FAST-LIVO2；
    * FAST-LIVO2 收到"稳定的新位姿且与已生效位姿差得够多"时，**把 odom 归零并
      用先验图重建体素地图**（need_map_reinit），所以 LIO 自己也能被拉回来。

数据流：
    livox 驱动 -> /livox/lidar ─┬-> livox_preprocess -> /livox/lidar_filtered -> fastlivo_mapping
                                └-> teaser_gicp_node -> /icp_result (map←livox)
                                                              │
                                          reloc_bridge  P = S·E⁻¹ (map←base)
                                                              ├-> /relocalization_result -> fastlivo
                                                              └-> TF map->odom

用法：
    ./real_reloc.sh                                   # 默认用 robot/maps/prior_map.pcd
    ./real_reloc.sh map_path:=/abs/x.pcd rviz:=true
    ./real_reloc.sh livox_driver:=false               # 驱动已在别的终端跑
    ./real_reloc.sh nav2:=true map:=robot/maps/prior_costmap.yaml   # 需要导航时
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REAL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REAL_DIR, 'scripts')
CONFIG = os.path.join(REAL_DIR, 'config')
# ★ real / sim **共用**的 FAST-LIVO2 参数（M3：两边读同一份 yaml，改一处两边都变）
SHARED_LIVO_CONFIG = os.path.join(
    os.path.dirname(REAL_DIR), 'config', 'livo_mid360.yaml')
# 相机参数（img_en=0 时只用于满足 FAST-LIVO2 的参数声明）同样 real/sim 共用
SHARED_CAMERA_CONFIG = os.path.join(
    os.path.dirname(REAL_DIR), 'config', 'camera_pinhole.yaml')
MAPS = os.path.join(REAL_DIR, 'maps')
LAUNCH = os.path.join(REAL_DIR, 'launch')
RVIZ_CFG = os.path.join(REAL_DIR, 'rviz', 'real_reloc.rviz')


def generate_launch_description():
    livox_driver = LaunchConfiguration('livox_driver')
    preprocess = LaunchConfiguration('preprocess')
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    raw_topic = LaunchConfiguration('raw_topic')
    filtered_topic = LaunchConfiguration('filtered_topic')
    points_per_frame = LaunchConfiguration('points_per_frame')
    rate = LaunchConfiguration('rate')
    map_path = LaunchConfiguration('map_path')
    map_frame = LaunchConfiguration('map_frame')
    nav2_map = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')
    map_config = LaunchConfiguration('map_config')
    camera_config = LaunchConfiguration('camera_config')
    lio_delay = LaunchConfiguration('lio_delay')
    reinit_trans = LaunchConfiguration('reinit_trans_thre')
    reinit_rot = LaunchConfiguration('reinit_rot_thre')
    stable_count = LaunchConfiguration('stable_count')
    stable_trans = LaunchConfiguration('stable_trans')
    stable_rot = LaunchConfiguration('stable_rot')
    min_change_trans = LaunchConfiguration('min_change_trans')
    min_change_rot = LaunchConfiguration('min_change_rot')
    map_voxel = LaunchConfiguration('map_voxel')
    teaser_inlier = LaunchConfiguration('teaser_inlier')
    publish_after = LaunchConfiguration('publish_after')

    declare = [
        DeclareLaunchArgument('livox_driver', default_value='true',
                              description='是否由本 launch 拉起 livox_ros_driver2'),
        DeclareLaunchArgument('preprocess', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='是否顺带起 Nav2（需要 2D 图；本阶段默认关）'),
        DeclareLaunchArgument('map_path', default_value=os.path.join(MAPS, 'prior_map.pcd'),
                              description='先验 3D 点云（重定位与 LIO 共用同一份）'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('map', default_value=os.path.join(MAPS, 'prior_costmap.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(CONFIG, 'nav2_real.yaml')),
        DeclareLaunchArgument('raw_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('filtered_topic', default_value='/livox/lidar_filtered'),
        DeclareLaunchArgument('points_per_frame', default_value='4000'),
        DeclareLaunchArgument('rate', default_value='15.0'),
        DeclareLaunchArgument('map_config', default_value=SHARED_LIVO_CONFIG,
                              description='real/sim 共用参数（config/livo_mid360.yaml）'),
        DeclareLaunchArgument('camera_config',
                              default_value=SHARED_CAMERA_CONFIG),
        DeclareLaunchArgument('lio_delay', default_value='3.0'),
        DeclareLaunchArgument('reinit_trans_thre', default_value='0.8',
                              description='稳定重定位与已生效位姿差超过它就重建体素地图 (m)'),
        DeclareLaunchArgument('reinit_rot_thre', default_value='15.0'),
        DeclareLaunchArgument('stable_count', default_value='3',
                              description='reloc_bridge：连续多少帧一致才算稳定'),
        DeclareLaunchArgument('stable_trans', default_value='0.25'),
        DeclareLaunchArgument('stable_rot', default_value='8.0'),
        DeclareLaunchArgument('min_change_trans', default_value='0.5'),
        DeclareLaunchArgument('min_change_rot', default_value='10.0'),
        DeclareLaunchArgument('map_voxel', default_value='0.3',
                              description='重定位节点对先验图的降采样体素'),
        DeclareLaunchArgument('teaser_inlier', default_value='30',
                              description='TEASER 内点阈值（场地难匹配时可降到 10~15）'),
        DeclareLaunchArgument('publish_after', default_value='3',
                              description='GICP 连续多少帧达标后开始每帧连续输出位姿'),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH, 'real_livox_driver.launch.py')),
        condition=IfCondition(livox_driver),
    )

    pre = ExecuteProcess(
        output='screen', condition=IfCondition(preprocess),
        cmd=['python3', os.path.join(SCRIPTS, 'livox_preprocess.py'),
             '--in-topic', raw_topic, '--out-topic', filtered_topic,
             '--points-per-frame', points_per_frame, '--rate', rate],
    )

    # FAST-LIVO2：先验图跟踪模式；locate_in_prior_map 会等 /relocalization_result
    livo = Node(
        package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
        output='screen',
        parameters=[map_config, camera_config,
                    {'locate_in_prior_map': True},
                    {'prior_map_path': map_path},
                    {'allow_map_reinit': True},
                    {'map_reinit_trans_thre': reinit_trans},
                    {'map_reinit_rot_thre': reinit_rot},
                    {'common': {'lid_topic': filtered_topic}}],
    )

    # 6-DOF 全局重定位：直接吃原始 CustomMsg（与 odom 无关）
    teaser = Node(
        package='relocalization', executable='teaser_gicp_node', name='teaser_gicp_node',
        output='screen',
        parameters=[{'map_path': map_path,
                     'map_frame_id': map_frame,
                     'pcl_type': 'livox',
                     'shutdown_on_success': False,
                     'map_voxel_leaf_size': map_voxel,
                     'cloud_voxel_leaf_size': map_voxel,
                     'teaser_inlier_threshold': teaser_inlier,
                     'publish_after_converged': publish_after}],
    )

    bridge = ExecuteProcess(
        output='screen',
        cmd=['python3', os.path.join(SCRIPTS, 'reloc_bridge.py'),
             '--map-frame', map_frame,
             '--stable-count', stable_count,
             '--stable-trans', stable_trans,
             '--stable-rot', stable_rot,
             '--min-change-trans', min_change_trans,
             '--min-change-rot', min_change_rot],
    )

    livox_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='livox_tf',
        output='log',
        arguments=['--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        parameters=[{'use_sim_time': False}],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(rviz), arguments=['-d', RVIZ_CFG],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH, 'real_nav2_only.launch.py')),
        condition=IfCondition(nav2),
        launch_arguments={'map': nav2_map, 'params_file': nav2_params}.items(),
    )

    ld = LaunchDescription()
    # TEASER++/small_gicp 的动态库不在 ROS 环境里：直接把 third_party/install 的 lib 目录
    # 补进 LD_LIBRARY_PATH，这样即使不经 real_reloc.sh / env.sh 直接 launch 也能找到。
    root = os.path.dirname(REAL_DIR)                          # 仓库根
    deps = os.environ.get('RC_DEPS_INSTALL', os.path.join(root, 'third_party', 'install'))
    lib_dirs = [d for d in (os.path.join(deps, 'lib'),
                            os.path.join(deps, 'teaserpp', 'lib'),
                            os.path.join(deps, 'small_gicp', 'lib'))
                if os.path.isdir(d)]
    ld.add_action(SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        ':'.join(lib_dirs + [os.environ.get('LD_LIBRARY_PATH', '')])))
    for a in declare:
        ld.add_action(a)
    ld.add_action(driver)
    ld.add_action(pre)
    ld.add_action(teaser)            # 先起重定位，LIO 在等它
    ld.add_action(bridge)
    ld.add_action(livox_tf)
    ld.add_action(TimerAction(period=lio_delay, actions=[livo]))
    ld.add_action(rviz_node)
    # Nav2 晚一点起：给重定位留出首次收敛时间（bootstrap TF 让 Nav2 能启动，
    # 但真实 map->odom 要等 reloc_bridge 接受第一帧结果）。
    ld.add_action(TimerAction(period=8.0, actions=[nav2_launch]))
    return ld
