#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""① 仿真建图：Gazebo(physics 驱动) + FAST-LIVO2（无视觉）+ 轨迹记录 + RViz

与真机 **同构**（M3）：同一套 LIO（fast_livo）、同一套话题、同一份参数文件
（`Nav/config/livo_mid360.yaml`）。平台差异只有两处，都写在本文件里：
    1) LiDAR-IMU 外参：仿真是 [0, 0, 0.06]（urdf: imu 0.44 / livox 0.50），
       真机是 Mid-360 出厂标定值 —— 用参数覆盖，不改共用 yaml；
    2) use_sim_time = true。

流程（配合仓库根目录 sim_*.sh 使用）：
    起本 launch -> 另开终端 sim_teleop.sh 把场地走一遍
    -> 另开终端 sim_save_map.sh 存成 prior_map.pcd / prior_costmap.yaml
    -> 再起 nav_prior_map.launch.py 用这张先验图导航

数据流（与真机一致）：
    /livox/lidar (CustomMsg) -> livox_preprocess.py -> /livox/lidar_filtered
      -> fastlivo_mapping -> /cloud_registered, /aft_mapped_to_init, TF odom->base_link

TF 约定：
  * odom->base_link 由 FAST-LIVO2 发布（sim.launch.py 传 publish_odom_tf:=false，
    轮速里程计改名 /odom_gz 只用于对照漂移）；
  * LIO 的 odom 原点 = 启动那一刻的 IMU 位置
    -> **建图和导航要用同一个出生点**（默认 -4.3, 1.5, yaw 0）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_repo_root():
    root = os.environ.get('RC_ROOT')
    if root and os.path.isdir(os.path.join(root, 'Nav', 'sim', 'worlds')):
        return root
    p = os.path.abspath(os.path.dirname(__file__))
    while p != '/':
        if os.path.isdir(os.path.join(p, 'Nav', 'sim', 'worlds')):
            return p
        p = os.path.dirname(p)
    return os.getcwd()


DEFAULT_FIELD_DIR = os.environ.get(
    'RC_FIELD_DIR', os.path.join(_find_repo_root(), 'Nav', 'sim', 'worlds'))

REPO_ROOT = _find_repo_root()
# ★ real / sim 共用（唯一真源）：LIO 参数 + 相机占位参数 + 雷达抽稀节点
SHARED_LIVO_CONFIG = os.path.join(REPO_ROOT, 'Nav', 'config', 'livo_mid360.yaml')
SHARED_CAMERA_CONFIG = os.path.join(REPO_ROOT, 'Nav', 'config', 'camera_pinhole.yaml')
PREPROCESS_SCRIPT = os.path.join(REPO_ROOT, 'Nav', 'real', 'scripts',
                                 'livox_preprocess.py')
# 仿真里 LiDAR 在 IMU 系下的平移 = livox(0.50) - imu(0.44)，与 urdf 一致
SIM_EXTRAINSIC_T = [0.0, 0.0, 0.06]
# 仿真里 LiDAR 与 IMU 同向（没倒装），LiDAR->IMU 旋转是单位阵
SIM_EXTRINSIC_R = [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_field_nav')

    try:
        get_package_share_directory('fast_livo')
    except Exception as exc:                      # noqa: BLE001
        raise RuntimeError(
            '[rc_field_nav] 找不到 fast_livo 包。请先编译 Deps 并 source 环境：\n'
            '  ./Deps/build_deps.sh fast_livo\n'
            '  source env.sh\n'
            '（原始错误：%s）' % exc)

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    # ★ sim.launch.py 的 rviz 参数要传 false（不要它自带的 2D RViz），
    #   但 ROS 2 launch 的配置是**全局共享**的，这个参数会把本文件自己的
    #   rviz 也改成 false —— 结果就是 RViz 根本不启动。
    #   所以先把用户给的值存到 rviz_self，后面就用 rviz_self 判断。
    rviz_self = LaunchConfiguration('rviz_self')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    livo_config = LaunchConfiguration('livo_config')
    camera_config = LaunchConfiguration('camera_config')

    preprocess = LaunchConfiguration('preprocess')
    points_per_frame = LaunchConfiguration('points_per_frame')
    rate = LaunchConfiguration('rate')

    record_traj = LaunchConfiguration('record_trajectory')
    traj_file = LaunchConfiguration('traj_file')
    lidar_samples = LaunchConfiguration('lidar_samples')
    lidar_downsample = LaunchConfiguration('lidar_downsample')

    default_traj = os.path.expanduser('~/rc_field_traj.txt')

    declare_args = [
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(DEFAULT_FIELD_DIR, 'robocon_track.world')),
        DeclareLaunchArgument('x', default_value='-4.3'),
        DeclareLaunchArgument('y', default_value='1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('livo_config', default_value=SHARED_LIVO_CONFIG,
                              description='real/sim 共用参数（Nav/config/livo_mid360.yaml）'),
        DeclareLaunchArgument('camera_config', default_value=SHARED_CAMERA_CONFIG,
                              description='FAST-LIVO2 无视觉模式也要求加载相机参数文件'),

        # 雷达抽稀/滤波：与真机跑同一个节点、同一组默认值
        DeclareLaunchArgument('preprocess', default_value='true',
                              description='是否跑 livox_preprocess.py（与真机同一节点）'),
        DeclareLaunchArgument('points_per_frame', default_value='4000',
                              description='每帧保留点数：3000~6000 之间最合适'),
        DeclareLaunchArgument('rate', default_value='15.0',
                              description='抽稀输出帧率 Hz（与真机 15 Hz 一致）'),

        DeclareLaunchArgument('record_trajectory', default_value='true',
                              description='是否把 LIO 轨迹记下来（生成 2D 代价地图时射线投射要用）'),
        DeclareLaunchArgument('traj_file', default_value=default_traj,
                              description='轨迹输出文件'),
        DeclareLaunchArgument('lidar_samples', default_value='20000',
                              description='Livox 插件每帧消费的 CSV 行数（=一个完整扫描花样）'),
        DeclareLaunchArgument('lidar_downsample', default_value='4'),
    ]

    # ---- 1) 仿真：Livox 后端 + 关掉插件的 odom TF（交给 LIO）----
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': 'false', 'gui': gui, 'world': world,
                          'publish_odom_tf': 'false',        # odom->base_link 交给 FAST-LIVO2
                          'odom_topic': 'odom_gz',
                          'use_sim_time': use_sim_time,
                          'lidar_samples': lidar_samples,
                          'lidar_downsample': lidar_downsample,
                          'x': x, 'y': y, 'yaw': yaw}.items(),
    )

    # ---- 2) 雷达抽稀/滤波（与真机同一个脚本、同一组参数）----
    pre = ExecuteProcess(
        output='screen', condition=IfCondition(preprocess),
        cmd=['python3', PREPROCESS_SCRIPT,
             '--in-topic', '/livox/lidar',
             '--out-topic', '/livox/lidar_filtered',
             '--points-per-frame', points_per_frame,
             '--rate', rate],
    )

    # ---- 3) FAST-LIVO2（无视觉，建图模式）----
    #   参数来自共用 yaml；只覆盖"平台差异"（外参）和仿真时间。
    livo = Node(
        package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
        output='screen',
        parameters=[livo_config, camera_config,
                    {'use_sim_time': use_sim_time},
                    # 仿真外参（LiDAR 在 IMU 系下 +6 cm）
                    {'extrin_calib': {'extrinsic_T': SIM_EXTRAINSIC_T}},
                    # 共用 yaml 是按"真机倒装"写的：仿真 IMU 没翻、话题也不同，覆盖回来
                    {'extrin_calib': {'extrinsic_R': SIM_EXTRINSIC_R}},
                    {'common': {'imu_topic': '/livox/imu'}},
                    {'locate_in_prior_map': False}],
    )

    # ---- 4) 轨迹记录：给 map3d_to_costmap.py 做射线投射（生成空闲区）----
    #   ★ 话题是 fast_livo 的 /aft_mapped_to_init（fast_lio 当年是 /Odometry）
    traj = Node(
        package='rc_field_nav',
        executable='record_trajectory.py',
        name='record_trajectory',
        output='screen',
        condition=IfCondition(record_traj),
        arguments=['-t', '/aft_mapped_to_init', '-o', traj_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- 5) 全程累积点云：边建图边把 /cloud_registered 攒成一张稠密地图 ----
    #   为什么不用 /Laser_map：那是 LIO 内部降采样后的地图，做 2D 代价地图会
    #   "墙是虚线、空地随机出点"。这里改成累积每一帧配准后的点云（体素去重）。
    #   落盘位置用临时名 accum.pcd，由 save_prior_map.sh 触发后改名成 <名字>_map.pcd。
    accumulator = Node(
        package='rc_field_nav',
        executable='save_cloud_pcd.py',
        name='cloud_accumulator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--topic', '/cloud_registered',
                   '-o', os.path.join(pkg_share, 'maps', 'accum.pcd'),
                   '--accumulate', '--trigger-topic', '/save_prior_map',
                   '--voxel', '0.02', '--verbose'],
    )

    # ---- 6) RViz（3D 视图）----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',        # 输出到终端：RViz 起不来时能直接看到原因
        condition=IfCondition(rviz_self),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'field_lio.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)
    # 在 sim 包进来之前先把用户给的 rviz 存起来
    ld.add_action(SetLaunchConfiguration('rviz_self', rviz))
    ld.add_action(sim)
    # LIO 要等雷达/IMU 有数据
    ld.add_action(TimerAction(period=8.0, actions=[pre]))
    ld.add_action(TimerAction(period=10.0, actions=[livo, traj, accumulator]))
    ld.add_action(rviz_node)
    return ld
