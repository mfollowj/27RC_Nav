#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""② 仿真导航：先验地图 + **6-DOF 重定位（TEASER++ / VGICP）+ FAST-LIVO2** + Nav2

与真机 **同构**（M3）：和 `robot/launch/real_relocalization.launch.py` 走同一条链路：

    Gazebo (Livox Mid-360 + IMU)
        ├─ /livox/lidar ─> livox_preprocess -> /livox/lidar_filtered ─┐
        └─ /livox/imu ────────────────────────────────────────────────┤
                                                                      v
    先验图 maps/prior_map.pcd ─> teaser_gicp_node ─> /icp_result (map←livox)
                                                          │
                                          reloc_bridge  P = S·E⁻¹ (map←base)
                                                          ├─> /relocalization_result
                                                          └─> TF map->odom
                                                                      │
    /relocalization_result ─> FAST-LIVO2（locate_in_prior_map）: 收到稳定新位姿就
                              重置 odom + 用先验图重建体素地图（need_map_reinit）
                                                                      │
    map_server(先验 2D 代价地图) + Nav2 (planner/controller/BT) ─────────┴─> /cmd_vel

★ 出生点必须与建图时相同（默认 -4.3, 1.5, yaw 0）：LIO 的 odom 原点就是启动位置。
★ 车要先静止等 TEASER 收敛（reloc_bridge 会打印 FIRST），再发导航目标。

用法
----
    ros2 launch rc_field_nav sim_navigation.launch.py gui:=true
    # 旧链路（4-DOF 体素重叠，仅用于 A/B 对比）：
    ros2 launch rc_field_nav sim_navigation.launch.py reloc_impl:=voxel
    # 目标点：
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 3.9, y: 4.5}, orientation: {w: 1.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _find_repo_root():
    root = os.environ.get('RC_ROOT')
    if root and os.path.isdir(os.path.join(root, 'simulation', 'worlds')):
        return root
    p = os.path.abspath(os.path.dirname(__file__))
    while p != '/':
        if os.path.isdir(os.path.join(p, 'simulation', 'worlds')):
            return p
        p = os.path.dirname(p)
    return os.getcwd()


DEFAULT_FIELD_DIR = os.environ.get(
    'RC_FIELD_DIR', os.path.join(_find_repo_root(), 'simulation', 'worlds'))

REPO_ROOT = _find_repo_root()
# ★ robot / simulation 共用参数与脚本。
SHARED_LIVO_CONFIG = os.path.join(REPO_ROOT, 'config', 'livo_mid360.yaml')
SHARED_CAMERA_CONFIG = os.path.join(REPO_ROOT, 'config', 'camera_pinhole.yaml')
PREPROCESS_SCRIPT = os.path.join(REPO_ROOT, 'robot', 'scripts',
                                 'livox_preprocess.py')
BRIDGE_SCRIPT = os.path.join(REPO_ROOT, 'robot', 'scripts', 'reloc_bridge.py')

# 仿真车几何（与 urdf 默认值一致，改了 urdf 的 mount 就要同步改这里）：
#   imu_mount_z   = 0.30   (omni4_field.urdf.xacro)
#   lidar_mount_z = 0.36   (omni4_field.urdf.xacro)
SIM_IMU_Z = 0.30
SIM_LIDAR_Z = 0.36
# LIO 要的是"LiDAR 在 IMU 系下"的平移；bridge 要的是"LiDAR 在 base_link 系下"的平移
SIM_LIO_EXTRINSIC_T = [0.0, 0.0, SIM_LIDAR_Z - SIM_IMU_Z]
# 仿真里 LiDAR 与 IMU 同向（没倒装），LiDAR->IMU 旋转是单位阵
SIM_EXTRINSIC_R = [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
SIM_BRIDGE_EXT_Z = SIM_LIDAR_Z


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_field_nav')
    get_package_share_directory('nav2_bringup')     # 缺 Nav2 时尽早报错

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    # ★ sim.launch.py 的 rviz 参数要传 false（不要它自带的 2D RViz），
    #   但 ROS 2 launch 的配置是**全局共享**的，这个参数会把本文件自己的
    #   rviz 也改成 false —— 结果就是 RViz 根本不启动。
    #   所以先把用户给的值存到 rviz_self，后面就用 rviz_self 判断。
    rviz_self = LaunchConfiguration('rviz_self')
    field = LaunchConfiguration('field')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    prior_map = LaunchConfiguration('prior_map')
    costmap_yaml = LaunchConfiguration('map')
    # 2D 图的 meta.json（同一次存图产出）里有权威的 ground_z，
    # 旧 4-DOF 定位器直接读它，不再写死数值
    costmap_meta = PythonExpression(
        ["'", costmap_yaml, "'.replace('.yaml', '.meta.json')"])
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    livo_config = LaunchConfiguration('livo_config')
    camera_config = LaunchConfiguration('camera_config')

    reloc_impl = LaunchConfiguration('reloc_impl')
    use_teaser = PythonExpression(["'", reloc_impl, "' == 'teaser'"])
    use_voxel = PythonExpression(["'", reloc_impl, "' == 'voxel'"])

    teaser_inlier = LaunchConfiguration('teaser_inlier')
    map_voxel = LaunchConfiguration('map_voxel')
    publish_after = LaunchConfiguration('publish_after')
    stable_count = LaunchConfiguration('stable_count')
    stable_trans = LaunchConfiguration('stable_trans')
    stable_rot = LaunchConfiguration('stable_rot')
    min_change_trans = LaunchConfiguration('min_change_trans')
    min_change_rot = LaunchConfiguration('min_change_rot')
    reinit_trans = LaunchConfiguration('reinit_trans_thre')
    reinit_rot = LaunchConfiguration('reinit_rot_thre')

    preprocess = LaunchConfiguration('preprocess')
    points_per_frame = LaunchConfiguration('points_per_frame')
    rate = LaunchConfiguration('rate')

    reloc_period = LaunchConfiguration('reloc_period')
    initial_pose = LaunchConfiguration('initial_pose')
    min_score = LaunchConfiguration('min_score')
    nav2_timeout = LaunchConfiguration('nav2_timeout')

    declare_args = [
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('field', default_value='legacy',
                              choices=['legacy', 'cn_v3_draft'],
                              description='切换场地时必须另传对应场景同次采集的 prior_map 与 map'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='', description='出生点 x；留空时按 field 选择'),
        DeclareLaunchArgument('y', default_value='', description='出生点 y；留空时按 field 选择'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # ★ 这两个文件必须来自**同一次** save_prior_map.sh 的产物（同一坐标系）：
        #   prior_map.pcd 是 LIO 的 odom 系点云，prior_costmap.yaml 是同一套点云
        #   投影出来的 2D 图。混用不同次建图的文件会出现"重定位结果对、但代价地图
        #   整体错位"。出生点也要和建图时一致（默认 -4.3, 1.5, yaw 0）。
        DeclareLaunchArgument(
            'prior_map',
            default_value=os.path.join(pkg_share, 'maps', 'prior_map.pcd'),
            description='先验 3D 点云地图（重定位用；由 save_prior_map.sh 产出）'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'prior_costmap.yaml'),
            description='先验 2D 代价地图（Nav2 map_server 用；同一次建图的产物）'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_omni_params.yaml')),
        DeclareLaunchArgument('livo_config', default_value=SHARED_LIVO_CONFIG,
                              description='robot/simulation 共用参数（config/livo_mid360.yaml）'),
        DeclareLaunchArgument('camera_config', default_value=SHARED_CAMERA_CONFIG),

        DeclareLaunchArgument(
            'reloc_impl', default_value='teaser',
            description='teaser = TEASER++/VGICP + reloc_bridge（与真机同构，默认）；'
                        'voxel = 旧 4-DOF 体素重叠定位器（仅 A/B 对比用）'),
        DeclareLaunchArgument('teaser_inlier', default_value='10',
                              description='TEASER 内点阈值。仿真场地小、2D 障碍少，'
                                          'FPFH 对应点只有 20~70 个，阈值 30 永远不收敛'
                                          '（实测 11x11 m 的图：内点 3~12）。'
                                          '真机默认 30（real_nav.sh 里可按需降到 15）'),
        DeclareLaunchArgument('map_voxel', default_value='0.15',
                              description='重定位节点对先验图的降采样体素。仿真的稠密图'
                                          '（5 万点）用 0.3 会退化成几百点，对应点太少；'
                                          '真机用 0.2~0.3'),
        DeclareLaunchArgument('publish_after', default_value='3',
                              description='GICP 连续多少帧达标后开始每帧连续输出位姿'),
        DeclareLaunchArgument('stable_count', default_value='3',
                              description='reloc_bridge：连续多少帧一致才算稳定'),
        DeclareLaunchArgument('stable_trans', default_value='0.25'),
        DeclareLaunchArgument('stable_rot', default_value='8.0'),
        DeclareLaunchArgument('min_change_trans', default_value='0.5'),
        DeclareLaunchArgument('min_change_rot', default_value='10.0'),
        DeclareLaunchArgument('reinit_trans_thre', default_value='0.8',
                              description='稳定重定位与已生效位姿差超过它就重建体素地图 (m)'),
        DeclareLaunchArgument('reinit_rot_thre', default_value='15.0'),

        # 雷达抽稀/滤波：与真机同一个节点、同一组默认值
        DeclareLaunchArgument('preprocess', default_value='true'),
        DeclareLaunchArgument('points_per_frame', default_value='4000'),
        DeclareLaunchArgument('rate', default_value='15.0',
                              description='抽稀输出帧率 Hz（与真机 15 Hz 一致）'),

        # 旧 4-DOF 定位器参数（reloc_impl:=voxel 时才有意义）
        DeclareLaunchArgument('reloc_period', default_value='6.0',
                              description='[voxel] 重定位周期（秒）'),
        DeclareLaunchArgument('initial_pose', default_value='0,0,0',
                              description='[voxel] 初始位姿 "x,y,yaw_度"；想强制全图搜索给 auto'),
        DeclareLaunchArgument('min_score', default_value='0.55',
                              description='[voxel] 体素重叠度阈值'),
        DeclareLaunchArgument('nav2_timeout', default_value='75.0',
                              description='Nav2 bringup 单轮等待激活的最长时间（秒），'
                                          '超时由 nav2_supervisor.py 整轮重启'),
    ]

    # ---------------- 1) 仿真：odom->base_link 交给 LIO ----------------
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': 'false', 'gui': gui, 'field': field, 'world': world,
                          'publish_odom_tf': 'false',
                          'odom_topic': 'odom_gz',
                          'use_sim_time': use_sim_time,
                          'x': x, 'y': y, 'z': z, 'yaw': yaw}.items(),
    )

    # ---------------- 2) 雷达抽稀/滤波（与真机同一节点） ----------------
    pre = ExecuteProcess(
        output='screen', condition=IfCondition(preprocess),
        cmd=['python3', PREPROCESS_SCRIPT,
             '--in-topic', '/livox/lidar',
             '--out-topic', '/livox/lidar_filtered',
             '--points-per-frame', points_per_frame,
             '--rate', rate],
    )

    # ---------------- 3) 6-DOF 全局重定位：直接吃原始 CustomMsg（与 odom 无关） ----------------
    teaser = Node(
        package='relocalization', executable='teaser_gicp_node', name='teaser_gicp_node',
        output='screen', condition=IfCondition(use_teaser),
        parameters=[{'use_sim_time': use_sim_time,
                     'map_path': prior_map,
                     'map_frame_id': 'map',
                     'pcl_type': 'livox',
                     'shutdown_on_success': False,
                     'map_voxel_leaf_size': map_voxel,
                     'cloud_voxel_leaf_size': map_voxel,
                     'teaser_inlier_threshold': teaser_inlier,
                     'publish_after_converged': publish_after}],
    )

    # map←livox -> map←base + 稳定性过滤 + map->odom（与真机同一个脚本）
    bridge = ExecuteProcess(
        output='screen', condition=IfCondition(use_teaser),
        cmd=['python3', BRIDGE_SCRIPT,
             '--map-frame', 'map',
             '--odom-frame', 'odom',
             '--base-frame', 'base_link',
             '--lidar-frame', 'livox_frame',
             '--odom-topic', '/aft_mapped_to_init',
             # 仿真外参：LiDAR 在 base_link 正上方 0.36 m，无旋转
             '--ext-x', '0.0', '--ext-y', '0.0', '--ext-z', str(SIM_BRIDGE_EXT_Z),
             '--stable-count', stable_count,
             '--stable-trans', stable_trans,
             '--stable-rot', stable_rot,
             '--min-change-trans', min_change_trans,
             '--min-change-rot', min_change_rot,
             # 仿真时间：TF 与 dt 判据都走 /clock，和 Nav2/LIO 保持同一时间基准
             '--ros-args', '-p', 'use_sim_time:=true'],
    )

    # ---------------- 4) FAST-LIVO2：先验图跟踪模式（等 /relocalization_result 播种） ----------------
    livo = Node(
        package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
        output='screen',
        parameters=[livo_config, camera_config,
                    {'use_sim_time': use_sim_time},
                    {'extrin_calib': {'extrinsic_T': SIM_LIO_EXTRINSIC_T}},
                    # 共用 yaml 按"真机倒装"写：仿真 IMU 没翻、话题不同，覆盖回来
                    {'extrin_calib': {'extrinsic_R': SIM_EXTRINSIC_R}},
                    {'common': {'imu_topic': '/livox/imu'}},
                    {'locate_in_prior_map': True},
                    {'prior_map_path': prior_map},
                    {'allow_map_reinit': True},
                    {'map_reinit_trans_thre': reinit_trans},
                    {'map_reinit_rot_thre': reinit_rot}],
    )

    # ---------------- 4b) 旧 4-DOF 定位器（reloc_impl:=voxel，A/B 对比用） ----------------
    loc_base = ['--prior-map', prior_map,
                '--cloud-topic', '/cloud_registered',
                '--odom-topic', '/aft_mapped_to_init',
                '--period', reloc_period,
                '--min-score', min_score,
                '--window', '40', '--window-sec', '6.0',
                '--acc-voxel', '0.03',
                '--meta', costmap_meta]
    use_seed = PythonExpression(["'", initial_pose, "' != 'auto'"])
    use_auto = PythonExpression(["'", initial_pose, "' == 'auto'"])
    localizer = Node(
        package='rc_field_nav', executable='prior_map_localizer.py',
        name='prior_map_localizer', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", reloc_impl, "' == 'voxel' and '", initial_pose, "' != 'auto'"])),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=loc_base + ['--initial-pose', initial_pose],
    )
    localizer_auto = Node(
        package='rc_field_nav', executable='prior_map_localizer.py',
        name='prior_map_localizer', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", reloc_impl, "' == 'voxel' and '", initial_pose, "' == 'auto'"])),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=loc_base,
    )

    # ---------------- 5) Nav2 全套（map_server + 规划/控制/BT/行为） ----------------
    #   ★ 用 nav2_supervisor.py 起，而不是直接 IncludeLaunchDescription：
    #   本机只有 Fast DDS，实测在"仿真+LIO+定位器"同时跑的时候，
    #   lifecycle_manager 连续切换 7 个节点的生命周期时偶发某次 change_state 的
    #   **响应发不出去**（服务端写超时），客户端永远等不到响应 ->
    #   lifecycle_manager 停在 "Configuring velocity_smoother"，
    #   bt_navigator 永远是 inactive（同一套配置 4 次里成功 2 次）。
    #   守护进程会轮询 bt_navigator 状态，超时没激活就把整轮 bringup 杀掉重启。
    #   注意：守护命令是一条**纯字面量**，地图/参数通过环境变量传进去。
    navigation = ExecuteProcess(
        output='screen',
        cmd=['ros2', 'run', 'rc_field_nav', 'nav2_supervisor.py',
             '--node', 'bt_navigator',
             '--timeout', nav2_timeout,
             '--retries', '4',
             '--watch-timeout', '30.0',
             '--',
             'ros2', 'launch', 'rc_field_nav', 'nav2_only.launch.py'],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(rviz_self),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'field_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    for a in declare_args:
        ld.add_action(a)
    # 在 sim 包进来之前先把用户给的 rviz 存起来
    ld.add_action(SetLaunchConfiguration('rviz_self', rviz))
    ld.add_action(sim)
    ld.add_action(TimerAction(period=8.0, actions=[pre]))
    # 重定位先起（LIO 在等它的结果），但必须等雷达出数据
    ld.add_action(TimerAction(period=12.0, actions=[teaser, bridge]))
    ld.add_action(TimerAction(period=14.0, actions=[livo, localizer, localizer_auto]))
    # 先验 2D 地图路径 / Nav2 参数文件都通过**环境变量**传给被守护的
    # nav2_only.launch.py —— 守护进程的命令必须是纯字面量，不能在里面拼
    # "map:=<路径>" / "params_file:=<路径>"（ExecuteProcess 会把 name:=value 拆成两个 argv）
    ld.add_action(SetEnvironmentVariable('RC_MAP_YAML', costmap_yaml))
    ld.add_action(SetEnvironmentVariable('RC_NAV2_PARAMS', params_file))
    # Nav2 必须等 map->odom 存在（否则代价地图一直报 "frame does not exist"）
    ld.add_action(TimerAction(period=50.0, actions=[navigation]))
    ld.add_action(rviz_node)
    return ld
