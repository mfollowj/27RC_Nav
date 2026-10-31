#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同一个 Gazebo 里放两辆 omni4 小车，各用一套 RViz + 键盘遥控。

    ros2 launch rc_field_nav sim_two_robots.launch.py            # 默认开 Gazebo 窗口
    gui:=false rviz:=true                                       # 无 Gazebo 界面
    field:=cn_v3_draft                                          # V3 场地

两辆车各自一个 ROS 命名空间，互不干扰：

    car1: /car1/cmd_vel  /car1/odom  /car1/joint_states  TF car1/odom -> car1/base_link
    car2: /car2/cmd_vel  /car2/odom  /car2/joint_states  TF car2/odom -> car2/base_link

遥控（另开终端，--topic 指到对应车）：

    ./sim_teleop.sh --topic /car1/cmd_vel
    ./sim_teleop.sh --topic /car2/cmd_vel

RViz：本 launch 直接开两个窗口，Fixed Frame 分别是 car1/odom、car2/odom。

实现要点（踩过的坑都写在这里）：
  * 一个 gzserver 里两辆车 -> 两套插件都跑在同一个 ROS 域里，所以话题必须带命名空间，
    否则 /cmd_vel、/joint_states 会互相抢；用 urdf 里的 <ros><namespace> 实现。
  * ROS 侧的 TF 用 robot_state_publisher 的 frame_prefix 加 carN/ 前缀，
    两棵 TF 树因此不会同名冲突。
  * ros2_control 每辆车一个 controller_manager（/carN/controller_manager），
    控制器参数文件按命名空间生成在 /tmp 下。
  * 雷达点云的 frame_id 是 urdf 里的 link 名（livox_frame），本轮没有给 link 加前缀，
    所以单个 RViz 里先只画车体/里程计，不画点云（要画点云需要给 link 加前缀，另做）。
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


REPO_ROOT = _find_repo_root()
DEFAULT_FIELD_DIR = os.environ.get(
    'RC_FIELD_DIR', os.path.join(REPO_ROOT, 'simulation', 'worlds'))


def _controllers_yaml(ns: str) -> str:
    """按命名空间生成控制器参数文件（controller_manager 的节点名是 /<ns>/controller_manager）。"""
    cfg = {
        f'/{ns}/controller_manager': {
            'ros__parameters': {
                'use_sim_time': True,
                'update_rate': 100,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster'},
                'wheel_velocity_controller': {
                    'type': 'velocity_controllers/JointGroupVelocityController'},
            }
        },
        f'/{ns}/wheel_velocity_controller': {
            'ros__parameters': {
                'joints': ['base_to_wheel1', 'base_to_wheel2',
                           'base_to_wheel3', 'base_to_wheel4'],
                'interface_name': 'velocity',
            }
        },
    }
    path = os.path.join(tempfile.gettempdir(), f'omni4_controllers_{ns}.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
    return path


def _world_for(field_pkg_share: str, field: str) -> str:
    if field == 'cn_v3_draft':
        return os.path.join(field_pkg_share, 'worlds', 'cn_v3_draft', 'worlds',
                            'robocon_cn_v3_draft.world')
    return os.path.join(DEFAULT_FIELD_DIR, 'robocon_track.world')


def _spawn_pose(field: str, x, y, yaw):
    if x == '' or y == '':
        if field == 'cn_v3_draft':
            x = x if x != '' else '-5.15'
            y = y if y != '' else '-5.15'
        else:
            x = x if x != '' else '-4.3'
            y = y if y != '' else '1.5'
    return x, y, (yaw if yaw != '' else '0.0')


def generate_launch_description():
    pkg_share = get_package_share_directory('rc_field_nav')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    lidar_sensor_xacro = os.path.join(pkg_share, 'urdf', 'lidar_livox_mid360.xacro')

    args = [
        DeclareLaunchArgument('field', default_value='legacy',
                              choices=['legacy', 'cn_v3_draft']),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('clean_stale', default_value='true'),
        DeclareLaunchArgument('robots', default_value='car1,car2',
                              description='要生成的车（调试用，例如 car1）'),
        DeclareLaunchArgument('lidar_samples', default_value='20000'),
        DeclareLaunchArgument('lidar_downsample', default_value='4'),
        # 两辆车的出生点
        DeclareLaunchArgument('x1', default_value=''), DeclareLaunchArgument('y1', default_value=''),
        DeclareLaunchArgument('yaw1', default_value='0.0'),
        DeclareLaunchArgument('x2', default_value=''), DeclareLaunchArgument('y2', default_value='2.0'),
        DeclareLaunchArgument('yaw2', default_value='0.0'),
    ]

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    field = LaunchConfiguration('field')
    world = LaunchConfiguration('world')
    lidar_samples = LaunchConfiguration('lidar_samples')
    lidar_downsample = LaunchConfiguration('lidar_downsample')

    # ---------------- Gazebo（只起一个实例）----------------
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join([DEFAULT_FIELD_DIR,
                              os.path.join(pkg_share, 'worlds', 'cn_v3_draft', 'models')]))
    gazebo_master = SetEnvironmentVariable(
        name='GAZEBO_MASTER_URI',
        value=os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11355'))
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen',
                              condition=IfCondition(gui))

    actions = list(args) + [gazebo_model_path, gazebo_master]
    actions.append(OpaqueFunction(function=_launch_all,
                                  args=[pkg_share, pkg_gazebo_ros,
                                        lidar_sensor_xacro, world, field,
                                        gui, rviz, use_sim_time,
                                        lidar_samples, lidar_downsample]))
    return LaunchDescription(actions)


def _launch_all(context, pkg_share, pkg_gazebo_ros, lidar_sensor_xacro,
                world_cfg, field_cfg, gui, rviz, use_sim_time,
                lidar_samples, lidar_downsample):
    """在运行时确定 field/world 与出生点，然后生成两辆车的全部节点。"""
    lc = {k: v for k, v in context.launch_configurations.items()}
    field = lc.get('field', 'legacy')
    world = lc.get('world') or _world_for(pkg_share, field)
    gui_v = lc.get('gui', 'true').lower() in ('1', 'true', 'yes')
    rviz_v = lc.get('rviz', 'true').lower() in ('1', 'true', 'yes')
    use_sim = str(lc.get('use_sim_time', 'true')).lower() in ('1', 'true', 'yes')

    all_robots = [
        ('car1', *_spawn_pose(field, lc.get('x1', ''), lc.get('y1', ''), lc.get('yaw1', ''))),
        ('car2', *_spawn_pose(field, lc.get('x2', ''), lc.get('y2', ''), lc.get('yaw2', ''))),
    ]
    wanted = [w.strip() for w in lc.get('robots', 'car1,car2').split(',') if w.strip()]
    robots = [r for r in all_robots if r[0] in wanted]
    robot_actions = []
    for ns, x, y, yaw in robots:
        desc = ParameterValue(
            Command(['xacro ', os.path.join(pkg_share, 'urdf', 'omni4_field.urdf.xacro'),
                     ' lidar_sensor_xacro:=', lidar_sensor_xacro,
                     ' publish_odom_tf:=true',
                     ' kinematic:=true',
                     ' odom_topic:=', f'{ns}/odom',
                     ' lidar_topic:=', f'/{ns}/livox/lidar',
                     ' imu_topic:=', f'/{ns}/livox/imu',
                     ' ns:=', ns,
                     ' lidar_samples:=20000',
                     ' lidar_downsample:=4',
                     ' controllers_yaml:=', '']),
            value_type=str)

        republish = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[('cloud_in', f'/{ns}/livox/lidar/pointcloud'), ('scan', 'scan')],
            parameters=[os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml'),
                        {'use_sim_time': use_sim}],
        )

        robot_actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen', namespace=ns,
                 parameters=[{'use_sim_time': use_sim,
                              'frame_prefix': f'{ns}/',
                              'robot_description': desc}]),
            TimerAction(period=4.0 + 6.0 * robots.index((ns, x, y, yaw)), actions=[Node(
                package='gazebo_ros', executable='spawn_entity.py',
                name='spawn_entity', output='screen', namespace=ns,
                arguments=['-entity', ns, '-topic', 'robot_description',
                           '-x', x, '-y', y, '-z', '0.05', '-Y', yaw,
                           '-timeout', '120'])]),
            republish,
        ])


    rviz_nodes = []
    if rviz_v:
        for ns, *_ in robots:
            cfg = os.path.join(pkg_share, 'rviz', f'two_{ns}.rviz')
            rviz_nodes.append(Node(package='rviz2', executable='rviz2',
                                   name=f'rviz2_{ns}', output='screen',
                                   namespace=ns,
                                   arguments=['-d', cfg],
                                   parameters=[{'use_sim_time': use_sim}]))

    gz = []
    gz.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items()))
    if gui_v:
        gz.append(ExecuteProcess(cmd=['gzclient'], output='screen'))

    return gz + robot_actions + rviz_nodes
