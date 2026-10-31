#!/usr/bin/env python3
"""Generate independent, explicitly provisional TR and BR ROS/Gazebo description packages."""
from pathlib import Path
import math
import os

ROOT = Path(os.environ.get('ROBOCON_SRC_ROOT', Path(__file__).resolve().parent/'legacy_generated'))


def inertial(mass, ixx, iyy, izz, origin='0 0 0'):
    return f'''<inertial><origin xyz="{origin}"/><mass value="{mass}"/><inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/></inertial>'''


def box_link(name, size, mass, color='0.72 0.72 0.76 1', extra_visual='', extra_collision='', inertial_origin='0 0 0'):
    sx, sy, sz = size
    return f'''<link name="{name}">
  {inertial(mass, max(mass*(sy*sy+sz*sz)/12,1e-6), max(mass*(sx*sx+sz*sz)/12,1e-6), max(mass*(sx*sx+sy*sy)/12,1e-6), inertial_origin)}
  <visual>{extra_visual}<geometry><box size="{sx} {sy} {sz}"/></geometry><material name="{name}_mat"><color rgba="{color}"/></material></visual>
  <collision>{extra_collision}<geometry><box size="{sx} {sy} {sz}"/></geometry></collision>
</link>'''


def cyl_link(name, radius, length, mass, color='0.12 0.12 0.14 1', cylinder_axis_y=False):
    rpy = '1.57079632679 0 0' if cylinder_axis_y else '0 0 0'
    i_axis = mass * radius * radius / 2
    i_cross = mass * (3*radius*radius + length*length) / 12
    return f'''<link name="{name}">
  {inertial(mass, i_cross, i_axis if cylinder_axis_y else i_cross, i_cross if cylinder_axis_y else i_axis)}
  <visual><origin xyz="0 0 0" rpy="{rpy}"/><geometry><cylinder radius="{radius}" length="{length}"/></geometry><material name="{name}_mat"><color rgba="{color}"/></material></visual>
  <collision><origin xyz="0 0 0" rpy="{rpy}"/><geometry><cylinder radius="{radius}" length="{length}"/></geometry></collision>
</link>'''


def standard_wheel(name, x, y):
    return '\n'.join([
        cyl_link(name, .12, .05, .65, cylinder_axis_y=True),
        f'<joint name="{name}_drive_joint" type="continuous"><parent link="base_link"/><child link="{name}"/><origin xyz="{x} {y} -0.10"/><axis xyz="0 1 0"/><limit effort="18" velocity="15"/><dynamics damping="0.04" friction="0.02"/></joint>',
        f'<gazebo reference="{name}"><mu1>0.8</mu1><mu2>0.8</mu2><kp>100000</kp><kd>10</kd><maxVel>0.5</maxVel><minDepth>0.001</minDepth></gazebo>'
    ])


def mecanum_wheel(name, x, y, handed):
    parts = [cyl_link(name, .082, .07, .34, cylinder_axis_y=True),
             f'<joint name="{name}_drive_joint" type="continuous"><parent link="base_link"/><child link="{name}"/><origin xyz="{x} {y} -0.10"/><axis xyz="0 1 0"/><limit effort="18" velocity="15"/><dynamics damping="0.04" friction="0.02"/></joint>',
             f'<gazebo reference="{name}"><mu1>0.5</mu1><mu2>0.5</mu2><kp>100000</kp><kd>8</kd><maxVel>0.5</maxVel><minDepth>0.001</minDepth></gazebo>']
    for i in range(12):
        theta = 2*math.pi*i/12
        rx = .094*math.cos(theta)
        rz = .094*math.sin(theta)
        vx = -math.sin(theta)/math.sqrt(2)
        vy = handed/math.sqrt(2)
        vz = math.cos(theta)/math.sqrt(2)
        yaw = math.atan2(vy, vx)
        pitch = math.acos(max(-1.0,min(1.0,vz)))
        roller = f'{name}_roller_{i:02d}'
        parts += [
            cyl_link(roller, .026, .112, .025, '0.16 0.16 0.18 1'),
            f'<joint name="{roller}_spin_joint" type="continuous"><parent link="{name}"/><child link="{roller}"/><origin xyz="{rx:.6f} 0 {rz:.6f}" rpy="0 {pitch:.6f} {yaw:.6f}"/><axis xyz="0 0 1"/><limit effort="1.5" velocity="20"/><dynamics damping="0.015" friction="0.01"/></joint>',
            f'<gazebo reference="{roller}"><mu1>0.8</mu1><mu2>0.8</mu2><kp>100000</kp><kd>8</kd><maxVel>0.5</maxVel><minDepth>0.001</minDepth></gazebo>'
        ]
    return '\n'.join(parts)


def sensor_block(ns):
    return f'''<link name="lidar_link"><inertial><mass value="0.18"/><inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/></inertial><visual><geometry><cylinder radius="0.045" length="0.06"/></geometry><material name="lidar_grey"><color rgba="0.2 0.2 0.22 1"/></material></visual><collision><geometry><cylinder radius="0.045" length="0.06"/></geometry></collision></link>
<joint name="lidar_mount_joint" type="fixed"><parent link="base_link"/><child link="lidar_link"/><origin xyz="0 0 0.15"/></joint>
<link name="imu_link"><inertial><mass value="0.03"/><inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/></inertial></link>
<joint name="imu_mount_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.12"/></joint>
<link name="front_camera_link"><inertial><mass value="0.04"/><inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/></inertial></link>
<joint name="front_camera_mount_joint" type="fixed"><parent link="base_link"/><child link="front_camera_link"/><origin xyz="0.20 0 0.14" rpy="0 0 0"/></joint>
<gazebo reference="lidar_link"><sensor name="{ns}_mid360_lidar" type="gpu_lidar"><topic>/{ns}/livox/lidar</topic><update_rate>10</update_rate><always_on>true</always_on><visualize>false</visualize><ray><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal><vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.12217</min_angle><max_angle>0.90757</max_angle></vertical></scan><range><min>0.10</min><max>40.0</max><resolution>0.01</resolution></range></ray></sensor></gazebo>
<gazebo reference="imu_link"><sensor name="{ns}_imu" type="imu"><topic>/{ns}/imu</topic><update_rate>200</update_rate><always_on>true</always_on><visualize>false</visualize></sensor></gazebo>
<gazebo reference="front_camera_link"><sensor name="{ns}_color_camera" type="camera"><topic>/{ns}/camera/color/image</topic><update_rate>30</update_rate><always_on>true</always_on><visualize>false</visualize><camera><horizontal_fov>1.0472</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.05</near><far>12.0</far></clip></camera></sensor></gazebo>'''


def arm(ns, with_flipper=False):
    parts=[
        box_link('lift_carriage',(.08,.08,.30),1.0,'0.78 0.64 0.16 1'),
        '<joint name="lift_joint" type="prismatic"><parent link="base_link"/><child link="lift_carriage"/><origin xyz="0 0 0.05"/><axis xyz="0 0 1"/><limit lower="-0.08" upper="0.70" effort="120" velocity="0.30"/><dynamics damping="25" friction="2"/></joint>',
        box_link('reach_stage',(.30,.08,.06),.65,'0.78 0.64 0.16 1','<origin xyz="0.15 0 0"/>','<origin xyz="0.15 0 0"/>','0.15 0 0'),
        '<joint name="reach_joint" type="prismatic"><parent link="lift_carriage"/><child link="reach_stage"/><origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.30" effort="80" velocity="0.25"/><dynamics damping="20" friction="1"/></joint>',
        box_link('gripper_base',(.10,.12,.035),.35,'0.30 0.32 0.34 1'),
        '<joint name="gripper_mount_joint" type="fixed"><parent link="reach_stage"/><child link="gripper_base"/><origin xyz="0.30 0 -0.10"/></joint>',
        box_link('jaw_left_link',(.06,.025,.20),.10,'0.85 0.82 0.72 1'),
        '<joint name="jaw_left_joint" type="prismatic"><parent link="gripper_base"/><child link="jaw_left_link"/><origin xyz="0 0.05 0"/><axis xyz="0 1 0"/><limit lower="0" upper="0.17" effort="45" velocity="0.15"/><dynamics damping="8" friction="1"/></joint>',
        box_link('jaw_right_link',(.06,.025,.20),.10,'0.85 0.82 0.72 1'),
        '<joint name="jaw_right_joint" type="prismatic"><parent link="gripper_base"/><child link="jaw_right_link"/><origin xyz="0 -0.05 0"/><axis xyz="0 -1 0"/><limit lower="0" upper="0.17" effort="45" velocity="0.15"/><dynamics damping="8" friction="1"/></joint>',
        '<gazebo reference="jaw_left_link"><mu1>0.9</mu1><mu2>0.9</mu2><kp>50000</kp><kd>5</kd></gazebo>',
        '<gazebo reference="jaw_right_link"><mu1>0.9</mu1><mu2>0.9</mu2><kp>50000</kp><kd>5</kd></gazebo>',
        '<link name="wrist_camera_link"><inertial><mass value="0.03"/><inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/></inertial></link>',
        '<joint name="wrist_camera_mount_joint" type="fixed"><parent link="gripper_base"/><child link="wrist_camera_link"/><origin xyz="0.08 0 0.02"/></joint>'
    ]
    if with_flipper:
        parts += [
            box_link('cap_cradle',(.22,.22,.025),.28,'0.25 0.28 0.31 1'),
            '<joint name="cap_flip_joint" type="revolute"><parent link="gripper_base"/><child link="cap_cradle"/><origin xyz="0 0 -0.025"/><axis xyz="1 0 0"/><limit lower="-1.5707963" upper="1.5707963" effort="25" velocity="0.8"/><dynamics damping="2" friction="0.5"/></joint>',
            '<gazebo reference="cap_cradle"><mu1>0.9</mu1><mu2>0.9</mu2><kp>50000</kp><kd>5</kd></gazebo>'
        ]
    return '\n'.join(parts)


def controller_plugins(joints):
    return '\n'.join(f'''<plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController"><joint_name>{j}</joint_name><p_gain>80</p_gain></plugin>''' for j in joints)


def robot_urdf(is_br=False):
    name='robocon_br_proto' if is_br else 'robocon_tr_proto'
    ns='br' if is_br else 'tr'
    wheels=[]
    wheel_positions=[('front_left',.20,.22),('front_right',.20,-.22),('rear_left',-.20,.22),('rear_right',-.20,-.22)]
    for w,x,y in wheel_positions:
        if is_br:
            handed=1 if w in ('front_left','rear_right') else -1
            wheels.append(mecanum_wheel(w+'_wheel',x,y,handed))
        else:
            wheels.append(standard_wheel(w+'_wheel',x,y))
    drive_topic=f'/model/{name}/cmd_vel'
    drive_plugin=f'''<plugin filename="ignition-gazebo-mecanum-drive-system" name="ignition::gazebo::systems::MecanumDrive"><topic>{drive_topic}</topic><front_left_joint>front_left_wheel_drive_joint</front_left_joint><front_right_joint>front_right_wheel_drive_joint</front_right_joint><back_left_joint>rear_left_wheel_drive_joint</back_left_joint><back_right_joint>rear_right_wheel_drive_joint</back_right_joint><wheel_separation>0.44</wheel_separation><wheelbase>0.40</wheelbase><wheel_radius>0.12</wheel_radius><min_acceleration>-0.5</min_acceleration><max_acceleration>0.5</max_acceleration></plugin>''' if is_br else f'''<plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive"><topic>{drive_topic}</topic><left_joint>front_left_wheel_drive_joint</left_joint><left_joint>rear_left_wheel_drive_joint</left_joint><right_joint>front_right_wheel_drive_joint</right_joint><right_joint>rear_right_wheel_drive_joint</right_joint><wheel_separation>0.44</wheel_separation><wheel_radius>0.12</wheel_radius><max_linear_velocity>0.35</max_linear_velocity><min_linear_velocity>-0.35</min_linear_velocity><max_angular_velocity>1.0</max_angular_velocity><min_angular_velocity>-1.0</min_angular_velocity></plugin>'''
    armj=['lift_joint','reach_joint','jaw_left_joint','jaw_right_joint']+(['cap_flip_joint'] if is_br else [])
    return f'''<?xml version="1.0"?>
<robot name="{name}">
<link name="base_footprint"/>
<link name="base_link">{inertial(16,.40,.40,.58)}<visual><geometry><box size="0.54 0.50 0.12"/></geometry><material name="chassis"><color rgba="0.75 0.59 0.20 1"/></material></visual><collision><geometry><box size="0.54 0.50 0.12"/></geometry></collision></link>
<joint name="base_mount_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.22"/></joint>
{''.join(wheels)}
{arm(ns,is_br)}
{sensor_block(ns)}
<gazebo>{drive_plugin}{controller_plugins(armj)}</gazebo>
</robot>'''


def package_xml(name, description):
    return f'''<?xml version="1.0"?>
<package format="3"><name>{name}</name><version>0.1.0</version><description>{description}</description><maintainer email="robotics@example.invalid">RoboCon prototype</maintainer><license>Apache-2.0</license><buildtool_depend>ament_cmake</buildtool_depend><exec_depend>ros_gz_sim</exec_depend><exec_depend>ament_index_python</exec_depend><exec_depend>launch</exec_depend><exec_depend>launch_ros</exec_depend><exec_depend>robot_state_publisher</exec_depend><exec_depend>joint_state_publisher_gui</exec_depend><exec_depend>rviz2</exec_depend><export><build_type>ament_cmake</build_type></export></package>'''


def cmake(name, dirs):
    return f'''cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY {dirs} DESTINATION share/${{PROJECT_NAME}})
ament_package()
'''


def common_config(is_br):
    ns='br' if is_br else 'tr'
    layout='mecanum_x_layout_12_rollers_per_wheel' if is_br else 'four_ordinary_wheels_differential'
    model_name='robocon_br_proto' if is_br else 'robocon_tr_proto'
    roller_cfg='rollers_per_wheel: 12\n  roller_angle_deg: 45\n  ' if is_br else ''
    flip_joint_cfg=', cap_flip_joint: 0.0' if is_br else ''
    cap_flipper_enabled=str(is_br).lower()
    carried=2 if is_br else 3
    return f'''prototype_parameters: true
robot: {ns}
geometry:
  base_footprint_to_base_link_z_m: 0.22
  chassis_length_m: 0.54
  chassis_width_m: 0.50
  chassis_height_m: 0.12
  wheel_layout: {layout}
  wheel_radius_m: 0.12
  wheelbase_m: 0.40
  track_m: 0.44
  {roller_cfg}arm:
  lift_joint_origin_in_base_link_z_m: 0.05
  gripper_contact_center_from_carriage_z_m: -0.10
  lift_joint_min_m: -0.08
  lift_joint_max_m: 0.70
  reach_min_m: 0.0
  reach_max_m: 0.30
  gripper_closed_gap_m: 0.075
  gripper_open_gap_m: 0.415
  gripper_open_each_finger_travel_m: 0.17
  stowed_joint_positions: {{lift_joint: 0.0, reach_joint: 0.0, jaw_left_joint: 0.0, jaw_right_joint: 0.0{flip_joint_cfg}}}
competition:
  block_edge_m: 0.35
  block_mass_range_kg: [0.20, 0.35]
  cap_edge_m: 0.20
  cap_mass_range_kg: [0.08, 0.15]
  carried_competition_objects_max: {carried}
  l2_surface_height_world_m: 0.90
  l2_plan_size_m: [3.0, 3.0]
  tower_build_square_m: [0.5, 0.5]
  l2_full_tower_blocks_before_cap: 2
  l2_cap_target_center_above_platform_m: 0.80
  whole_robot_projection_inside_l2_required_for_l2_cap: true
  l2_projection_check: "use the union of all collision-link polygons and carried-object polygons; rotate by robot yaw and require the complete projection to fit in [-1.5, 1.5] m in both platform axes"
reach_budget:
  tool_home_center_above_base_footprint_m: 0.17
  l2_cap_center_above_platform_m: 0.80
  clearance_margin_m: 0.05
  computed_lift_stroke_min_m: 0.68
  prototype_lift_stroke_m: 0.70
  formula: "target_center_above_platform - (base_footprint_to_base_link + lift_joint_origin + tool_center_offset) + margin"
contact:
  status: PROTOTYPE_UNCALIBRATED
  wheel_mu1_mu2: [0.8, 0.8]
  roller_mu1_mu2: [0.8, 0.8]
  gripper_mu1_mu2: [0.9, 0.9]
  contact_stiffness_damping: "placeholder only; calibrate against selected Gazebo physics and measured materials"
sensors:
  lidar: {{frame: lidar_link, base_link_xyz_m: [0.0, 0.0, 0.15], rate_hz: 10, topic: /{ns}/livox/lidar}}
  imu: {{frame: imu_link, base_link_xyz_m: [0.0, 0.0, 0.12], rate_hz: 200, topic: /{ns}/imu, acceleration_unit: m/s^2, angular_velocity_unit: rad/s}}
  front_camera: {{frame: front_camera_link, base_link_xyz_m: [0.20, 0.0, 0.14], rate_hz: 30, topic: /{ns}/camera/color/image}}
  wrist_camera: {{frame: wrist_camera_link, parent: gripper_base, pose: TBD}}
  prototype_T_lidar_imu_translation_m: [0.0, 0.0, 0.03]
  prototype_T_lidar_imu_rotation_rpy_rad: [0.0, 0.0, 0.0]
rule_envelope:
  start_cube_m: 0.70
  deployed_max_xyz_m: [1.40, 1.00, 1.20]
  start_pose: stowed_joint_positions
  region_rule: "At L2 cap placement, project every robot collision link and any carried object onto XY; every point must remain within the 3.0 m x 3.0 m L2 boundary. Checking only base_link center is insufficient."
  tr_region: "TR stays in ground region and transfer zone; no part crosses into L1 or L2."
  br_region: "BR enters L1 only through the ramp/stairs; any part crossing a boundary violates the area limit."
cap_flipper:
  enabled: {cap_flipper_enabled}
  joint: cap_flip_joint
  axis: [1.0, 0.0, 0.0]
  angle_range_rad: [-1.5707963, 1.5707963]
  stowed_angle_rad: 0.0
  initial_cap_pose_source: "explicit scenario override; never rely on the SDF default pose"
actuator_topics:
  drive_command: /model/{model_name}/cmd_vel
  joint_position_pattern: /model/{model_name}/joint/<joint_name>/cmd_pos
'''


def spawn_launch(ns, name, pkg):
    return f'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('{pkg}'), 'urdf', '{name}.urdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='robocon_world'),
        DeclareLaunchArgument('entity', default_value='{name}'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('ign_partition', default_value='robocon_prototypes'),
        DeclareLaunchArgument('ros_domain_id', default_value='28'),
        SetEnvironmentVariable('IGN_PARTITION', LaunchConfiguration('ign_partition')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-world', LaunchConfiguration('world'), '-file', urdf,
            '-name', LaunchConfiguration('entity'), '-allow_renaming=false',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', LaunchConfiguration('z')]),
    ])
'''


def view_launch(pkg, filename):
    return f'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    root = get_package_share_directory('{pkg}')
    urdf = Path(root, 'urdf', '{filename}.urdf').read_text()
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', parameters=[{{'robot_description': urdf, 'use_sim_time': LaunchConfiguration('use_sim_time')}}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{{'robot_description': urdf, 'use_sim_time': LaunchConfiguration('use_sim_time')}}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(root, 'rviz', 'prototype.rviz')]),
    ])
'''


def write_package(name, robot_name, is_br):
    package=ROOT/name
    manifest=package/'package.xml'
    if package.exists() and manifest.exists():
        import xml.etree.ElementTree as ET
        if ET.parse(manifest).getroot().findtext('name') != name:
            raise RuntimeError(f'Refusing to overwrite package with a different manifest: {package}')
    elif package.exists():
        raise RuntimeError(f'Refusing to overwrite directory without a package manifest: {package}')
    (package/'urdf').mkdir(parents=True,exist_ok=True)
    (package/'config').mkdir(exist_ok=True)
    (package/'launch').mkdir(exist_ok=True)
    (package/'rviz').mkdir(exist_ok=True)
    (package/'urdf'/f'{robot_name}.urdf').write_text(robot_urdf(is_br))
    (package/'config'/'prototype_geometry.yaml').write_text(common_config(is_br))
    (package/'launch'/f'spawn_{"br" if is_br else "tr"}.launch.py').write_text(spawn_launch('br' if is_br else 'tr',robot_name,name))
    (package/'launch'/'view_prototype.launch.py').write_text(view_launch(name,robot_name))
    (package/'package.xml').write_text(package_xml(name,f'Independent provisional {"BR mecanum" if is_br else "TR ordinary-wheel"} description package.'))
    (package/'CMakeLists.txt').write_text(cmake(name,'urdf config launch rviz'))
    (package/'rviz'/'prototype.rviz').write_text('''Panels:\n- Class: rviz_common/Displays\n  Name: Displays\nVisualization Manager:\n  Class: ""\n  Displays:\n  - Class: rviz_default_plugins/Grid\n    Name: Grid\n    Plane: XY\n    Cell Size: 1\n    Plane Cell Count: 20\n    Value: true\n  - Class: rviz_default_plugins/RobotModel\n    Name: RobotModel\n    Description Topic:\n      Value: /robot_description\n    Value: true\n  Global Options:\n    Fixed Frame: base_footprint\n  Name: root\n  Enabled: true\n''')
    return package


def game_objects():
    name='robocon_game_objects'
    package=ROOT/name
    manifest=package/'package.xml'
    if package.exists() and manifest.exists():
        import xml.etree.ElementTree as ET
        if ET.parse(manifest).getroot().findtext('name') != name:
            raise RuntimeError(f'Refusing to overwrite package with a different manifest: {package}')
    elif package.exists():
        raise RuntimeError(f'Refusing to overwrite directory without a package manifest: {package}')
    models={'tower_block_350':(.35,.275,'0.78 0.78 0.82 1'),'tower_cap_200':(.20,.115,'0.85 0.82 0.75 1')}
    for model,(edge,mass,color) in models.items():
        md=package/'models'/model; md.mkdir(parents=True,exist_ok=True)
        inertia=mass*edge*edge/6
        cap_visual=''
        if model=='tower_cap_200':
            # Colored face panels are visual-only; the single full-cube collision remains authoritative.
            red = '<material><ambient>0.9 0.04 0.04 1</ambient><diffuse>0.9 0.04 0.04 1</diffuse></material>'
            blue = '<material><ambient>0.05 0.10 0.9 1</ambient><diffuse>0.05 0.10 0.9 1</diffuse></material>'
            cap_visual=''.join([
              f'<visual name="red_top"><pose>0 0 0.1005 0 0 0</pose><geometry><box><size>0.2 0.2 0.001</size></box></geometry>{red}</visual>',
              f'<visual name="blue_bottom"><pose>0 0 -0.1005 0 0 0</pose><geometry><box><size>0.2 0.2 0.001</size></box></geometry>{blue}</visual>',
              *[f'<visual name="side_x_{side}_{color_name}"><pose>{x:.4f} 0 {z:.3f} 0 0 0</pose><geometry><box><size>0.001 0.2 0.1</size></box></geometry>{material}</visual>' for side, x in [('pos',0.1005),('neg',-0.1005)] for color_name,z,material in [('red',-0.05,red),('blue',0.05,blue)]],
              *[f'<visual name="side_y_{side}_{color_name}"><pose>0 {y:.4f} {z:.3f} 0 0 0</pose><geometry><box><size>0.2 0.001 0.1</size></box></geometry>{material}</visual>' for side, y in [('pos',0.1005),('neg',-0.1005)] for color_name,z,material in [('red',-0.05,red),('blue',0.05,blue)]]
            ])
        sdf=f'''<?xml version="1.0"?>\n<sdf version="1.8"><model name="{model}"><static>false</static><link name="body"><inertial><mass>{mass}</mass><inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial><collision name="collision"><geometry><box><size>{edge} {edge} {edge}</size></box></geometry><surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision><visual name="body"><geometry><box><size>{edge} {edge} {edge}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>{cap_visual}</link></model></sdf>'''
        (md/'model.sdf').write_text(sdf)
        (md/'model.config').write_text(f'''<?xml version="1.0"?><model><name>{model}</name><version>0.1.0</version><sdf version="1.8">model.sdf</sdf><description>Provisional competition object geometry. Mass is an allowed-range midpoint and must be replaced by the weighed part.</description></model>''')
    (package/'config').mkdir(exist_ok=True); (package/'launch').mkdir(exist_ok=True)
    (package/'config'/'cap_initial_orientation_cases.yaml').write_text('''prototype_parameters: true
# The rules show three alternative initial layouts. Each spawned cap must receive an explicit pose override.
# These are orientation test presets, not a claim about which competition layout will be selected.
cases:
  case_1_roll_side_up: {roll_rad: 1.5707963, pitch_rad: 0.0, yaw_rad: 0.0}
  case_2_pitch_side_up: {roll_rad: 0.0, pitch_rad: 1.5707963, yaw_rad: 0.0}
  case_3_opposite_side_up: {roll_rad: -1.5707963, pitch_rad: 0.0, yaw_rad: 1.5707963}
required_spawn_inputs: [x, y, z, roll_rad, pitch_rad, yaw_rad]
''')
    (package/'launch'/'spawn_cap.launch.py').write_text('''from launch import LaunchDescription\nfrom launch.actions import DeclareLaunchArgument, SetEnvironmentVariable\nfrom launch.substitutions import LaunchConfiguration\nfrom launch_ros.actions import Node\nfrom ament_index_python.packages import get_package_share_directory\nimport os\n\ndef generate_launch_description():\n    model = os.path.join(get_package_share_directory("robocon_game_objects"), "models", "tower_cap_200", "model.sdf")\n    return LaunchDescription([\n        DeclareLaunchArgument("entity", default_value="tower_cap_200"),\n        DeclareLaunchArgument("world", default_value="robocon_world"),\n        DeclareLaunchArgument("x", default_value="0.0"), DeclareLaunchArgument("y", default_value="0.0"), DeclareLaunchArgument("z", default_value="0.10"),\n        DeclareLaunchArgument("roll", default_value="1.5707963"), DeclareLaunchArgument("pitch", default_value="0.0"), DeclareLaunchArgument("yaw", default_value="0.0"),\n        DeclareLaunchArgument("ign_partition", default_value="robocon_prototypes"),\n        SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("ign_partition")),\n        Node(package="ros_gz_sim", executable="create", output="screen", arguments=["-world", LaunchConfiguration("world"), "-file", model, "-name", LaunchConfiguration("entity"), "-allow_renaming=false", "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", LaunchConfiguration("z"), "-R", LaunchConfiguration("roll"), "-P", LaunchConfiguration("pitch"), "-Y", LaunchConfiguration("yaw")]),\n    ])\n''')
    (package/'package.xml').write_text(package_xml(name,'Competition block and cap objects with explicit orientation override launch support.'))
    (package/'CMakeLists.txt').write_text(cmake(name,'models config launch'))
    return package


def main():
    ROOT.mkdir(parents=True,exist_ok=True)
    p_tr=write_package('robocon_tr_description','robocon_tr_proto',False)
    p_br=write_package('robocon_br_description','robocon_br_proto',True)
    p_obj=game_objects()
    print('\n'.join(str(p) for p in (p_tr,p_br,p_obj)))

if __name__=='__main__': main()
