#!/usr/bin/env python3
"""Independent TR video-inspired mechanism. Dimensions are prototype choices."""
from pathlib import Path
import math
import os
import xml.etree.ElementTree as ET
import yaml
from sensor_visuals_v2_20261022 import stylize_sensors

ROOT=Path(os.environ.get('TR_VIDEO_V2_ROOT',Path(__file__).resolve().parent/'generated/robocon_tr_description'))
(ROOT/'urdf').mkdir(parents=True,exist_ok=True)
(ROOT/'config').mkdir(parents=True,exist_ok=True)
NAME='robocon_tr_video_v2';NS='tr_v2'
robot=ET.Element('robot',name=NAME)
parts=[]
colors={'steel':'.54 .57 .60 1','light':'.76 .79 .81 1','dark':'.12 .15 .18 1',
        'blue':'.12 .38 .55 1','rubber':'.07 .08 .09 1','pad':'.30 .38 .38 1','red':'.72 .10 .08 1'}
def sub(parent,tag,text=None,**attrs):
    e=ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
    if text is not None:e.text=str(text)
    return e
def xyz(v):return ' '.join(f'{a:.8g}' for a in v)
def link(name,shape,size,mass,color='steel',rpy=(0,0,0)):
    l=sub(robot,'link',name=name)
    i=sub(l,'inertial');sub(i,'mass',value=mass)
    if shape=='box':x,y,z=size;ix=mass*(y*y+z*z)/12;iy=mass*(x*x+z*z)/12;iz=mass*(x*x+y*y)/12
    else:
        radius,length=size;ia=mass*radius**2/2;ic=mass*(3*radius**2+length**2)/12
        ix,iy,iz=(ic,ia,ic) if abs(rpy[0])>1 else (ic,ic,ia)
    sub(i,'inertia',ixx=max(ix,1e-7),iyy=max(iy,1e-7),izz=max(iz,1e-7),ixy=0,ixz=0,iyz=0)
    for typ in ('visual','collision'):
        e=sub(l,typ);sub(e,'origin',xyz='0 0 0',rpy=xyz(rpy));g=sub(e,'geometry')
        if shape=='box':sub(g,'box',size=xyz(size))
        else:sub(g,'cylinder',radius=size[0],length=size[1])
        if typ=='visual':mat=sub(e,'material',name=color);sub(mat,'color',rgba=colors[color])
    return l
def joint(name,parent,child,pos=(0,0,0),kind='fixed',axis=(0,0,1),limits=None):
    e=sub(robot,'joint',name=name,type=kind);sub(e,'parent',link=parent);sub(e,'child',link=child);sub(e,'origin',xyz=xyz(pos))
    if kind!='fixed':
        sub(e,'axis',xyz=xyz(axis));sub(e,'limit',**limits);sub(e,'dynamics',damping=.2 if kind=='continuous' else 3,friction=.05)
def component(name,parent,pos,size,mass,color='steel',shape='box',rpy=(0,0,0)):
    link(name,shape,size,mass,color,rpy);joint(name+'_mount',parent,name,pos)
def contact(name,mu=.8):
    g=sub(robot,'gazebo',reference=name)
    for k,v in [('mu1',mu),('mu2',mu),('kp',100000),('kd',12),('maxVel',.3),('minDepth',.0005)]:sub(g,k,v)

sub(robot,'link',name='base_footprint')
link('base_link','box',(.60,.54,.11),10,'dark');joint('base_mount_joint','base_footprint','base_link',(0,0,.16))
# Low equipment bay and actual protective collision panels, with a clear front pickup space.
component('battery_bay','base_link',(-.14,0,.02),(.25,.36,.06),2.1,'steel')
component('electronics_cover','base_link',(-.14,0,.055),(.25,.36,.014),.25,'light')
component('front_bumper','base_link',(.30,0,-.015),(.025,.56,.07),.18,'rubber')
component('rear_bumper','base_link',(-.30,0,-.015),(.025,.56,.07),.18,'rubber')
for side,sgn in [('left',1),('right',-1)]:
    component(side+'_equipment_access_panel','base_link',(-.04,sgn*.277,0),(.34,.012,.075),.12,'steel')
    for vent in range(4):
        component(f'{side}_vent_{vent}','base_link',(-.13+vent*.055,sgn*.284,.005),(.028,.003,.030),.006,'dark')
    for end,x in [('front',.19),('rear',-.19)]:
        n=f'{end}_{side}_wheel';link(n,'cylinder',(.10,.046),.52,'rubber',(math.pi/2,0,0))
        joint(n+'_joint','base_link',n,(x,sgn*.285,-.06),'continuous',(0,1,0),{'effort':18,'velocity':12})
        contact(n,.85)
        component(n+'_hub',n,(0,sgn*.025,0),(.040,.012),.035,'light','cylinder',(math.pi/2,0,0))
    # Tall fixed channels are load-bearing supports, not skins concealing an unsupported slider.
    component(side+'_fixed_mast','base_link',(-.23,sgn*.235,.27),(.055,.046,.44),.75,'steel')
    component(side+'_mast_rail','base_link',(-.196,sgn*.235,.27),(.014,.022,.42),.12,'light')
    component(side+'_side_guide_panel','base_link',(-.095,sgn*.265,.20),(.25,.016,.27),.30,'steel')
    component(side+'_guide_top_rail','base_link',(-.095,sgn*.265,.34),(.27,.025,.018),.10,'light')
# Wide deck supports a carried cube after it is lifted, not a magic attachment.
component('load_deck','base_link',(-.01,0,.075),(.58,.38,.015),.65,'light')
component('deck_rear_stop','base_link',(-.285,0,.127),(.025,.48,.10),.15,'dark')
for side,sgn in [('left',1),('right',-1)]:
    component('deck_'+side+'_outer_ledge','base_link',(-.01,sgn*.255,.075),(.55,.025,.015),.09,'light')
    component('deck_'+side+'_fence','base_link',(-.01,sgn*.247,.119),(.55,.018,.075),.15,'dark')
component('mast_fixed_crosshead','base_link',(-.23,0,.481),(.06,.50,.025),.24,'steel')

link('moving_mast_crosshead','box',(.055,.46,.045),.65,'steel')
joint('mast_extension_joint','base_link','moving_mast_crosshead',(-.23,0,.02),'prismatic',(0,0,1),{'lower':-.08,'upper':.28,'effort':350,'velocity':.18})
for side,sgn in [('left',1),('right',-1)]:
    component(side+'_inner_mast','moving_mast_crosshead',(.037,sgn*.209,.22),(.032,.028,.42),.43,'light')
    component(side+'_outer_bearing','moving_mast_crosshead',(.001,sgn*.235,.16),(.075,.065,.10),.18,'dark')
component('inner_mast_crosshead','moving_mast_crosshead',(.037,0,.43),(.045,.44,.03),.19,'steel')

link('lift_carriage','box',(.07,.40,.04),.65,'steel')
joint('lift_joint','moving_mast_crosshead','lift_carriage',(.08,0,.39),'prismatic',(0,0,1),{'lower':-.17,'upper':.19,'effort':300,'velocity':.20})
for side,sgn in [('left',1),('right',-1)]:
    component(side+'_carriage_bearing','lift_carriage',(-.043,sgn*.209,0),(.060,.060,.10),.12,'dark')
    component(side+'_carriage_guide_leg','lift_carriage',(-.043,sgn*.209,-.08),(.032,.030,.20),.13,'light')
    component(side+'_lower_carriage_bearing','lift_carriage',(-.043,sgn*.209,-.16),(.060,.060,.10),.12,'dark')
    component(side+'_reach_outer_rail','lift_carriage',(.16,sgn*.15,0),(.36,.045,.045),.22,'steel')

link('reach_stage','box',(.08,.34,.045),.48,'blue')
joint('reach_joint','lift_carriage','reach_stage',(.13,0,0),'prismatic',(1,0,0),{'lower':0,'upper':.42,'effort':220,'velocity':.18})
for side,sgn in [('left',1),('right',-1)]:
    component(side+'_reach_inner_rail','reach_stage',(-.14,sgn*.15,0),(.44,.026,.026),.16,'light')
# Frame origin is cube contact center; an overhead bridge avoids intersecting a 350 mm cube.
link('tool_backbone','box',(.035,.46,.04),.38,'steel')
joint('tool_mount_joint','reach_stage','tool_backbone',(.22,0,0))
for side,sgn in [('left',1),('right',-1)]:
    component(side+'_tool_bridge','reach_stage',(.11,sgn*.15,0),(.25,.035,.035),.18,'steel')
for side,sgn in [('left',1),('right',-1)]:
    component(side+'_tool_back_brace','tool_backbone',(-.17,sgn*.22,0),(.36,.035,.04),.15,'steel')
    component(side+'_tool_drop_bracket','tool_backbone',(-.34,sgn*.22,-.105),(.05,.035,.21),.22,'steel')
    component(side+'_jaw_linear_guide','tool_backbone',(-.34,sgn*.145,-.22),(.055,.29,.035),.20,'light')

for side,sgn in [('left',1),('right',-1)]:
    n='jaw_'+side+'_link';link(n,'box',(.40,.025,.13),.25,'steel')
    color='.92 .39 .10 1' if side=='left' else '.08 .47 .78 1'
    material=robot.find(f"link[@name='{n}']/visual/material")
    material.set('name',f'tr_jaw_{side}_metal');material.find('color').set('rgba',color)
    # Contact center in footprint: x=.10+reach, z=.35+extension+lift.
    joint('jaw_'+side+'_joint','tool_backbone',n,(-.10,sgn*.091,-.22),'prismatic',(0,sgn,0),{'lower':0,'upper':.16,'effort':80,'velocity':.12})
    component(side+'_jaw_slider_bearing',n,(-.24,0,0),(.10,.06,.07),.10,'dark')
    component(side+'_pad_servo',n,(0,sgn*.034,0),(.065,.045,.065),.12,'dark')
    component(side+'_pad_bearing',n,(0,-sgn*.019,0),(.025,.030),.05,'light','cylinder',(math.pi/2,0,0))
    pad='rotating_pad_'+side;link(pad,'box',(.19,.012,.13),.07,'pad')
    # Preserve the existing team's two-layer pad visuals within the unchanged collision.
    pad_link=robot.find(f"link[@name='{pad}']")
    pad_link.remove(pad_link.find('visual'))
    for suffix,offset,thickness,rgba in [('metal_backing',sgn*.0015,.009,color),
                                        ('inward_rubber',-sgn*.0045,.003,'.10 .11 .12 1')]:
        visual=sub(pad_link,'visual',name=f'{side}_{suffix}')
        sub(visual,'origin',xyz=xyz((0,offset,0)),rpy='0 0 0')
        sub(sub(visual,'geometry'),'box',size=xyz((.19,thickness,.13)))
        sub(sub(visual,'material',name=f'tr_{side}_{suffix}'),'color',rgba=rgba)
    joint('pad_'+side+'_spin_joint',n,pad,(0,-sgn*.035,0),'revolute',(0,1,0),{'lower':-1.57079632679,'upper':1.57079632679,'effort':6,'velocity':.5})
    contact(pad,1.0)
sub(robot,'link',name='tool_contact_frame')
joint('tool_contact_mount_joint','tool_backbone','tool_contact_frame',(-.10,0,-.22))

# Visible mid-360 with a rear brace; IMU shares the optical-frame origin in this prototype.
component('sensor_rear_post','base_link',(-.275,0,.27),(.025,.030,.42),.22,'steel')
component('sensor_top_bracket','base_link',(-.23,0,.477),(.12,.09,.016),.08,'dark')
component('lidar_link','base_link',(-.25,0,.505),(.065,.065,.060),.25,'dark')
component('lidar_window','lidar_link',(0,0,.010),(.067,.067,.018),.01,'blue')
link('imu_link','box',(.005,.005,.005),.005,'dark');joint('imu_mount_joint','lidar_link','imu_link')
component('front_camera_link','base_link',(.292,0,.045),(.040,.060,.030),.045,'dark')
component('wrist_camera_link','tool_backbone',(-.18,0,.015),(.035,.055,.030),.04,'dark')
robot.find("joint[@name='wrist_camera_link_mount']/origin").set('rpy','0 1.05 0')
robot.find("joint[@name='front_camera_link_mount']/origin").set('rpy','0 .12 0')

g=sub(robot,'gazebo')
p=sub(g,'plugin',filename='ignition-gazebo-diff-drive-system',name='ignition::gazebo::systems::DiffDrive')
for n in ('front_left_wheel_joint','rear_left_wheel_joint'):sub(p,'left_joint',n)
for n in ('front_right_wheel_joint','rear_right_wheel_joint'):sub(p,'right_joint',n)
for k,v in [('topic','/tr_v2/cmd_vel'),('odom_topic','/tr_v2/wheel/odometry'),('tf_topic','/tr_v2/wheel/tf'),
            ('frame_id','tr_v2/wheel_odom'),('child_frame_id','tr_v2/base_footprint'),('wheel_separation',.57),('wheel_radius',.10),
            ('max_linear_velocity',.25),('min_linear_velocity',-.25),('max_angular_velocity',.6),('min_angular_velocity',-.6),
            ('max_linear_acceleration',.3),('min_linear_acceleration',-.3)]:sub(p,k,v)
controlled=['mast_extension_joint','lift_joint','reach_joint','jaw_left_joint','jaw_right_joint','pad_left_spin_joint','pad_right_spin_joint']
for name in controlled:
    p=sub(g,'plugin',filename='ignition-gazebo-joint-position-controller-system',name='ignition::gazebo::systems::JointPositionController')
    values={'joint_name':name,'topic':f'/tr_v2/joint/{name}/cmd_pos','p_gain':1200 if 'lift' in name or 'mast' in name else 500,
            'i_gain':150 if 'lift' in name or 'mast' in name else 40,'d_gain':65 if 'lift' in name or 'mast' in name else 15,
            'i_max':150,'i_min':-150,'cmd_max':300 if 'lift' in name or 'mast' in name else 80,'cmd_min':-300 if 'lift' in name or 'mast' in name else -80}
    for k,v in values.items():sub(p,k,v)
p=sub(g,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher');sub(p,'topic','/tr_v2/sim/joint_states')
p=sub(g,'plugin',filename='ignition-gazebo-odometry-publisher-system',name='ignition::gazebo::systems::OdometryPublisher')
for k,v in [('dimensions',3),('odom_publish_frequency',50),('odom_topic','/tr_v2/ground_truth/odometry'),('odom_frame','world'),('robot_base_frame','tr_v2/base_footprint')]:sub(p,k,v)

sensor_g=sub(robot,'gazebo',reference='lidar_link');s=sub(sensor_g,'sensor',name='tr_v2_mid360',type='gpu_lidar')
for k,v in [('topic','/tr_v2/sim/lidar'),('update_rate',10),('always_on','true'),('visualize','false'),('ignition_frame_id','tr_v2/lidar_link')]:sub(s,k,v)
scan=sub(sub(s,'ray'),'scan')
for typ,n,lo,hi in [('horizontal',360,-math.pi,math.pi),('vertical',32,-.12217,.90757)]:
    e=sub(scan,typ)
    for k,v in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]:sub(e,k,v)
r=sub(s.find('ray'),'range')
for k,v in [('min',.10),('max',40),('resolution',.01)]:sub(r,k,v)
sensor_g=sub(robot,'gazebo',reference='imu_link');s=sub(sensor_g,'sensor',name='tr_v2_mid360_imu',type='imu')
for k,v in [('topic','/tr_v2/sim/imu'),('update_rate',200),('always_on','true'),('ignition_frame_id','tr_v2/imu_link')]:sub(s,k,v)
for name,topic in [('front_camera_link','color'),('wrist_camera_link','wrist')]:
    s=sub(sub(robot,'gazebo',reference=name),'sensor',name='tr_v2_'+topic,type='camera')
    for k,v in [('topic',f'/tr_v2/camera/{topic}/image'),('update_rate',15),('always_on','true')]:sub(s,k,v)
    cam=sub(s,'camera');sub(cam,'horizontal_fov',1.05);im=sub(cam,'image')
    for k,v in [('width',480),('height',360),('format','R8G8B8')]:sub(im,k,v)
    cl=sub(cam,'clip');sub(cl,'near',.03);sub(cl,'far',12)

stylize_sensors(robot, 'tr', ROOT)
ET.indent(robot)
(ROOT/'urdf'/f'{NAME}.urdf').write_text(ET.tostring(robot,encoding='unicode'))
bridge=[]
def bridge_entry(topic,ros,gz,direction='GZ_TO_ROS',gz_topic=None):
    bridge.append(dict(ros_topic_name=topic,gz_topic_name=gz_topic or topic,ros_type_name=ros,gz_type_name=gz,direction=direction))
bridge_entry('/tr_v2/cmd_vel','geometry_msgs/msg/Twist','ignition.msgs.Twist','ROS_TO_GZ')
bridge_entry('/tr_v2/sim/lidar','sensor_msgs/msg/PointCloud2','ignition.msgs.PointCloudPacked',gz_topic='/tr_v2/sim/lidar/points')
bridge_entry('/tr_v2/sim/imu','sensor_msgs/msg/Imu','ignition.msgs.IMU')
bridge_entry('/tr_v2/joint_states','sensor_msgs/msg/JointState','ignition.msgs.Model',gz_topic='/tr_v2/sim/joint_states')
for t in ('wheel/odometry','ground_truth/odometry'):bridge_entry('/tr_v2/'+t,'nav_msgs/msg/Odometry','ignition.msgs.Odometry')
for n in controlled:bridge_entry(f'/tr_v2/joint/{n}/cmd_pos','std_msgs/msg/Float64','ignition.msgs.Double','ROS_TO_GZ')
for t in ('color','wrist'):bridge_entry(f'/tr_v2/camera/{t}/image','sensor_msgs/msg/Image','ignition.msgs.Image')
(ROOT/'config'/'video_v2_bridge.yaml').write_text(yaml.safe_dump(bridge,sort_keys=False))
poses={
 'stowed':[0,0,0,0,0,0,0],
 'extend_clear':[0,0,.42,.16,.16,0,0],
 'open':[0,-.17,.42,.16,.16,0,0],
 'open200':[-.08,-.17,.42,.16,.16,0,0],
 'grip350':[0,-.17,.42,.121,.121,0,0],
 'grip200':[-.08,-.17,.42,.046,.046,0,0],
 'clear_fences':[0,.18,.42,.121,.121,0,0],
 'over_deck':[0,.18,0,.121,.121,0,0],
 'holding350':[0,.0675,0,.121,.121,0,0],
 'holding200':[0,-.0075,.12,.046,.046,0,0],
 'raised':[.18,.08,.10,.121,.121,0,0],
 'place':[.18,0,.36,.16,.16,0,0],
 'flip_cap_ready':[.10,.10,.42,.046,.046,0,0],
 'flip_cap_90':[.10,.10,.42,.046,.046,math.pi/2,math.pi/2]}
config={'prototype':True,'video_confirmed':'general mast/load-tray arrangement and two rotating finger pads; dimensions and ordinary-wheel differential drive are design hypotheses',
 'model_name':NAME,'namespace':NS,'joint_order':controlled,'poses':{k:dict(zip(controlled,v)) for k,v in poses.items()},
 'geometry':{'chassis_m':[.60,.54,.11],'nominal_stowed_envelope_estimate_m':[.693,.632,.695],
             'jaw_inner_gap_formula_m':'.100 + q_left + q_right','max_open_gap_m':.42,
             'tool_center_footprint_xyz_m':['.10 + reach','0','.35 + mast_extension + lift'],
             'load_deck_top_height_footprint_m':.2425,'lidar_imu_footprint_xyz_m':[-.25,0,.665],
             'T_IL':'identity, simulated co-origin','sensor_model':'360x32 simultaneous GPU raster, not true nonrepetitive MID-360'},
 'unverified':['grasp/rotate/release with physical blocks','full joint sweeps and sensor occlusion','3-object magazine','stairs and climbing auxiliaries','payload and friction calibration'],
 'physical_logic':'grip ahead of chassis, lift above deck/fences, retract onto load deck; both pads rotate together only for a single 200mm cap, while jaw closure remains lateral'}
(ROOT/'config'/'video_v2_geometry.yaml').write_text(yaml.safe_dump(config,sort_keys=False,allow_unicode=True))
print(ROOT/'urdf'/f'{NAME}.urdf')
