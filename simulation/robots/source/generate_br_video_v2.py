#!/usr/bin/env python3
"""Independent, explicitly dimensioned video-inspired BR prototype, not a CAD copy."""
from pathlib import Path
import math,copy,os,shutil,xml.etree.ElementTree as E
import numpy as np,yaml
from sensor_visuals_v2_20261022 import stylize_sensors
SOURCE=Path(__file__).resolve().parent
D=Path(os.environ.get('BR_VIDEO_V2_ROOT',SOURCE/'generated/robocon_br_description'))
(D/'urdf').mkdir(parents=True,exist_ok=True)
(D/'config').mkdir(parents=True,exist_ok=True)
shutil.copy2(SOURCE/'templates/robocon_br_proto.urdf',D/'urdf/robocon_br_proto.urdf')
r=E.Element('robot',name='robocon_br_video_v2');r.append(E.Comment('VIDEO-INSPIRED PROTOTYPE. Mechanical dimensions are design values, not dimensions measured from the video.'))
COL={'frame':'.50 .54 .58 1','dark':'.10 .13 .16 1','light':'.72 .75 .78 1','blue':'.08 .32 .68 1','pad':'.055 .06 .07 1','yellow':'.90 .70 .12 1','sensor':'.22 .25 .28 1','grip_left':'.92 .39 .10 1','grip_right':'.08 .47 .78 1','bearing':'.78 .81 .84 1','screw':'.58 .46 .24 1','rubber':'.10 .11 .12 1'}
def put(p,tag,text):e=E.SubElement(p,tag);e.text=str(text);return e
def box(xyz,size,color='frame'):return ('box',xyz,size,color,(0,0,0))
def cyl(xyz,radius,length,color='frame',rpy=(math.pi/2,0,0)):return ('cylinder',xyz,(radius,length),color,rpy)
def rotation(v):
 a,b,c=v;ca,sa,cb,sb,cc,sc=math.cos(a),math.sin(a),math.cos(b),math.sin(b),math.cos(c),math.sin(c)
 return np.array([[cc*cb,cc*sb*sa-sc*ca,cc*sb*ca+sc*sa],[sc*cb,sc*sb*sa+cc*ca,sc*sb*ca-cc*sa],[-sb,cb*sa,cb*ca]])
def link(name,mass,parts):
 l=E.SubElement(r,'link',name=name)
 if parts:
  vols=[np.prod(s) if t=='box' else math.pi*s[0]**2*s[1] for t,x,s,c,a in parts];weights=np.array(vols)/sum(vols)*mass;com=sum(w*np.array(p[1]) for w,p in zip(weights,parts))/mass;I=np.zeros((3,3))
  for w,(typ,xyz,size,color,rpy) in zip(weights,parts):
   if typ=='box':x,y,z=size;J=np.diag([w*(y*y+z*z)/12,w*(x*x+z*z)/12,w*(x*x+y*y)/12])
   else:rad,le=size;J=np.diag([w*(3*rad*rad+le*le)/12]*2+[w*rad*rad/2])
   R=rotation(rpy);v=np.array(xyz)-com;I+=R@J@R.T+w*((v@v)*np.eye(3)-np.outer(v,v))
  it=E.SubElement(l,'inertial');E.SubElement(it,'origin',xyz=' '.join(map(str,com)));E.SubElement(it,'mass',value=str(mass));E.SubElement(it,'inertia',ixx=str(I[0,0]),iyy=str(I[1,1]),izz=str(I[2,2]),ixy=str(I[0,1]),ixz=str(I[0,2]),iyz=str(I[1,2]))
  for n,(typ,xyz,size,color,rpy) in enumerate(parts):
   for tag in ['visual','collision']:
    el=E.SubElement(l,tag,name=f'{name}_{n}_{tag}');E.SubElement(el,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)));g=E.SubElement(el,'geometry')
    if typ=='box':E.SubElement(g,'box',size=' '.join(map(str,size)))
    else:E.SubElement(g,'cylinder',radius=str(size[0]),length=str(size[1]))
    if tag=='visual':mat=E.SubElement(el,'material',name=f'v2_{color}');E.SubElement(mat,'color',rgba=COL[color])
 return l
def visual_only(l,name,part):
 # Cosmetic representation of the already-modeled actuator/guide, no new physics.
 typ,xyz,size,color,rpy=part
 v=E.SubElement(l,'visual',name=name);E.SubElement(v,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)));g=E.SubElement(v,'geometry')
 if typ=='box':E.SubElement(g,'box',size=' '.join(map(str,size)))
 else:E.SubElement(g,'cylinder',radius=str(size[0]),length=str(size[1]))
 m=E.SubElement(v,'material',name=f'v2_{color}');E.SubElement(m,'color',rgba=COL[color]);return v
def joint(name,parent,child,typ='fixed',xyz=(0,0,0),axis=(0,0,1),lower=0,upper=0,effort=100,velocity=.2,rpy=(0,0,0)):
 j=E.SubElement(r,'joint',name=name,type=typ);E.SubElement(j,'parent',link=parent);E.SubElement(j,'child',link=child);E.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
 if typ!='fixed':E.SubElement(j,'axis',xyz=' '.join(map(str,axis)));E.SubElement(j,'limit',lower=str(lower),upper=str(upper),effort=str(effort),velocity=str(velocity));E.SubElement(j,'dynamics',damping='2' if typ=='revolute' else '8',friction='.05' if typ=='revolute' else '.5')
 return j
# Existing validated wheel branches, now with handedness matching MecanumDrive.
old=E.parse(D/'urdf/robocon_br_proto.urdf').getroot();E.SubElement(r,'link',name='base_footprint')
link('base_link',5,[box((0,.215,0),(.54,.07,.12),'dark'),box((0,-.215,0),(.54,.07,.12),'dark'),box((-.235,0,0),(.07,.36,.12),'dark')]);joint('base_mount_joint','base_footprint','base_link',xyz=(0,0,.22))
for e in old:
 if (e.tag=='link' and 'wheel' in e.get('name','')) or (e.tag=='joint' and 'wheel' in e.get('name','')) or (e.tag=='gazebo' and 'wheel' in e.get('reference','')):r.append(copy.deepcopy(e))
link('rear_battery_bay',4,[box((-.19,0,.10),(.13,.34,.12),'dark')]);joint('battery_mount','base_link','rear_battery_bay')
link('cargo_side_structure',1.8,[box((-.045,.245,.235),(.39,.018,.30)),box((-.045,-.245,.235),(.39,.018,.30)),box((-.24,0,.235),(.025,.49,.30)),box((-.045,.245,.39),(.39,.025,.018),'light'),box((-.045,-.245,.39),(.39,.025,.018),'light')]);joint('cargo_structure_mount','base_link','cargo_side_structure')
link('fixed_lift_guides',1.5,[box((.14,.31,.23),(.03,.03,.34),'dark'),box((.14,-.31,.23),(.03,.03,.34),'dark'),box((-.22,0,.405),(.03,.65,.025),'light'),box((-.04,.31,.405),(.39,.025,.025),'light'),box((-.04,-.31,.405),(.39,.025,.025),'light'),box((.14,.265,.075),(.06,.12,.025)),box((.14,-.265,.075),(.06,.12,.025))]);joint('fixed_guide_mount','base_link','fixed_lift_guides')
# Each rail is 320 mm; 230 mm per-stage travel leaves at least 90 mm rail overlap.
for i in [1,2,3]:
 name=f'lift_stage{i}';parent='base_link' if i==1 else f'lift_stage{i-1}';xyz=(.175,0,-.09) if i==1 else (.035,0,0)
 parts=[box((0,.31,.16),(.025,.025,.32),'light'),box((0,-.31,.16),(.025,.025,.32),'light'),box((-.38,0,.29),(.025,.645,.025)),box((-.19,.31,.29),(.38,.025,.025)),box((-.19,-.31,.29),(.38,.025,.025))]
 link(name,.70,parts);joint(f'lift_stage{i}_joint',parent,name,'prismatic',xyz,upper=.23,effort=220,velocity=.15)
# Hollow outer box section and supported inner reach member.
link('reach_outer_guide',.65,[box((-.19,.045,.21),(.38,.015,.08),'dark'),box((-.19,-.045,.21),(.38,.015,.08),'dark'),box((-.19,0,.2425),(.38,.075,.015),'dark'),box((-.19,0,.1775),(.38,.075,.015),'dark')]);joint('reach_outer_mount','lift_stage3','reach_outer_guide',xyz=(.025,0,.04))
link('reach_stage',.55,[box((-.19,0,.21),(.38,.05,.04),'light'),box((.02,0,.21),(.08,.54,.04))]);joint('reach_joint','reach_outer_guide','reach_stage','prismatic',axis=(1,0,0),upper=.30,effort=100,velocity=.20)
link('gripper_base',.08,[box((.02,0,.21),(.08,.10,.04),'blue')]);joint('gripper_mount_joint','reach_stage','gripper_base')
for side,sgn in [('left',1),('right',-1)]:
 name=f'jaw_{side}_carrier';parts=[box((.02,.035*sgn,.105),(.035,.03,.21)),box((0,.035*sgn,0),(.05,.03,.045),'blue'),box((.02,0,.21),(.07,.04,.04),'light')]
 link(name,.18,parts);joint(f'jaw_{side}_joint','gripper_base',name,'prismatic',(0,.05*sgn,0),(0,sgn,0),upper=.17,effort=30,velocity=.10)
 pad=f'pad_{side}_link';link(pad,.10,[box((0,0,0),(.12,.025,.12),'pad')]);joint(f'pad_{side}_rotate_joint',name,pad,'revolute',axis=(0,1,0),lower=-math.pi,upper=math.pi,effort=3,velocity=.5)
 g=E.SubElement(r,'gazebo',reference=pad)
 for k,v in [('mu1',.9),('mu2',.9),('kp',50000),('kd',5)]:put(g,k,v)
# Make the existing lateral/rotary actuation readable. Every addition in this block
# is visual-only on existing links: no new link, collision, mass, joint or drive law.
# Actuator housings represent hardware already lumped into the prototype link masses;
# screw pitch, belt compliance and bearing mechanics are not newly simulated here.
beam=r.find("link[@name='reach_stage']")
beam.find("visual[@name='reach_stage_1_visual']/material/color").set('rgba',COL['dark'])
for z in [.20,.23]:visual_only(beam,f'jaw_guide_rod_{z}',cyl((.072,0,z),.005,.50,'bearing'))
for side,sgn in [('left',1),('right',-1)]:
 color='grip_left' if sgn==1 else 'grip_right'
 visual_only(beam,f'{side}_independent_drive_screw',cyl((.085,.13*sgn,.215),.004,.22,'screw'))
 visual_only(beam,f'{side}_linear_motor_cover',box((.075,.285*sgn,.215),(.04,.055,.055),color))
 visual_only(beam,f'{side}_drive_end_bearing',cyl((.085,.25*sgn,.215),.012,.016,'dark'))
 carriage=r.find(f"link[@name='jaw_{side}_carrier']")
 carriage.find('visual/material/color').set('rgba',COL[color])
 visual_only(carriage,f'{side}_moving_guide_carriage',box((.075,0,.215),(.03,.055,.055),'bearing'))
 visual_only(carriage,f'{side}_carriage_connection',box((.055,0,.215),(.025,.035,.035),color))
 visual_only(carriage,f'{side}_rotary_drive_cover',cyl((0,.0525*sgn,0),.027,.04,color))
 visual_only(carriage,f'{side}_rotary_output_shaft',cyl((0,.0225*sgn,0),.012,.020,'bearing'))
 pad=r.find(f"link[@name='pad_{side}_link']")
 for v in list(pad.findall('visual')):pad.remove(v)
 # The two visual layers exactly fill the unchanged 25 mm collision thickness.
 visual_only(pad,f'{side}_metal_pad_backing',box((0,.0025*sgn,0),(.12,.020,.12),color))
 visual_only(pad,f'{side}_inward_rubber_contact',box((0,-.010*sgn,0),(.12,.005,.12),'rubber'))
 # A thin paint stripe lies inside the pad surface extent and shows pad rotation.
 visual_only(pad,f'{side}_rotation_witness_paint',box((0,-.01245*sgn,.035),(.080,.0001,.004),'bearing'))

# A real cargo platform can slide to the front grip station; fold only when empty.
link('cargo_slide_carriage',.18,[box((.19,.205,0),(.38,.025,.025),'blue'),box((.19,-.205,0),(.38,.025,.025),'blue')]);joint('cargo_slide_joint','base_link','cargo_slide_carriage','prismatic',(-.21,0,.13),(1,0,0),upper=.29,effort=80,velocity=.15)
link('cargo_tray',.65,[box((.19,0,0),(.38,.40,.018),'light'),box((.19,.205,.035),(.38,.015,.07)),box((.19,-.205,.035),(.38,.015,.07)),box((.008,0,.035),(.015,.41,.07))]);joint('cargo_fold_joint','cargo_slide_carriage','cargo_tray','revolute',axis=(0,1,0),lower=-math.pi/2,upper=0,effort=35,velocity=.4)
# Rear anti-tip extension with a real passive contact wheel. No force is applied to the chassis.
link('tail_slide',.18,[box((.04,0,0),(.08,.08,.05),'blue')]);joint('tail_slide_joint','base_link','tail_slide','prismatic',(-.21,0,-.12),(-1,0,0),upper=.20,effort=100,velocity=.12)
link('tail_arm',.18,[box((-.11,0,0),(.22,.035,.035),'light')]);joint('tail_pitch_joint','tail_slide','tail_arm','revolute',rpy=(0,math.pi/2,0),axis=(0,1,0),lower=-1.90,upper=0,effort=25,velocity=.5)
link('tail_contact_wheel',.05,[cyl((0,0,0),.035,.035,'yellow')]);joint('tail_contact_wheel_joint','tail_arm','tail_contact_wheel','continuous',(-.22,0,0),(0,1,0),effort=1,velocity=30)
# Visible fixed rear sensor mast; 65x65x60 mm, 265 g combined sensor housing.
link('lidar_pedestal',.15,[box((-.27,0,.251),(.025,.025,.292),'dark'),box((-.27,0,.403),(.10,.09,.012),'light')]);joint('lidar_pedestal_mount_joint','base_link','lidar_pedestal')
link('lidar_link',.260,[box((0,0,0),(.065,.065,.060),'sensor')]);joint('lidar_mount_joint','base_link','lidar_link',xyz=(-.27,0,.439))
link('imu_link',.005,[box((0,0,0),(.008,.008,.008),'blue')]);joint('imu_mount_joint','base_link','imu_link',xyz=(-.27,0,.439))
link('front_camera_link',.035,[box((0,0,0),(.025,.035,.025),'sensor')]);joint('front_camera_mount_joint','reach_stage','front_camera_link',xyz=(.07,0,.255),rpy=(0,.8,0))
# Copy the snapshot lidar/IMU measurement model; it remains explicitly not MID-360 scanning time.
for ref,typ,topic in [('lidar_link','gpu_lidar','/br_v2/sim/lidar'),('imu_link','imu','/br_v2/sim/imu')]:
 oldg=old.find(f"gazebo[@reference='{ref}']");g=E.SubElement(r,'gazebo',reference=ref);s=copy.deepcopy(oldg.find('sensor'));s.set('name','br_v2_'+typ);s.find('topic').text=topic
 if s.find('ignition_frame_id') is not None:s.find('ignition_frame_id').text='br_v2/'+ref
 g.append(s);put(g,'preserveFixedJoint','true')
g=E.SubElement(r,'gazebo');p=E.SubElement(g,'plugin',filename='ignition-gazebo-mecanum-drive-system',name='ignition::gazebo::systems::MecanumDrive')
for k,v in {'topic':'/br_v2/cmd_vel','front_left_joint':'front_left_wheel_drive_joint','front_right_joint':'front_right_wheel_drive_joint','back_left_joint':'rear_left_wheel_drive_joint','back_right_joint':'rear_right_wheel_drive_joint','wheel_separation':.44,'wheelbase':.40,'wheel_radius':.12,'min_acceleration':-.4,'max_acceleration':.4,'odom_topic':'/br_v2/wheel/odometry','tf_topic':'/br_v2/wheel/tf','frame_id':'br_v2/odom','child_frame_id':'br_v2/base_footprint'}.items():put(p,k,v)
act=[]
for j in r.findall('joint'):
 if j.get('type') not in ['prismatic','revolute']:continue
 n=j.get('name');act.append(n);p=E.SubElement(g,'plugin',filename='ignition-gazebo-joint-position-controller-system',name='ignition::gazebo::systems::JointPositionController');maximum=float(j.find('limit').get('effort'))
 gains=(6000,200,80) if n.startswith('lift_stage') else (800,0,8) if n.startswith('jaw') else (20,0,.2) if n.startswith('pad') else (700,0,20) if n=='reach_joint' else (100,5,5)
 for k,v in {'joint_name':n,'topic':f'/br_v2/joint/{n}/cmd_pos','p_gain':gains[0],'i_gain':gains[1],'d_gain':gains[2],'i_max':100,'i_min':-100,'cmd_max':maximum,'cmd_min':-maximum,'initial_position':0}.items():put(p,k,v)
p=E.SubElement(g,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher');put(p,'topic','/br_v2/sim/joint_states')
p=E.SubElement(g,'plugin',filename='ignition-gazebo-odometry-publisher-system',name='ignition::gazebo::systems::OdometryPublisher')
for k,v in {'odom_frame':'world','robot_base_frame':'br_v2/base_footprint','dimensions':3,'odom_publish_frequency':50,'odom_topic':'/br_v2/ground_truth/odometry','tf_topic':'/br_v2/ground_truth/pose','gaussian_noise':0}.items():put(p,k,v)
stylize_sensors(r, 'br', D)
E.indent(r,space='  ');E.ElementTree(r).write(D/'urdf/robocon_br_video_v2.urdf',encoding='utf-8',xml_declaration=True)
bridges=[]
def b(ros,gz,rt,gt,direction):bridges.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction=direction))
b('/br_v2/cmd_vel','/br_v2/cmd_vel','geometry_msgs/msg/Twist','ignition.msgs.Twist','ROS_TO_GZ');b('/br_v2/sim/lidar','/br_v2/sim/lidar/points','sensor_msgs/msg/PointCloud2','ignition.msgs.PointCloudPacked','GZ_TO_ROS');b('/br_v2/sim/imu','/br_v2/sim/imu','sensor_msgs/msg/Imu','ignition.msgs.IMU','GZ_TO_ROS');b('/br_v2/joint_states','/br_v2/sim/joint_states','sensor_msgs/msg/JointState','ignition.msgs.Model','GZ_TO_ROS');b('/br_v2/ground_truth/odometry','/br_v2/ground_truth/odometry','nav_msgs/msg/Odometry','ignition.msgs.Odometry','GZ_TO_ROS')
for n in act:b(f'/br_v2/joint/{n}/cmd_pos',f'/br_v2/joint/{n}/cmd_pos','std_msgs/msg/Float64','ignition.msgs.Double','ROS_TO_GZ')
(D/'config/bridge_video_v2.yaml').write_text(yaml.safe_dump(bridges,sort_keys=False))
poses={n:0 for n in act}
allposes={'stowed':dict(poses),'open':dict(poses,jaw_left_joint=.17,jaw_right_joint=.17),'holding350':dict(poses,reach_joint=.25,jaw_left_joint=.13,jaw_right_joint=.13),'raised':dict(poses,reach_joint=.25,lift_stage1_joint=.20,lift_stage2_joint=.20,lift_stage3_joint=.20,jaw_left_joint=.13,jaw_right_joint=.13),'flip_cap':dict(poses,reach_joint=.25,lift_stage1_joint=.08,lift_stage2_joint=.08,lift_stage3_joint=.08,jaw_left_joint=.055,jaw_right_joint=.055,pad_left_rotate_joint=math.pi/2,pad_right_rotate_joint=math.pi/2),'place':dict(poses,reach_joint=.25,lift_stage1_joint=.20,lift_stage2_joint=.20,lift_stage3_joint=.20,jaw_left_joint=.17,jaw_right_joint=.17),'channel_open':dict(poses,lift_stage1_joint=.23,lift_stage2_joint=.23,lift_stage3_joint=.23,jaw_left_joint=.17,jaw_right_joint=.17,cargo_fold_joint=-math.pi/2),'tail_support':dict(poses,tail_slide_joint=.10,tail_pitch_joint=-1.897)}
(D/'config/poses_video_v2.yaml').write_text(yaml.safe_dump({'prototype':True,'poses':allposes},sort_keys=False))
(D/'config/video_v2_geometry.yaml').write_text(yaml.safe_dump({'prototype':True,'reference':'User video actual frames; geometry design values, not video-derived dimensions.','robot':'robocon_br_video_v2','namespace':'br_v2','lift':{'stage_count':3,'stage_rail_length_m':.32,'stroke_each_m':.23,'minimum_overlap_m':.09,'total_stroke_m':.69,'tcp_home_from_footprint_m':[.27,0,.17]},'reach':{'stroke_m':.30,'inner_tube_length_m':.38,'minimum_overlap_m':.08},'gripper':{'inner_gap_m':[.075,.415],'rotating_pad_box_m':[.12,.025,.12],'rotation_axes':'Both world-aligned local Y, not mirrored signs','rotation_limit_rad':[-math.pi,math.pi]},'cargo':{'one_tray_center_from_footprint_at_home_m':[-.02,0,.35],'slide_stroke_m':.29,'fold_empty_to_open_channel_rad':-math.pi/2,'second_payload_status':'Tray transfer reach is designed; storage+gripper two-object physical transfer not validated.'},'lidar_imu':{'from_base_link_m':[-.27,0,.439],'from_footprint_m':[-.27,0,.659],'rotation_rpy':[0,0,0],'co_location':'simulation assumption, not measured MID-360 IMU calibration'},'tail':{'slide_stroke_m':.20,'stowed_pitch_joint_rad':0,'nominal_ground_support_pitch_rad':-1.897,'status':'Real prismatic/revolute/passive-wheel model; stair or anti-tip efficacy unverified.'}},sort_keys=False))
print('Wrote',D/'urdf/robocon_br_video_v2.urdf','actuators',len(act))
