#!/usr/bin/env python3
"""Visual-only MID-360 / camera identifiers. Does not edit physics or frames."""
import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as E

GREEN = '.28 .72 .08 1'
GLASS = '.025 .045 .055 1'
SILVER = '.72 .76 .79 1'
WHITE = '.94 .96 .94 1'

def visual(link, name, shape, size, xyz=(0, 0, 0), rpy=(0, 0, 0), color=GREEN):
    e = E.SubElement(link, 'visual', name=name)
    E.SubElement(e, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    g = E.SubElement(e, 'geometry')
    if shape == 'box': E.SubElement(g, shape, size=' '.join(map(str, size)))
    elif shape == 'cylinder': E.SubElement(g, shape, radius=str(size[0]), length=str(size[1]))
    else: E.SubElement(g, 'mesh', filename=size)
    m = E.SubElement(e, 'material', name='sensor_identifier_' + name)
    E.SubElement(m, 'color', rgba=color)

def label_mesh(path):
    """A single untextured mesh, not dozens of tiny render objects."""
    glyphs = {
        'M':['10001','11011','10101','10101','10001','10001','10001'],
        'I':['11111','00100','00100','00100','00100','00100','11111'],
        'D':['11110','10001','10001','10001','10001','10001','11110'],
        '3':['11110','00001','00001','01110','00001','00001','11110'],
        '6':['01110','10000','10000','11110','10001','10001','01110'],
        '0':['01110','10001','10011','10101','11001','10001','01110'],
    }
    vertices=[]; indices=[]; pitch=.00105
    for k, c in enumerate('MID360'):
        for row, line in enumerate(glyphs[c]):
            col=0
            while col < len(line):
                if line[col] != '1': col+=1; continue
                start=col
                while col < len(line) and line[col]=='1': col+=1
                y0=(k*6+start-17.5)*pitch; y1=(k*6+col-17.5)*pitch
                z0=(3-row)*pitch; z1=(4-row)*pitch
                n=len(vertices); vertices.extend([(0,y0,z0),(0,y1,z0),(0,y1,z1),(0,y0,z1)])
                indices.extend([n,n+1,n+2,n,n+2,n+3])
    values=' '.join(str(v) for xyz in vertices for v in xyz)
    xml=f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
 <library_geometries><geometry id="label"><mesh>
  <source id="pos"><float_array id="pos-data" count="{len(vertices)*3}">{values}</float_array><technique_common><accessor source="#pos-data" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <vertices id="verts"><input semantic="POSITION" source="#pos"/></vertices>
  <triangles count="{len(indices)//3}"><input semantic="VERTEX" source="#verts" offset="0"/><p>{' '.join(map(str,indices))}</p></triangles>
 </mesh></geometry></library_geometries>
 <library_visual_scenes><visual_scene id="scene"><node id="label-node"><instance_geometry url="#label"/></node></visual_scene></library_visual_scenes>
 <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>'''
    path.parent.mkdir(parents=True, exist_ok=True); path.write_text(xml)

def without_visuals(root):
    r=copy.deepcopy(root)
    for l in r.findall('link'):
        for v in l.findall('visual'): l.remove(v)
    # Ignore formatting whitespace; compare exact element/attribute/text meaning.
    def value(e): return (e.tag, tuple(sorted(e.attrib.items())), (e.text or '').strip(), tuple(value(c) for c in e))
    return value(r)

def stylize_sensors(root, robot, package_root):
    before=without_visuals(root)
    lidar=root.find("link[@name='lidar_link']")
    for v in lidar.findall('visual'): lidar.remove(v)
    # Identifiable visual proxy within the original 65 x 65 x 60 mm hull.
    # Not a manufacturer CAD or a new LiDAR / IMU coordinate calibration.
    visual(lidar,'mid360_green_base','box',(.065,.065,.018),(0,0,-.021))
    visual(lidar,'mid360_optical_window','cylinder',(.0315,.034),(0,0,.010),color=GLASS)
    visual(lidar,'mid360_top_lip','cylinder',(.0318,.003),(0,0,.0285),color=GREEN)
    visual(lidar,'mid360_metal_foot','box',(.063,.063,.002),(0,0,-.029),color=SILVER)
    uri=f'package://robocon_{robot}_description/urdf/meshes/mid360_label.dae'
    visual(lidar,'mid360_front_label','mesh',uri,(.0326,0,-.021),color=WHITE)
    visual(lidar,'mid360_rear_label','mesh',uri,(-.0326,0,-.021),(0,0,3.14159265359),WHITE)
    label_mesh(Path(package_root)/'urdf/meshes/mid360_label.dae')
    # The old TR blue window is a separate fixed link with physical properties.
    # Leave that link, collision, mass and joint intact; suppress only its visual.
    window=root.find("link[@name='lidar_window']")
    if window is not None:
        for v in window.findall('visual'): window.remove(v)
    bracket_name='sensor_top_bracket' if robot=='tr' else 'lidar_pedestal'
    bracket=root.find(f"link[@name='{bracket_name}']")
    if bracket is not None:
        for v in bracket.findall('visual'):
            color=v.find('material/color')
            if color is not None: color.set('rgba',SILVER)
    for name in ('front_camera_link','wrist_camera_link'):
        cam=root.find(f"link[@name='{name}']")
        if cam is None: continue
        # Remove only identifiers from a previous application; preserve housing.
        for v in list(cam.findall('visual')):
            if v.get('name','').startswith('sensor_camera_'): cam.remove(v)
        body=cam.find('visual/geometry/box')
        half=float(body.get('size').split()[0])/2 if body is not None else .0175
        visual(cam,'sensor_camera_lens_rim','cylinder',(.011,.003),(half+.0015,0,0),(0,1.57079632679,0),SILVER)
        visual(cam,'sensor_camera_lens_glass','cylinder',(.0085,.0032),(half+.003,0,0),(0,1.57079632679,0),'.04 .30 .50 1')
    if before != without_visuals(root):
        raise RuntimeError('A non-visual element changed; refusing to save.')
    return root

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('urdf',type=Path); p.add_argument('--robot',choices=['br','tr'],required=True)
    p.add_argument('--package-root',type=Path)
    a=p.parse_args(); root=E.parse(a.urdf).getroot()
    stylize_sensors(root,a.robot,a.package_root or a.urdf.parent.parent)
    E.indent(root,space='  '); a.urdf.write_text(E.tostring(root,encoding='unicode'))
    print('Updated visual elements only:',a.urdf)
if __name__=='__main__': main()
