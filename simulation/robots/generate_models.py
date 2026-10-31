#!/usr/bin/env python3
"""Regenerate the existing TR/BR video_v2 models for Gazebo Fortress."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent


def generate(output, ign):
    source = HERE / 'source'
    generated = output / 'source/generated'
    models = output / 'models'
    env = os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE'] = '1'

    for robot in ('br', 'tr'):
        name = f'robocon_{robot}_video_v2'
        package = generated / f'robocon_{robot}_description'
        env[f'{robot.upper()}_VIDEO_V2_ROOT'] = str(package)
        subprocess.run([sys.executable, str(source / f'generate_{robot}_video_v2.py')],
                       env=env, check=True)

        model_dir = models / name
        model_dir.mkdir(parents=True, exist_ok=True)
        shutil.copytree(package / 'urdf/meshes', model_dir / 'meshes', dirs_exist_ok=True)
        urdf = ET.parse(package / 'urdf' / f'{name}.urdf')
        mesh_prefix = f'package://robocon_{robot}_description/urdf/meshes/'
        for mesh in urdf.getroot().findall('.//mesh'):
            mesh.set('filename', mesh.get('filename').replace(mesh_prefix, f'model://{name}/meshes/'))

        # Convert a temporary URI-adapted input; the original generated URDF stays readable.
        with tempfile.TemporaryDirectory(prefix=f'{name}-') as temp:
            input_path = Path(temp) / f'{name}.urdf'
            urdf.write(input_path, encoding='utf-8', xml_declaration=True)
            converted = subprocess.run([ign, 'sdf', '-p', str(input_path)],
                                       env=env, check=True, capture_output=True, text=True)
        if converted.stderr:
            print(converted.stderr, file=sys.stderr, end='')
        sdf = ET.ElementTree(ET.fromstring(converted.stdout))
        write_xml(sdf, model_dir / 'model.sdf')

        config = ET.Element('model')
        ET.SubElement(config, 'name').text = name
        ET.SubElement(config, 'version').text = '2.0-prototype'
        ET.SubElement(config, 'sdf', version=sdf.getroot().get('version')).text = 'model.sdf'
        ET.SubElement(config, 'description').text = (
            f'Existing team {robot.upper()} video_v2 prototype for Gazebo Fortress; '
            'original physical joints, contacts and control topics retained.')
        write_xml(ET.ElementTree(config), model_dir / 'model.config')
        print(f'Generated {model_dir}')


def write_xml(tree, path):
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE)
    parser.add_argument('--ign', default='ign', help='Ignition command available in this environment')
    args = parser.parse_args()
    generate(args.output.resolve(), args.ign)
