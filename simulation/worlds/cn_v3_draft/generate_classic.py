#!/usr/bin/env python3
"""Generate Gazebo Classic resources from the portable CN V3 draft templates."""

import argparse
import copy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent


def generate(output):
    source = HERE / 'source'
    models = output / 'models'
    worlds = output / 'worlds'
    models.mkdir(parents=True, exist_ok=True)
    worlds.mkdir(parents=True, exist_ok=True)

    for template in sorted((source / 'models').iterdir()):
        target = models / template.name
        shutil.copytree(template, target, dirs_exist_ok=True)
        tree = ET.parse(template / 'model.sdf')
        # The template geometry and ODE surface fields are supported by SDF 1.6.
        tree.getroot().set('version', '1.6')
        write_xml(tree, target / 'model.sdf')
        config = ET.parse(template / 'model.config')
        config.getroot().find('sdf').set('version', '1.6')
        config.getroot().find('version').text = '3.0-draft'
        write_xml(config, target / 'model.config')

    tree = ET.parse(source / 'classic_world_base.world')
    world = tree.getroot().find('world')
    world.set('name', 'robocon_cn_v3_draft')
    # Import only model initial poses; Fortress system plugins are not Classic plugins.
    initial = ET.parse(source / 'initial_objects.world').getroot().find('world')
    for include in initial.findall('include'):
        world.append(copy.deepcopy(include))
    write_xml(tree, worlds / 'robocon_cn_v3_draft.world')

    world.set('name', 'robocon_cn_v3_draft_field_only')
    for include in list(world.findall('include'))[1:]:
        world.remove(include)
    write_xml(tree, worlds / 'robocon_cn_v3_draft_field_only.world')
    print(f'Generated Classic V3 draft resources: {output}')


def write_xml(tree, path):
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE)
    generate(parser.parse_args().output)
