#!/usr/bin/env python3
"""Generate the M5 fixed workcell with reproducibly randomized objects."""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


COLORS = ('red', 'blue', 'yellow')


def set_pose(model, x, y, z, yaw=0.):
    model.find('pose').text = f'{x:.9f} {y:.9f} {z:.9f} 0 0 {yaw:.9f}'


def table(template, name, center, size, top, yaw=0.):
    model = deepcopy(template)
    model.set('name', name)
    set_pose(model, *center, top/2, yaw)
    for geometry in model.findall('.//box/size'):
        geometry.text = f'{size[0]} {size[1]} {top}'
    return model


def overlaps(a, a_size, b, b_size, clearance=0.):
    return all(abs(x-y) < (w+h)/2+clearance for x, y, w, h in zip(a, b, a_size, b_size))


def generate(seed, settings, autonomy, gazebo):
    rng = random.Random(seed)
    scene = settings['m5']
    tree = ET.parse(autonomy / 'worlds' / 'vision_table.sdf')
    world = tree.find('world')
    template = world.find("model[@name='vision_table']")
    world.remove(template)
    top = scene['table_top_m']
    pickup = scene['pickup_table_world_xy']
    pickup_size = scene['pickup_table_size_xy']
    world.append(table(template, 'm5_pickup_table', pickup, pickup_size, top))
    for color in COLORS:
        zone = scene['drop_zones'][color]
        world.append(table(template, f'm5_drop_{color}', zone['table_world_xy'],
                           scene['drop_table_size_xy'], top, zone['dock_world'][2]))

    lanes = scene['cube_lanes_y_m'].copy()
    rng.shuffle(lanes)
    cubes = {}
    for color, lane in zip(COLORS, lanes):
        model = world.find(f"model[@name='m3_{color}_cube']")
        size = float(model.findtext('.//collision/geometry/box/size').split()[0])
        x = round(rng.uniform(*scene['cube_x_range_m']), 6)
        y = round(lane+rng.uniform(-scene['cube_y_jitter_m'], scene['cube_y_jitter_m']), 6)
        if any(abs(v-c)+size/2 >= w/2 for v,c,w in zip((x,y), pickup, pickup_size)):
            raise ValueError(f'{color} cube does not fit on the pickup table')
        if any(overlaps((x,y), (size,size), other['spawn_world_m'][:2],
                        (other['size_m'],)*2, clearance=.012) for other in cubes.values()):
            raise ValueError('Spawned cubes overlap or have insufficient spacing')
        z = round(top+size/2+.001, 6)
        set_pose(model, x, y, z)
        ET.SubElement(model, 'static').text = 'false'
        ET.SubElement(model.find('link'), 'gravity').text = 'true'
        dx, dy = scene['cube_from_pickup_dock_xy']
        dock = (round(x-dx, 6), round(y-dy, 6), 0.)
        front = pickup[0]-pickup_size[0]/2
        if dock[0]+.215 >= front:
            raise ValueError(f'{color} nominal pickup footprint overlaps the table')
        cubes[color] = {'model': model.get('name'), 'spawn_world_m': [x,y,z],
                        'size_m': size, 'pickup_dock_hint_world': dock}

    room = ET.parse(gazebo / 'worlds' / 'slam_room.sdf')
    for wall in room.findall('world/model'):
        if wall.get('name').endswith('_wall'):
            world.append(wall)
    obstacle = ET.parse(autonomy / 'models' / 'm2_obstacle.sdf').find('model')
    size = [float(v) for v in obstacle.findtext('.//collision/geometry/box/size').split()]
    limits = scene['clear_transfer_xy']
    clear_center = [(a+b)/2 for a,b in limits]
    clear_size = [b-a for a,b in limits]
    jitter = scene['obstacle_jitter_m']
    obstacles = []
    for i, slot in enumerate(rng.sample(scene['obstacle_slots_world_xy'], scene['obstacle_count']), 1):
        xy = [round(v+rng.uniform(-jitter,jitter), 6) for v in slot]
        if overlaps(xy,size,clear_center,clear_size, clearance=.02):
            raise ValueError('Obstacle intersects the reserved transfer space')
        if any(overlaps(xy,size,r['center_world_m'][:2],size,clearance=.04) for r in obstacles):
            raise ValueError('Obstacle envelopes overlap')
        if not (-2.45+size[0]/2 < xy[0] < 3.45-size[0]/2 and abs(xy[1])+size[1]/2 < 2.45):
            raise ValueError('Obstacle intersects a room wall')
        model = deepcopy(obstacle)
        model.set('name', f'm5_obstacle_{i}')
        set_pose(model, *xy, size[2]/2)
        world.append(model)
        obstacles.append({'model': model.get('name'), 'center_world_m': [*xy,size[2]/2],
                          'size_m': size})

    ET.indent(tree, space='  ')
    manifest = {'seed': seed, 'observation_world': scene['observation_world'],
                'table_top_m': top, 'clear_transfer_xy': limits,
                'drop_zones': scene['drop_zones'], 'cubes': cubes, 'obstacles': obstacles}
    return tree, manifest


def main():
    autonomy = Path(get_package_share_directory('mobile_manipulator_autonomy'))
    gazebo = Path(get_package_share_directory('mobile_manipulator_gazebo'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=51)
    parser.add_argument('--config', type=Path, default=autonomy/'config'/'workcells.yaml')
    parser.add_argument('--sdf', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    world, manifest = generate(args.seed, yaml.safe_load(args.config.read_text()), autonomy, gazebo)
    with args.sdf.open('xb') as output:
        world.write(output, encoding='utf-8', xml_declaration=True)
    with args.manifest.open('x') as output:
        json.dump(manifest, output, indent=2)
        output.write('\n')


if __name__ == '__main__':
    main()
