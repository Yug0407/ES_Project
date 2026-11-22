#!/usr/bin/env python3
"""
add_moving_traffic.py  --  put clean traffic models into worlds/duckietown_city.sdf
=============================================================================
Run from the project folder:      python3 add_moving_traffic.py

What it does (safe to run again and again):
  1. Saves a backup:  worlds/duckietown_city.sdf.bak
  2. Deletes the OLD traffic (TrajectoryFollower cars / pedestrians / actors)
  3. Adds 5 cars + 2 pedestrians as plain kinematic models -- NO motion plugin.
     They are moved by traffic_controller.py (started by the launch file),
     which puts them on the real lanes at the right speed.
  4. Moves the apartment block 0.6 m west so it stops poking into the road corner.

Nothing else in your world (roads, lamps, signs, barriers, camera) is touched.
"""
import math
import os
import shutil
import xml.etree.ElementTree as ET

from traffic_sim import CARS, PEDESTRIANS, TrafficSim

WORLD = 'worlds/duckietown_city.sdf'


def f(v):
    return f'{v:.4f}'


def box_visual(name, pose, size, rgb, a=1.0):
    r, g, b = rgb
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
            f'<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>'
            f'<specular>0.3 0.3 0.3 1</specular></material></visual>')


def wheel(name, x, y):
    return (f'<visual name="{name}"><pose>{x} {y} 0.045 1.5708 0 0</pose><geometry><cylinder>'
            f'<radius>0.045</radius><length>0.035</length></cylinder></geometry>'
            f'<material><ambient>0.04 0.04 0.04 1</ambient><diffuse>0.07 0.07 0.07 1</diffuse></material></visual>')


def car_sdf(cfg, pose):
    """Car faces +X.  Size about 0.62 x 0.30 m (a 1:6 scale city car)."""
    rgb = tuple(f(c) for c in cfg['color'])
    glass = (0.08, 0.10, 0.14)
    tall = cfg['style'] == 'suv'
    cab_h = 0.085 if tall else 0.065
    cab_len = 0.42 if tall else 0.34
    parts = [
        box_visual('body', '0 0 0.075 0 0 0', '0.62 0.30 0.07', rgb),
        box_visual('cabin', f'-0.04 0 {0.110 + cab_h / 2:.3f} 0 0 0', f'{cab_len} 0.26 {cab_h}', glass, 0.95),
        box_visual('headlight_l', '0.311 0.095 0.080 0 0 0', '0.01 0.06 0.03', (1, 1, 0.8)),
        box_visual('headlight_r', '0.311 -0.095 0.080 0 0 0', '0.01 0.06 0.03', (1, 1, 0.8)),
        box_visual('taillight_l', '-0.311 0.095 0.080 0 0 0', '0.01 0.06 0.03', (0.9, 0.05, 0.05)),
        box_visual('taillight_r', '-0.311 -0.095 0.080 0 0 0', '0.01 0.06 0.03', (0.9, 0.05, 0.05)),
        wheel('w_fl', 0.20, 0.15), wheel('w_fr', 0.20, -0.15),
        wheel('w_rl', -0.20, 0.15), wheel('w_rr', -0.20, -0.15),
    ]
    if cfg['style'] == 'taxi':
        parts.append(box_visual('taxi_sign', '-0.04 0 0.215 0 0 0', '0.10 0.05 0.04', (1, 1, 0.3)))
    return f'''
    <model name="{cfg['name']}">
      <static>false</static>
      <pose>{f(pose[0])} {f(pose[1])} {f(pose[2])} 0 0 {f(pose[3])}</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision">
          <pose>0 0 0.10 0 0 0</pose>
          <geometry><box><size>0.62 0.30 0.16</size></box></geometry>
        </collision>
        {''.join(parts)}
      </link>
    </model>'''


def pedestrian_sdf(cfg, pose):
    """Person about 0.30 m tall (1:6 scale), faces +X."""
    sr, sg, sb = cfg['shirt']
    return f'''
    <model name="{cfg['name']}">
      <static>false</static>
      <pose>{f(pose[0])} {f(pose[1])} {f(pose[2])} 0 0 {f(pose[3])}</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision">
          <pose>0 0 0.15 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.30</length></cylinder></geometry>
        </collision>
        <visual name="legs"><pose>0 0 0.065 0 0 0</pose><geometry><cylinder><radius>0.040</radius><length>0.13</length></cylinder></geometry>
          <material><ambient>0.10 0.15 0.35 1</ambient><diffuse>0.12 0.20 0.45 1</diffuse></material></visual>
        <visual name="torso"><pose>0 0 0.185 0 0 0</pose><geometry><cylinder><radius>0.052</radius><length>0.12</length></cylinder></geometry>
          <material><ambient>{sr} {sg} {sb} 1</ambient><diffuse>{sr} {sg} {sb} 1</diffuse></material></visual>
        <visual name="head"><pose>0 0 0.265 0 0 0</pose><geometry><sphere><radius>0.038</radius></sphere></geometry>
          <material><ambient>0.90 0.70 0.55 1</ambient><diffuse>0.95 0.75 0.60 1</diffuse></material></visual>
        <visual name="nose"><pose>0.036 0 0.265 0 0 0</pose><geometry><box><size>0.014 0.014 0.014</size></box></geometry>
          <material><ambient>0.8 0.55 0.4 1</ambient><diffuse>0.8 0.55 0.4 1</diffuse></material></visual>
      </link>
    </model>'''


def is_old_traffic(el):
    name = el.get('name', '')
    if el.tag == 'actor':
        return True
    if el.tag != 'model':
        return False
    if name.startswith(('npc_', 'pedestrian_')):
        return True
    if name.startswith('traffic_light'):          # keep the traffic lights!
        return False
    for plug in el.findall('plugin'):
        if 'trajectory-follower' in plug.get('filename', ''):
            return True
    return False


def main():
    if not os.path.exists(WORLD):
        raise SystemExit(f'Cannot find {WORLD} - run this from the project folder.')
    if not os.path.exists(WORLD + '.bak'):          # keep the FIRST backup safe
        shutil.copyfile(WORLD, WORLD + '.bak')

    tree = ET.parse(WORLD)
    world = tree.getroot().find('world')

    removed = [el.get('name') for el in list(world) if is_old_traffic(el)]
    for el in list(world):
        if is_old_traffic(el):
            world.remove(el)

    # the apartment overlaps the south-west road corner -> nudge it west
    for m in world.findall('model'):
        if m.get('name') == 'apartment_block':
            p = m.find('pose')
            v = p.text.split()
            if float(v[0]) > -7.2:
                v[0] = '-7.4000'
                p.text = ' '.join(v)

    poses = TrafficSim().initial_poses()
    for cfg in CARS:
        world.append(ET.fromstring(car_sdf(cfg, poses[cfg['name']])))
    for cfg in PEDESTRIANS:
        world.append(ET.fromstring(pedestrian_sdf(cfg, poses[cfg['name']])))

    tree.write(WORLD, encoding='utf-8', xml_declaration=True)
    print(f'Removed old traffic : {removed}')
    print(f'Added               : {len(CARS)} cars + {len(PEDESTRIANS)} pedestrians (no plugins)')
    print(f'Backup saved        : {WORLD}.bak')
    print('Next: ros2 launch launch/duckietown_world.launch.py')


if __name__ == '__main__':
    main()
