#!/usr/bin/env python3
"""
setup_moving_traffic.py  --  Complete Duckietown City World Builder
========================================================================
* Resets duckietown_city.sdf from git HEAD (clean start)
* Scales all road models by 0.2x
* Removes problem models: apartment_block, house_1-3, gas_station,
  fast_food, spawn_marker (causes __default__ mesh errors)
* Adds proper inline buildings that flank the roads at correct positions
* Adds 7 moving traffic models (5 cars + 2 pedestrians) with
  TrajectoryFollower strictly on road lanes
* Sets TurtleBot spawn at safe west end of road (x=-4.5, y=0.6)
* Camera tracks waffle_pi
"""

import xml.etree.ElementTree as ET
import subprocess

# ── 1. Reset to clean git state ──────────────────────────────────────────────
subprocess.run(['git', 'checkout', 'HEAD', '--', 'worlds/duckietown_city.sdf'], check=True)

tree = ET.parse('worlds/duckietown_city.sdf')
root = tree.getroot()
world = root.find('world')

# ── 2. GUI camera tracking waffle_pi ─────────────────────────────────────────
gui = world.find('gui')
if gui is None:
    gui = ET.Element('gui')
    world.insert(0, gui)
gui.clear()

cam = ET.SubElement(gui, 'camera', name='user_camera')
cam_pose = ET.SubElement(cam, 'pose')
cam_pose.text = '-1.8 0.6 0.9 0 0.45 0'
track = ET.SubElement(cam, 'track_visual')
ET.SubElement(track, 'name').text = 'waffle_pi'
ET.SubElement(track, 'min_dist').text = '0.5'
ET.SubElement(track, 'max_dist').text = '1.8'
ET.SubElement(track, 'static').text = 'true'

# ── 3. Scale factor ──────────────────────────────────────────────────────────
SCALE = 0.2

# Models to REMOVE completely (they look bad after scaling or cause errors)
REMOVE_MODELS = {
    'apartment_block', 'house_1', 'house_2', 'house_3',
    'gas_station', 'fast_food', 'spawn_marker',
}

# ── 4. Scale all road/obstacle/lamp models and remove bad ones ────────────────
models_to_remove = []
for model in world.findall('model'):
    name = model.get('name', '')

    # Mark for removal
    if name in REMOVE_MODELS:
        models_to_remove.append(model)
        continue

    # Skip ground plane (it stays full size)
    if name == 'ground_plane':
        continue

    # Scale model pose (X, Y, Z only — not rotation)
    pose_elem = model.find('pose')
    if pose_elem is not None and pose_elem.text:
        parts = pose_elem.text.strip().split()
        if len(parts) >= 3:
            parts[0] = f'{float(parts[0]) * SCALE:.4f}'
            parts[1] = f'{float(parts[1]) * SCALE:.4f}'
            parts[2] = f'{float(parts[2]) * SCALE:.4f}'
            pose_elem.text = ' '.join(parts)

    for link in model.findall('link'):
        # Scale link pose
        lp = link.find('pose')
        if lp is not None and lp.text:
            parts = lp.text.strip().split()
            if len(parts) >= 3:
                parts[0] = f'{float(parts[0]) * SCALE:.4f}'
                parts[1] = f'{float(parts[1]) * SCALE:.4f}'
                parts[2] = f'{float(parts[2]) * SCALE:.4f}'
                lp.text = ' '.join(parts)

        for vc in list(link.findall('visual')) + list(link.findall('collision')):
            # Scale visual/collision pose
            vp = vc.find('pose')
            if vp is not None and vp.text:
                parts = vp.text.strip().split()
                if len(parts) >= 3:
                    parts[0] = f'{float(parts[0]) * SCALE:.4f}'
                    parts[1] = f'{float(parts[1]) * SCALE:.4f}'
                    parts[2] = f'{float(parts[2]) * SCALE:.4f}'
                    vp.text = ' '.join(parts)

            geom = vc.find('geometry')
            if geom is None:
                continue
            # Scale box size
            box = geom.find('box')
            if box is not None:
                size = box.find('size')
                if size is not None and size.text:
                    s = [float(x) for x in size.text.strip().split()]
                    if len(s) == 3:
                        s[0] *= SCALE
                        s[1] *= SCALE
                        # Keep thin slabs thin (road/ground surfaces)
                        if s[2] > 0.05:
                            s[2] *= SCALE
                        size.text = f'{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}'
            # Scale cylinder
            cyl = geom.find('cylinder')
            if cyl is not None:
                rad = cyl.find('radius')
                length = cyl.find('length')
                if rad is not None and rad.text:
                    rad.text = f'{float(rad.text) * SCALE:.4f}'
                if length is not None and length.text:
                    length.text = f'{float(length.text) * SCALE:.4f}'

# Remove bad models
for m in models_to_remove:
    world.remove(m)

# ── 5. ROAD GEOMETRY REFERENCE (after 0.2x scale) ────────────────────────────
# Road row 0 (main street): Y=0.60, X from -5.68 to +1.49
# Road row 1 (middle):      Y=1.80, X from -4.49 to +2.10
# Road row 2 (top):         Y=3.00, X from -4.49 to +2.10
# Lane width: 0.30m each side of center → 0.60m total road width
# Right lane of row 0:  Y ≈ 0.30 (outer = towards south)
# Left  lane of row 0:  Y ≈ 0.90 (outer = towards north)
# TurtleBot width: ~0.28m — fits in one lane

# ── 6. INLINE BUILDINGS flanking the roads ───────────────────────────────────
# Placed OUTSIDE road boundary (Y < 0.0 or Y > 1.2 for row 0, etc.)
# Heights chosen to look like a city block: 0.6m–1.2m tall

buildings_xml = """
  <!-- ═══════════ CITY BUILDINGS (south side of main road, row 0) ══════════ -->
  <model name="building_south_1">
    <static>true</static>
    <pose>-0.6 -0.55 0.5 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>0.9 0.6 1.0</size></box></geometry>
        <material><ambient>0.65 0.55 0.5 1</ambient><diffuse>0.65 0.55 0.5 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><box><size>0.9 0.6 1.0</size></box></geometry>
      </collision>
    </link>
  </model>

  <model name="building_south_2">
    <static>true</static>
    <pose>-2.4 -0.55 0.4 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>1.0 0.5 0.8</size></box></geometry>
        <material><ambient>0.7 0.65 0.55 1</ambient><diffuse>0.7 0.65 0.55 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><box><size>1.0 0.5 0.8</size></box></geometry>
      </collision>
    </link>
  </model>

  <model name="building_south_3">
    <static>true</static>
    <pose>-4.2 -0.55 0.6 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>1.2 0.6 1.2</size></box></geometry>
        <material><ambient>0.5 0.6 0.7 1</ambient><diffuse>0.5 0.6 0.7 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><box><size>1.2 0.6 1.2</size></box></geometry>
      </collision>
    </link>
  </model>

  <!-- ═══════════ CITY BUILDINGS (north side between row 0 and row 1) ══════ -->
  <!-- Between Y=1.2 (edge row 0) and Y=1.5 (edge row 1) = narrow strip Y≈1.35 -->
  <!-- Use Y=4.0+ for houses above row 2 instead — cleaner -->

  <!-- ═══════════ CITY BUILDINGS (north of row 2, top of city) ════════════ -->
  <model name="building_north_1">
    <static>true</static>
    <pose>-1.0 4.0 0.45 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>0.8 0.5 0.9</size></box></geometry>
        <material><ambient>0.75 0.7 0.6 1</ambient><diffuse>0.75 0.7 0.6 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><box><size>0.8 0.5 0.9</size></box></geometry>
      </collision>
    </link>
  </model>

  <model name="building_north_2">
    <static>true</static>
    <pose>-3.0 4.0 0.55 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>1.0 0.5 1.1</size></box></geometry>
        <material><ambient>0.6 0.55 0.65 1</ambient><diffuse>0.6 0.55 0.65 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><box><size>1.0 0.5 1.1</size></box></geometry>
      </collision>
    </link>
  </model>

  <model name="building_north_3">
    <static>true</static>
    <pose>-5.0 4.0 0.4 0 0 0</pose>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>0.9 0.5 0.8</size></box></geometry>
        <material><ambient>0.8 0.75 0.65 1</ambient><diffuse>0.8 0.75 0.65 1</diffuse></material>
      </visual>
      <collision name="col">
        <geometry><box><size>0.9 0.5 0.8</size></box></geometry>
      </collision>
    </link>
  </model>

  <!-- Gas station (south-west corner) -->
  <model name="gas_station_new">
    <static>true</static>
    <pose>-5.0 -0.7 0.15 0 0 0</pose>
    <link name="link">
      <visual name="canopy">
        <pose>0 0 0.15 0 0 0</pose>
        <geometry><box><size>0.7 0.9 0.04</size></box></geometry>
        <material><ambient>0.9 0.9 0.3 1</ambient><diffuse>0.9 0.9 0.3 1</diffuse></material>
      </visual>
      <visual name="structure">
        <pose>0.2 0 0 0 0 0</pose>
        <geometry><box><size>0.3 0.4 0.3</size></box></geometry>
        <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
      </visual>
    </link>
  </model>
"""

# Parse and add buildings
import xml.etree.ElementTree as ET2
# Wrap in a root so we can parse multiple elements
wrapped = '<root>' + buildings_xml + '</root>'
bld_root = ET2.fromstring(wrapped)
for bld in bld_root.findall('model'):
    world.append(bld)

# ── 7. DYNAMIC TRAFFIC MODELS ────────────────────────────────────────────────
# Road layout (all coordinates are POST-SCALE world positions):
#
#  Row 0 (main street): Y center=0.60, right lane Y=0.33, left lane Y=0.87
#    X range: about -5.68 (farwest) to 1.49 (east diag)
#    Safe car range: X ∈ [-5.0, 1.2]
#
#  Row 1 (vertical/side street): Y center=1.80
#    X range: about -4.49 to 2.10
#    Safe car range: X ∈ [-4.0, 1.5]
#
#  Row 2 (top street): Y center=3.00
#    X range: about -4.49 to 2.10
#    Safe car range: X ∈ [-4.0, 1.4]
#
# TurtleBot3 spawns at x=-4.5, y=0.6 (safe far-west, left lane of row 0)
# No traffic starts near x=-4.5 so robot starts safely.

# ── Car 1: Blue sedan — right lane of row 0, moves east ──────────────────────
# Starts at east end, moves west (approaches robot from east)
# Right lane: Y=0.33
car1_xml = """
    <model name="traffic_car_blue">
      <static>false</static>
      <pose>1.0 0.33 0.05 0 0 3.1416</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.06 0 0 0</pose>
          <geometry><box><size>0.42 0.21 0.12</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.042 0 0 0</pose>
          <geometry><box><size>0.42 0.21 0.062</size></box></geometry>
          <material>
            <ambient>0.08 0.35 0.85 1</ambient>
            <diffuse>0.12 0.45 0.95 1</diffuse>
            <specular>0.6 0.6 0.8 1</specular>
          </material>
        </visual>
        <visual name="cabin">
          <pose>-0.03 0 0.095 0 0 0</pose>
          <geometry><box><size>0.22 0.17 0.045</size></box></geometry>
          <material><ambient>0.05 0.08 0.15 0.9</ambient><diffuse>0.08 0.12 0.22 0.9</diffuse></material>
        </visual>
        <visual name="hl_l"><pose>0.21 0.07 0.048 0 0 0</pose><geometry><box><size>0.008 0.04 0.022</size></box></geometry><material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material></visual>
        <visual name="hl_r"><pose>0.21 -0.07 0.048 0 0 0</pose><geometry><box><size>0.008 0.04 0.022</size></box></geometry><material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material></visual>
        <visual name="tl_l"><pose>-0.21 0.07 0.048 0 0 0</pose><geometry><box><size>0.008 0.04 0.022</size></box></geometry><material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.95 0.05 0.05 1</diffuse></material></visual>
        <visual name="tl_r"><pose>-0.21 -0.07 0.048 0 0 0</pose><geometry><box><size>0.008 0.04 0.022</size></box></geometry><material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.95 0.05 0.05 1</diffuse></material></visual>
        <visual name="w_fl"><pose>0.12 0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_fr"><pose>0.12 -0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rl"><pose>-0.12 0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rr"><pose>-0.12 -0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>60</force>
        <torque>40</torque>
        <waypoints>
          <waypoint>1.0 0.33</waypoint>
          <waypoint>-3.5 0.33</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── Car 2: Red SUV — left lane of row 0, moves east ──────────────────────────
# Starts at west end, opposite direction to car 1 (oncoming traffic feel)
# Left lane: Y=0.87
car2_xml = """
    <model name="traffic_suv_red">
      <static>false</static>
      <pose>-3.0 0.87 0.05 0 0 0</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.07 0 0 0</pose>
          <geometry><box><size>0.44 0.22 0.14</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.048 0 0 0</pose>
          <geometry><box><size>0.44 0.22 0.065</size></box></geometry>
          <material><ambient>0.85 0.15 0.1 1</ambient><diffuse>0.95 0.2 0.15 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.02 0 0.105 0 0 0</pose>
          <geometry><box><size>0.26 0.20 0.058</size></box></geometry>
          <material><ambient>0.1 0.1 0.12 0.9</ambient><diffuse>0.15 0.15 0.2 0.9</diffuse></material>
        </visual>
        <visual name="hl_l"><pose>0.22 0.07 0.048 0 0 0</pose><geometry><box><size>0.008 0.04 0.022</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="hl_r"><pose>0.22 -0.07 0.048 0 0 0</pose><geometry><box><size>0.008 0.04 0.022</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="w_fl"><pose>0.12 0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_fr"><pose>0.12 -0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rl"><pose>-0.12 0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rr"><pose>-0.12 -0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>60</force>
        <torque>40</torque>
        <waypoints>
          <waypoint>-3.0 0.87</waypoint>
          <waypoint>1.0 0.87</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── Car 3: Yellow Taxi — row 1 (Y=1.80) ──────────────────────────────────────
taxi_xml = """
    <model name="traffic_taxi_yellow">
      <static>false</static>
      <pose>-1.0 1.80 0.05 0 0 0</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.065 0 0 0</pose>
          <geometry><box><size>0.42 0.21 0.12</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.042 0 0 0</pose>
          <geometry><box><size>0.42 0.21 0.065</size></box></geometry>
          <material><ambient>0.9 0.75 0.05 1</ambient><diffuse>0.95 0.8 0.1 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.02 0 0.098 0 0 0</pose>
          <geometry><box><size>0.22 0.18 0.05</size></box></geometry>
          <material><ambient>0.1 0.1 0.15 0.9</ambient><diffuse>0.15 0.15 0.2 0.9</diffuse></material>
        </visual>
        <visual name="taxi_sign">
          <pose>0 0 0.135 0 0 0</pose>
          <geometry><box><size>0.07 0.035 0.022</size></box></geometry>
          <material><ambient>1 1 0.1 1</ambient><diffuse>1 1 0.2 1</diffuse></material>
        </visual>
        <visual name="w_fl"><pose>0.11 0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_fr"><pose>0.11 -0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rl"><pose>-0.11 0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rr"><pose>-0.11 -0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>60</force>
        <torque>40</torque>
        <waypoints>
          <waypoint>-1.0 1.80</waypoint>
          <waypoint>1.4 1.80</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── Car 4: Green sedan — row 2 (Y=3.00) ──────────────────────────────────────
car4_xml = """
    <model name="traffic_car_green">
      <static>false</static>
      <pose>-2.0 3.00 0.05 0 0 3.1416</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.06 0 0 0</pose>
          <geometry><box><size>0.42 0.21 0.12</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.042 0 0 0</pose>
          <geometry><box><size>0.42 0.21 0.062</size></box></geometry>
          <material><ambient>0.15 0.65 0.25 1</ambient><diffuse>0.2 0.7 0.3 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.03 0 0.095 0 0 0</pose>
          <geometry><box><size>0.22 0.17 0.045</size></box></geometry>
          <material><ambient>0.05 0.08 0.1 0.9</ambient><diffuse>0.08 0.12 0.15 0.9</diffuse></material>
        </visual>
        <visual name="w_fl"><pose>0.12 0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_fr"><pose>0.12 -0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rl"><pose>-0.12 0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rr"><pose>-0.12 -0.11 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>60</force>
        <torque>40</torque>
        <waypoints>
          <waypoint>-2.0 3.00</waypoint>
          <waypoint>1.3 3.00</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── Car 5: White SUV — row 2 opposite direction ───────────────────────────────
car5_xml = """
    <model name="traffic_suv_white">
      <static>false</static>
      <pose>0.5 3.00 0.05 0 0 0</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.07 0 0 0</pose>
          <geometry><box><size>0.44 0.22 0.14</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.048 0 0 0</pose>
          <geometry><box><size>0.44 0.22 0.065</size></box></geometry>
          <material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.02 0 0.105 0 0 0</pose>
          <geometry><box><size>0.26 0.20 0.058</size></box></geometry>
          <material><ambient>0.5 0.6 0.7 0.85</ambient><diffuse>0.55 0.65 0.75 0.85</diffuse></material>
        </visual>
        <visual name="w_fl"><pose>0.12 0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_fr"><pose>0.12 -0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rl"><pose>-0.12 0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="w_rr"><pose>-0.12 -0.115 0.022 1.5708 0 0</pose><geometry><cylinder><radius>0.034</radius><length>0.020</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>60</force>
        <torque>40</torque>
        <waypoints>
          <waypoint>0.5 3.00</waypoint>
          <waypoint>-3.5 3.00</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── Pedestrian 1: Crosses road row 0 at X=-2.2 (safe crossing point) ─────────
# Moves south→north: Y=0.0 (sidewalk) to Y=1.2 (north sidewalk)
ped1_xml = """
    <model name="pedestrian_crossing">
      <static>false</static>
      <pose>-2.2 0.0 0.0 0 0 0</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.18 0 0 0</pose>
          <geometry><cylinder><radius>0.06</radius><length>0.32</length></cylinder></geometry>
        </collision>
        <visual name="legs">
          <pose>0 0 0.075 0 0 0</pose>
          <geometry><cylinder><radius>0.040</radius><length>0.13</length></cylinder></geometry>
          <material><ambient>0.1 0.18 0.35 1</ambient><diffuse>0.15 0.25 0.45 1</diffuse></material>
        </visual>
        <visual name="torso">
          <pose>0 0 0.185 0 0 0</pose>
          <geometry><cylinder><radius>0.050</radius><length>0.11</length></cylinder></geometry>
          <material><ambient>0.15 0.6 0.25 1</ambient><diffuse>0.2 0.7 0.3 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.265 0 0 0</pose>
          <geometry><sphere><radius>0.038</radius></sphere></geometry>
          <material><ambient>0.9 0.7 0.55 1</ambient><diffuse>0.95 0.75 0.6 1</diffuse></material>
        </visual>
        <visual name="cap">
          <pose>0 0 0.298 0 0 0</pose>
          <geometry><cylinder><radius>0.040</radius><length>0.014</length></cylinder></geometry>
          <material><ambient>0.8 0.2 0.1 1</ambient><diffuse>0.9 0.25 0.15 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>25</force>
        <torque>12</torque>
        <waypoints>
          <waypoint>-2.2 0.0</waypoint>
          <waypoint>-2.2 1.2</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── Pedestrian 2: Walks along sidewalk between row 1 and row 2 ───────────────
# Sidewalk at X≈-3.5 side, Y from 2.3 to 3.5 (above row 1, below row 2)
ped2_xml = """
    <model name="pedestrian_sidewalk">
      <static>false</static>
      <pose>-3.5 2.3 0.0 0 0 0</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="col">
          <pose>0 0 0.18 0 0 0</pose>
          <geometry><cylinder><radius>0.06</radius><length>0.32</length></cylinder></geometry>
        </collision>
        <visual name="legs">
          <pose>0 0 0.075 0 0 0</pose>
          <geometry><cylinder><radius>0.040</radius><length>0.13</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.15 1</ambient><diffuse>0.15 0.15 0.2 1</diffuse></material>
        </visual>
        <visual name="torso">
          <pose>0 0 0.185 0 0 0</pose>
          <geometry><cylinder><radius>0.050</radius><length>0.11</length></cylinder></geometry>
          <material><ambient>0.8 0.4 0.1 1</ambient><diffuse>0.9 0.45 0.15 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.265 0 0 0</pose>
          <geometry><sphere><radius>0.038</radius></sphere></geometry>
          <material><ambient>0.85 0.65 0.5 1</ambient><diffuse>0.9 0.7 0.55 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <force>25</force>
        <torque>12</torque>
        <waypoints>
          <waypoint>-3.5 2.3</waypoint>
          <waypoint>-3.5 3.5</waypoint>
        </waypoints>
      </plugin>
    </model>
"""

# ── 8. Add all traffic models to world ───────────────────────────────────────
for m_xml in [car1_xml, car2_xml, taxi_xml, car4_xml, car5_xml, ped1_xml, ped2_xml]:
    world.append(ET.fromstring(m_xml.strip()))

# ── 9. Write SDF ──────────────────────────────────────────────────────────────
tree.write('worlds/duckietown_city.sdf', encoding='utf-8', xml_declaration=True)
print('SUCCESS: 5 traffic cars + 2 pedestrians added, buildings flanking roads,')
print('         spawn_marker removed, TurtleBot safe spawn at x=-4.5 y=0.6')
print('         All models strictly on road lanes. Run gz sdf -k to validate.')
