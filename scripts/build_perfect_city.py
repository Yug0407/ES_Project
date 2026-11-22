#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import subprocess

# 1. Reset duckietown_city.sdf from git
subprocess.run(['git', 'checkout', 'HEAD', '--', 'worlds/duckietown_city.sdf'], check=True)

# 2. Scale by 0.2
scale = 0.2
tree = ET.parse('worlds/duckietown_city.sdf')
root = tree.getroot()
world = root.find('world')

# Configure camera
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

for model in world.findall('model'):
    name = model.get('name')
    if name == 'ground_plane':
        continue
    
    pose_elem = model.find('pose')
    if pose_elem is not None and pose_elem.text:
        parts = [float(p) for p in pose_elem.text.strip().split()]
        if len(parts) >= 3:
            parts[0] *= scale
            parts[1] *= scale
            parts[2] *= scale
            pose_elem.text = ' '.join(f'{x:.4f}' if i < 3 else str(x) for i, x in enumerate(parts))

    for link in model.findall('link'):
        link_pose = link.find('pose')
        if link_pose is not None and link_pose.text:
            parts = [float(p) for p in link_pose.text.strip().split()]
            if len(parts) >= 3:
                parts[0] *= scale
                parts[1] *= scale
                parts[2] *= scale
                link_pose.text = ' '.join(f'{x:.4f}' if i < 3 else str(x) for i, x in enumerate(parts))

        for vc in list(link.findall('visual')) + list(link.findall('collision')):
            vc_pose = vc.find('pose')
            if vc_pose is not None and vc_pose.text:
                parts = [float(p) for p in vc_pose.text.strip().split()]
                if len(parts) >= 3:
                    parts[0] *= scale
                    parts[1] *= scale
                    parts[2] *= scale
                    vc_pose.text = ' '.join(f'{x:.4f}' if i < 3 else str(x) for i, x in enumerate(parts))

            geom = vc.find('geometry')
            if geom is not None:
                box = geom.find('box')
                if box is not None:
                    size = box.find('size')
                    if size is not None and size.text:
                        s = [float(x) for x in size.text.strip().split()]
                        if len(s) == 3:
                            s[0] *= scale
                            s[1] *= scale
                            if s[2] > 0.05:
                                s[2] *= scale
                            size.text = f'{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}'
                cyl = geom.find('cylinder')
                if cyl is not None:
                    rad = cyl.find('radius')
                    length = cyl.find('length')
                    if rad is not None and rad.text:
                        rad.text = f'{float(rad.text)*scale:.4f}'
                    if length is not None and length.text:
                        length.text = f'{float(length.text)*scale:.4f}'

# Realistic low-poly city car
# Styled like a modern sedan with bumpers, roof, front windshield, rear windshield, windows, wheels
car_xml = """
    <model name="traffic_car_blue">
      <pose>0.8 0.38 0.01 0 0 3.14159</pose>
      <link name="chassis">
        <inertial>
          <mass>30.0</mass>
          <inertia>
            <ixx>0.5</ixx><iyy>0.5</iyy><izz>0.5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="col">
          <pose>0 0 0.07 0 0 0</pose>
          <geometry><box><size>0.48 0.24 0.12</size></box></geometry>
        </collision>

        <!-- Lower Car Body -->
        <visual name="body">
          <pose>0 0 0.045 0 0 0</pose>
          <geometry><box><size>0.48 0.24 0.065</size></box></geometry>
          <material>
            <ambient>0.08 0.35 0.85 1</ambient>
            <diffuse>0.12 0.45 0.95 1</diffuse>
            <specular>0.7 0.7 0.8 1</specular>
          </material>
        </visual>

        <!-- Front Bumper -->
        <visual name="front_bumper">
          <pose>0.241 0 0.035 0 0 0</pose>
          <geometry><box><size>0.015 0.24 0.04</size></box></geometry>
          <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>

        <!-- Rear Bumper -->
        <visual name="rear_bumper">
          <pose>-0.241 0 0.035 0 0 0</pose>
          <geometry><box><size>0.015 0.24 0.04</size></box></geometry>
          <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>

        <!-- Cabin / Roof -->
        <visual name="cabin">
          <pose>-0.03 0 0.10 0 0 0</pose>
          <geometry><box><size>0.26 0.20 0.05</size></box></geometry>
          <material>
            <ambient>0.05 0.08 0.15 0.95</ambient>
            <diffuse>0.08 0.12 0.22 0.95</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>

        <!-- Windshield (Tilted visual effect) -->
        <visual name="windshield">
          <pose>0.11 0 0.08 0 0.45 0</pose>
          <geometry><box><size>0.01 0.19 0.06</size></box></geometry>
          <material><ambient>0.7 0.85 0.95 0.9</ambient><diffuse>0.8 0.9 1.0 0.9</diffuse></material>
        </visual>

        <!-- Rear Window -->
        <visual name="rear_window">
          <pose>-0.17 0 0.08 0 -0.45 0</pose>
          <geometry><box><size>0.01 0.19 0.06</size></box></geometry>
          <material><ambient>0.1 0.15 0.25 0.9</ambient><diffuse>0.15 0.2 0.3 0.9</diffuse></material>
        </visual>

        <!-- Headlights -->
        <visual name="hl_l">
          <pose>0.245 0.08 0.05 0 0 0</pose>
          <geometry><box><size>0.008 0.05 0.025</size></box></geometry>
          <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
        </visual>
        <visual name="hl_r">
          <pose>0.245 -0.08 0.05 0 0 0</pose>
          <geometry><box><size>0.008 0.05 0.025</size></box></geometry>
          <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
        </visual>

        <!-- Taillights -->
        <visual name="tl_l">
          <pose>-0.245 0.08 0.05 0 0 0</pose>
          <geometry><box><size>0.008 0.05 0.025</size></box></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.95 0.05 0.05 1</diffuse></material>
        </visual>
        <visual name="tl_r">
          <pose>-0.245 -0.08 0.05 0 0 0</pose>
          <geometry><box><size>0.008 0.05 0.025</size></box></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.95 0.05 0.05 1</diffuse></material>
        </visual>

        <!-- Wheels with Rims -->
        <visual name="wheel_fl"><pose>0.14 0.122 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_fr"><pose>0.14 -0.122 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_rl"><pose>-0.14 0.122 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_rr"><pose>-0.14 -0.122 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
    </model>
"""

# Red SUV / Van
suv_xml = """
    <model name="traffic_suv_red">
      <pose>-2.5 3.0 0.01 0 0 0</pose>
      <link name="chassis">
        <inertial>
          <mass>35.0</mass>
          <inertia>
            <ixx>0.6</ixx><iyy>0.6</iyy><izz>0.6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="col">
          <pose>0 0 0.08 0 0 0</pose>
          <geometry><box><size>0.46 0.23 0.15</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>0.46 0.23 0.07</size></box></geometry>
          <material><ambient>0.85 0.15 0.1 1</ambient><diffuse>0.95 0.2 0.15 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.02 0 0.11 0 0 0</pose>
          <geometry><box><size>0.28 0.21 0.065</size></box></geometry>
          <material><ambient>0.1 0.1 0.12 0.95</ambient><diffuse>0.15 0.15 0.2 0.95</diffuse></material>
        </visual>
        <visual name="hl_l"><pose>0.235 0.075 0.05 0 0 0</pose><geometry><box><size>0.008 0.05 0.025</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="hl_r"><pose>0.235 -0.075 0.05 0 0 0</pose><geometry><box><size>0.008 0.05 0.025</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="wheel_fl"><pose>0.13 0.118 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_fr"><pose>0.13 -0.118 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_rl"><pose>-0.13 0.118 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_rr"><pose>-0.13 -0.118 0.025 1.5708 0 0</pose><geometry><cylinder><radius>0.038</radius><length>0.022</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
    </model>
"""

# Pedestrian waiting on the sidewalk (Clean human proportions: legs, jacket, head, cap)
pedestrian_standing_xml = """
    <model name="pedestrian_waiting">
      <pose>-2.2 0.05 0.0 0 0 1.5708</pose>
      <static>true</static>
      <link name="body">
        <collision name="col">
          <pose>0 0 0.17 0 0 0</pose>
          <geometry><cylinder><radius>0.06</radius><length>0.34</length></cylinder></geometry>
        </collision>
        <!-- Jeans / Legs -->
        <visual name="legs">
          <pose>0 0 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.045</radius><length>0.14</length></cylinder></geometry>
          <material><ambient>0.1 0.18 0.35 1</ambient><diffuse>0.15 0.25 0.45 1</diffuse></material>
        </visual>
        <!-- Jacket / Torso -->
        <visual name="torso">
          <pose>0 0 0.19 0 0 0</pose>
          <geometry><cylinder><radius>0.055</radius><length>0.12</length></cylinder></geometry>
          <material><ambient>0.15 0.55 0.25 1</ambient><diffuse>0.2 0.65 0.3 1</diffuse></material>
        </visual>
        <!-- Head -->
        <visual name="head">
          <pose>0 0 0.28 0 0 0</pose>
          <geometry><sphere><radius>0.04</radius></sphere></geometry>
          <material><ambient>0.9 0.7 0.55 1</ambient><diffuse>0.95 0.75 0.6 1</diffuse></material>
        </visual>
        <!-- Cap / Hat -->
        <visual name="cap">
          <pose>0 0 0.315 0 0 0</pose>
          <geometry><cylinder><radius>0.042</radius><length>0.018</length></cylinder></geometry>
          <material><ambient>0.8 0.2 0.1 1</ambient><diffuse>0.9 0.25 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

for m_str in [car_xml, suv_xml, pedestrian_standing_xml]:
    world.append(ET.fromstring(m_str))

tree.write('worlds/duckietown_city.sdf', encoding='utf-8', xml_declaration=True)
print('Successfully generated clean city models!')
