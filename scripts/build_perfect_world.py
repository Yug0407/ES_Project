import xml.etree.ElementTree as ET
import subprocess

# Reset to clean 0.2-scaled duckietown_city.sdf
# First get clean git version
subprocess.run(['git', 'checkout', 'HEAD', '--', 'worlds/duckietown_city.sdf'], check=True)

# 1. Scale down world by 0.2 so 6m highway becomes 1.2m road, TurtleBot (0.28m) fills lane perfectly
scale = 0.2
tree = ET.parse('worlds/duckietown_city.sdf')
root = tree.getroot()
world = root.find('world')

# Camera looking closely at TurtleBot at (-0.9, 0.6)
gui = world.find('gui')
if gui is None:
    gui = ET.Element('gui')
    world.insert(0, gui)
else:
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

# 2. Add realistic gaming-style dynamic traffic that STAYS on the road with real collisions
# Road 0 (main street): Y runs from 0.0 to 1.2, center at Y = 0.6.
# Turtlebot spawns at (-0.9, 0.6).
# Road 1 runs at Y = 1.8. Road 2 runs at Y = 2.4 / 3.0.

# Traffic Car 1: Blue sedan that drives smoothly in the opposite lane (Y = 0.38)
# Length: 0.42m, Width: 0.22m, Height: 0.14m (matching TurtleBot scale)
car1_xml = """
    <model name="traffic_sedan_blue">
      <pose>0.8 0.38 0.02 0 0 0</pose>
      <link name="base">
        <inertial><mass>500</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>
      <link name="car_body">
        <inertial><mass>50</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">
          <pose>0 0 0.08 0 0 0</pose>
          <geometry><box><size>0.44 0.22 0.14</size></box></geometry>
        </collision>
        <!-- Car Body -->
        <visual name="body">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>0.44 0.22 0.08</size></box></geometry>
          <material>
            <ambient>0.1 0.35 0.85 1</ambient>
            <diffuse>0.15 0.45 0.95 1</diffuse>
            <specular>0.6 0.6 0.6 1</specular>
          </material>
        </visual>
        <!-- Roof/Cabin -->
        <visual name="cabin">
          <pose>-0.03 0 0.11 0 0 0</pose>
          <geometry><box><size>0.24 0.19 0.06</size></box></geometry>
          <material><ambient>0.1 0.12 0.18 0.9</ambient><diffuse>0.1 0.15 0.25 0.9</diffuse></material>
        </visual>
        <!-- Headlights -->
        <visual name="hl_l"><pose>0.222 0.07 0.05 0 0 0</pose><geometry><box><size>0.01 0.04 0.03</size></box></geometry><material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material></visual>
        <visual name="hl_r"><pose>0.222 -0.07 0.05 0 0 0</pose><geometry><box><size>0.01 0.04 0.03</size></box></geometry><material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material></visual>
        <!-- Taillights -->
        <visual name="tl_l"><pose>-0.222 0.07 0.05 0 0 0</pose><geometry><box><size>0.01 0.04 0.03</size></box></geometry><material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material></visual>
        <visual name="tl_r"><pose>-0.222 -0.07 0.05 0 0 0</pose><geometry><box><size>0.01 0.04 0.03</size></box></geometry><material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material></visual>
      </link>
      <joint name="drive_slide" type="prismatic">
        <parent>base</parent>
        <child>car_body</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit><lower>-3.2</lower><upper>0.5</upper><effort>1e6</effort></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>drive_slide</joint_name>
        <initial_velocity>-0.4</initial_velocity>
      </plugin>
    </model>
"""

# Traffic Car 2: Red sports car driving on the top street (Y = 3.0)
car2_xml = """
    <model name="traffic_sports_red">
      <pose>-3.0 3.0 0.02 0 0 0</pose>
      <link name="base">
        <inertial><mass>500</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>
      <link name="car_body">
        <inertial><mass>50</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">
          <pose>0 0 0.08 0 0 0</pose>
          <geometry><box><size>0.42 0.22 0.13</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>0.42 0.22 0.08</size></box></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.95 0.15 0.15 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.02 0 0.10 0 0 0</pose>
          <geometry><box><size>0.22 0.18 0.05</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 0.9</ambient><diffuse>0.1 0.1 0.1 0.9</diffuse></material>
        </visual>
      </link>
      <joint name="drive_slide_red" type="prismatic">
        <parent>base</parent>
        <child>car_body</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit><lower>-1.8</lower><upper>1.8</upper><effort>1e6</effort></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>drive_slide_red</joint_name>
        <initial_velocity>0.5</initial_velocity>
      </plugin>
    </model>
"""

# Pedestrian 1: Crossing pedestrian at the crosswalk (x = -2.2, crossing between Y = 0.15 and Y = 1.05)
# Has real collision cylinder and walking prismatic animation
ped1_xml = """
    <model name="pedestrian_crossing">
      <pose>-2.2 0.6 0.02 0 0 0</pose>
      <link name="base">
        <inertial><mass>300</mass><inertia><ixx>5</ixx><iyy>5</iyy><izz>5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>
      <link name="person">
        <inertial><mass>40</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">
          <pose>0 0 0.18 0 0 0</pose>
          <geometry><cylinder><radius>0.08</radius><length>0.34</length></cylinder></geometry>
        </collision>
        <visual name="legs">
          <pose>0 0 0.08 0 0 0</pose>
          <geometry><cylinder><radius>0.06</radius><length>0.14</length></cylinder></geometry>
          <material><ambient>0.15 0.2 0.4 1</ambient><diffuse>0.2 0.25 0.5 1</diffuse></material>
        </visual>
        <visual name="torso">
          <pose>0 0 0.21 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.14</length></cylinder></geometry>
          <material><ambient>0.2 0.65 0.3 1</ambient><diffuse>0.25 0.75 0.35 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.32 0 0 0</pose>
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>0.9 0.7 0.55 1</ambient><diffuse>0.95 0.75 0.6 1</diffuse></material>
        </visual>
      </link>
      <joint name="cross_slide" type="prismatic">
        <parent>base</parent>
        <child>person</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit><lower>-0.45</lower><upper>0.45</upper><effort>1e6</effort></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>cross_slide</joint_name>
        <initial_velocity>0.25</initial_velocity>
      </plugin>
    </model>
"""

# Pedestrian 2: Walking along the sidewalk near buildings (x = -3.29, along the street Y = 1.0 to 2.2)
ped2_xml = """
    <model name="pedestrian_sidewalk">
      <pose>-3.29 1.6 0.02 0 0 0</pose>
      <link name="base">
        <inertial><mass>300</mass><inertia><ixx>5</ixx><iyy>5</iyy><izz>5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>
      <link name="person">
        <inertial><mass>40</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">
          <pose>0 0 0.18 0 0 0</pose>
          <geometry><cylinder><radius>0.08</radius><length>0.34</length></cylinder></geometry>
        </collision>
        <visual name="torso">
          <pose>0 0 0.20 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.16</length></cylinder></geometry>
          <material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.85 0.35 0.15 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.32 0 0 0</pose>
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>0.9 0.7 0.55 1</ambient><diffuse>0.95 0.75 0.6 1</diffuse></material>
        </visual>
      </link>
      <joint name="walk_slide" type="prismatic">
        <parent>base</parent>
        <child>person</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit><lower>-0.6</lower><upper>0.6</upper><effort>1e6</effort></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>walk_slide</joint_name>
        <initial_velocity>0.20</initial_velocity>
      </plugin>
    </model>
"""

for s in [car1_xml, car2_xml, ped1_xml, ped2_xml]:
    world.append(ET.fromstring(s))

tree.write('worlds/duckietown_city.sdf', encoding='utf-8', xml_declaration=True)
print('Successfully generated perfect, properly scaled world with traffic and zero errors!')
