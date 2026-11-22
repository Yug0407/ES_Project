import xml.etree.ElementTree as ET
import subprocess

# Restore clean original duckietown_city.sdf
subprocess.run(['git', 'checkout', 'HEAD', '--', 'worlds/duckietown_city.sdf'], check=True)

tree = ET.parse('worlds/duckietown_city.sdf')
world = tree.getroot().find('world')

# Add camera zoomed in at the robot starting position (-4.5, 3.0)
gui = world.find('gui')
if gui is None:
    gui = ET.Element('gui')
    world.insert(0, gui)
gui.clear()
cam = ET.SubElement(gui, 'camera', name='user_camera')
cam_pose = ET.SubElement(cam, 'pose')
cam_pose.text = '-8.5 3.0 3.5 0 0.35 0'
track = ET.SubElement(cam, 'track_visual')
ET.SubElement(track, 'name').text = 'waffle_pi'
ET.SubElement(track, 'min_dist').text = '1.2'
ET.SubElement(track, 'max_dist').text = '4.0'
ET.SubElement(track, 'static').text = 'true'

# 1. City traffic patrol car: drives around the rectangular city road loop
# Loop stays strictly on the road:
# (-4.5, 1.8) -> (6.0, 1.8) -> (6.0, 14.0) -> (-15.0, 14.0) -> (-15.0, 1.8) -> (-4.5, 1.8)
car_actor_xml = """
    <actor name="city_traffic_car">
      <link name="link">
        <visual name="body">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><box><size>2.2 1.2 0.5</size></box></geometry>
          <material>
            <ambient>0.15 0.35 0.85 1</ambient>
            <diffuse>0.2 0.45 0.95 1</diffuse>
          </material>
        </visual>
        <visual name="cabin">
          <pose>-0.2 0 0.65 0 0 0</pose>
          <geometry><box><size>1.3 1.0 0.4</size></box></geometry>
          <material><ambient>0.1 0.1 0.15 0.9</ambient><diffuse>0.1 0.1 0.2 0.9</diffuse></material>
        </visual>
        <visual name="headlight_l"><pose>1.11 0.4 0.25 0 0 0</pose><geometry><box><size>0.05 0.2 0.12</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="headlight_r"><pose>1.11 -0.4 0.25 0 0 0</pose><geometry><box><size>0.05 0.2 0.12</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="city_patrol">
          <waypoint><time>0.0</time><pose>-4.5 1.8 0.1 0 0 0</pose></waypoint>
          <waypoint><time>6.0</time><pose>6.0 1.8 0.1 0 0 0</pose></waypoint>
          <waypoint><time>7.5</time><pose>6.0 1.8 0.1 0 0 1.5708</pose></waypoint>
          <waypoint><time>13.5</time><pose>6.0 14.0 0.1 0 0 1.5708</pose></waypoint>
          <waypoint><time>15.0</time><pose>6.0 14.0 0.1 0 0 3.14159</pose></waypoint>
          <waypoint><time>23.0</time><pose>-15.0 14.0 0.1 0 0 3.14159</pose></waypoint>
          <waypoint><time>24.5</time><pose>-15.0 14.0 0.1 0 0 -1.5708</pose></waypoint>
          <waypoint><time>30.5</time><pose>-15.0 1.8 0.1 0 0 -1.5708</pose></waypoint>
          <waypoint><time>32.0</time><pose>-15.0 1.8 0.1 0 0 0</pose></waypoint>
          <waypoint><time>38.0</time><pose>-4.5 1.8 0.1 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""

# 2. Crosswalk pedestrian: crosses the road between sidewalks at x = 0.0
ped1_actor_xml = """
    <actor name="crosswalk_pedestrian">
      <link name="link">
        <visual name="torso">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.25</radius><length>0.9</length></cylinder></geometry>
          <material><ambient>0.2 0.7 0.3 1</ambient><diffuse>0.25 0.8 0.35 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 1.35 0 0 0</pose>
          <geometry><sphere><radius>0.18</radius></sphere></geometry>
          <material><ambient>0.9 0.75 0.6 1</ambient><diffuse>0.95 0.8 0.65 1</diffuse></material>
        </visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="crossing">
          <waypoint><time>0.0</time><pose>0.0 0.2 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>6.0</time><pose>0.0 5.8 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>7.0</time><pose>0.0 5.8 0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>13.0</time><pose>0.0 0.2 0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>14.0</time><pose>0.0 0.2 0 0 0 1.5708</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""

# 3. Sidewalk walker: walks along the sidewalk next to the avenue
ped2_actor_xml = """
    <actor name="sidewalk_walker">
      <link name="link">
        <visual name="torso">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.25</radius><length>0.9</length></cylinder></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.9 0.25 0.25 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 1.35 0 0 0</pose>
          <geometry><sphere><radius>0.18</radius></sphere></geometry>
          <material><ambient>0.9 0.75 0.6 1</ambient><diffuse>0.95 0.8 0.65 1</diffuse></material>
        </visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint><time>0.0</time><pose>-16.0 3.0 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>10.0</time><pose>-16.0 12.0 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>11.0</time><pose>-16.0 12.0 0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>21.0</time><pose>-16.0 3.0 0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>22.0</time><pose>-16.0 3.0 0 0 0 1.5708</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""

# 4. Moving physical vehicle obstacle with real physical collision:
# Uses prismatic joint on the road with joint velocity controller
# Slides between -10.0 and 2.0 along the main road
obstacle_vehicle_xml = """
    <model name="patrol_obstacle_vehicle">
      <pose>-4.0 4.2 0.1 0 0 0</pose>
      <link name="base">
        <inertial><mass>500</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>
      <link name="chassis">
        <inertial><mass>100</mass><inertia><ixx>5</ixx><iyy>5</iyy><izz>5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision">
          <pose>0 0 0.35 0 0 0</pose>
          <geometry><box><size>1.6 0.8 0.5</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.35 0 0 0</pose>
          <geometry><box><size>1.6 0.8 0.4</size></box></geometry>
          <material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.95 0.55 0.15 1</diffuse></material>
        </visual>
        <visual name="cabin">
          <pose>-0.1 0 0.65 0 0 0</pose>
          <geometry><box><size>0.9 0.7 0.3</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <visual name="headlight_l"><pose>0.81 0.25 0.35 0 0 0</pose><geometry><box><size>0.04 0.15 0.1</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="headlight_r"><pose>0.81 -0.25 0.35 0 0 0</pose><geometry><box><size>0.04 0.15 0.1</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
      </link>
      <joint name="street_slide" type="prismatic">
        <parent>base</parent>
        <child>chassis</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit><lower>-4.0</lower><upper>4.0</upper><effort>1e6</effort></limit>
        </axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>street_slide</joint_name>
        <initial_velocity>0.8</initial_velocity>
      </plugin>
    </model>
"""

for s in [car_actor_xml, ped1_actor_xml, ped2_actor_xml, obstacle_vehicle_xml]:
    world.append(ET.fromstring(s))

tree.write('worlds/duckietown_city.sdf', encoding='utf-8', xml_declaration=True)
print('Successfully updated worlds/duckietown_city.sdf with exact city road loops!')
