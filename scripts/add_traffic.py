import xml.etree.ElementTree as ET

tree = ET.parse('worlds/duckietown_city.sdf')
world = tree.getroot().find('world')

# Traffic vehicle 1: Blue sedan cruising east-west on row 0 (y=0.42, right lane)
car1_xml = """
    <model name="traffic_car_blue">
      <static>false</static>
      <pose>0.8 0.42 0.08 0 0 3.14159</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision">
          <geometry><box><size>0.48 0.24 0.16</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.04 0 0 0</pose>
          <geometry><box><size>0.48 0.24 0.10</size></box></geometry>
          <material>
            <ambient>0.1 0.35 0.85 1</ambient>
            <diffuse>0.15 0.45 0.95 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
        <visual name="cabin">
          <pose>-0.04 0 0.11 0 0 0</pose>
          <geometry><box><size>0.26 0.21 0.08</size></box></geometry>
          <material>
            <ambient>0.1 0.15 0.25 0.9</ambient>
            <diffuse>0.1 0.2 0.3 0.9</diffuse>
          </material>
        </visual>
        <visual name="headlight_left">
          <pose>0.242 0.08 0.04 0 0 0</pose>
          <geometry><box><size>0.01 0.05 0.03</size></box></geometry>
          <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
        </visual>
        <visual name="headlight_right">
          <pose>0.242 -0.08 0.04 0 0 0</pose>
          <geometry><box><size>0.01 0.05 0.03</size></box></geometry>
          <material><ambient>1 1 0.8 1</ambient><diffuse>1 1 0.8 1</diffuse></material>
        </visual>
        <visual name="taillight_left">
          <pose>-0.242 0.08 0.04 0 0 0</pose>
          <geometry><box><size>0.01 0.05 0.03</size></box></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
        <visual name="taillight_right">
          <pose>-0.242 -0.08 0.04 0 0 0</pose>
          <geometry><box><size>0.01 0.05 0.03</size></box></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
        <visual name="wheel_fl"><pose>0.14 0.125 0.0 1.5708 0 0</pose><geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_fr"><pose>0.14 -0.125 0.0 1.5708 0 0</pose><geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_rl"><pose>-0.14 0.125 0.0 1.5708 0 0</pose><geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
        <visual name="wheel_rr"><pose>-0.14 -0.125 0.0 1.5708 0 0</pose><geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <waypoints>
          <waypoint>0.8 0.42</waypoint>
          <waypoint>-2.8 0.42</waypoint>
          <waypoint>0.8 0.42</waypoint>
        </waypoints>
        <force>20</force>
        <torque>10</torque>
        <range_tolerance>0.1</range_tolerance>
      </plugin>
    </model>
"""

# Traffic vehicle 2: Red sports car patrolling row 2 (top boulevard, y=3.0)
car2_xml = """
    <model name="traffic_car_red">
      <static>false</static>
      <pose>-4.5 3.18 0.08 0 0 0</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision">
          <geometry><box><size>0.46 0.23 0.15</size></box></geometry>
        </collision>
        <visual name="body">
          <pose>0 0 0.04 0 0 0</pose>
          <geometry><box><size>0.46 0.23 0.10</size></box></geometry>
          <material>
            <ambient>0.85 0.1 0.1 1</ambient>
            <diffuse>0.95 0.15 0.15 1</diffuse>
            <specular>0.6 0.6 0.6 1</specular>
          </material>
        </visual>
        <visual name="cabin">
          <pose>-0.03 0 0.11 0 0 0</pose>
          <geometry><box><size>0.24 0.20 0.07</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 0.9</ambient><diffuse>0.1 0.1 0.1 0.9</diffuse></material>
        </visual>
        <visual name="headlight_left"><pose>0.232 0.08 0.04 0 0 0</pose><geometry><box><size>0.01 0.05 0.03</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
        <visual name="headlight_right"><pose>0.232 -0.08 0.04 0 0 0</pose><geometry><box><size>0.01 0.05 0.03</size></box></geometry><material><ambient>1 1 0.8 1</ambient></material></visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <waypoints>
          <waypoint>-4.5 3.18</waypoint>
          <waypoint>-1.2 3.18</waypoint>
          <waypoint>-4.5 3.18</waypoint>
        </waypoints>
        <force>22</force>
        <torque>10</torque>
        <range_tolerance>0.1</range_tolerance>
      </plugin>
    </model>
"""

# Pedestrian 1: Walking across the street (crosswalk pedestrian at x = -2.0)
ped1_xml = """
    <model name="pedestrian_crosswalk">
      <static>false</static>
      <pose>-2.0 0.1 0.18 0 0 1.5708</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision">
          <geometry><cylinder><radius>0.08</radius><length>0.36</length></cylinder></geometry>
        </collision>
        <visual name="torso">
          <pose>0 0 0 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.26</length></cylinder></geometry>
          <material><ambient>0.15 0.6 0.3 1</ambient><diffuse>0.2 0.7 0.35 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.18 0 0 0</pose>
          <geometry><sphere><radius>0.055</radius></sphere></geometry>
          <material><ambient>0.85 0.65 0.5 1</ambient><diffuse>0.9 0.7 0.55 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <waypoints>
          <waypoint>-2.0 0.05</waypoint>
          <waypoint>-2.0 1.15</waypoint>
          <waypoint>-2.0 0.05</waypoint>
        </waypoints>
        <force>12</force>
        <torque>6</torque>
        <range_tolerance>0.08</range_tolerance>
      </plugin>
    </model>
"""

# Pedestrian 2: Walking along the sidewalk near buildings (x = -3.29, y from 1.0 to 2.5)
ped2_xml = """
    <model name="pedestrian_sidewalk">
      <static>false</static>
      <pose>-3.29 1.1 0.18 0 0 1.5708</pose>
      <link name="link">
        <kinematic>true</kinematic>
        <gravity>false</gravity>
        <collision name="collision">
          <geometry><cylinder><radius>0.08</radius><length>0.36</length></cylinder></geometry>
        </collision>
        <visual name="torso">
          <pose>0 0 0 0 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.26</length></cylinder></geometry>
          <material><ambient>0.7 0.3 0.1 1</ambient><diffuse>0.85 0.35 0.15 1</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.18 0 0 0</pose>
          <geometry><sphere><radius>0.055</radius></sphere></geometry>
          <material><ambient>0.85 0.65 0.5 1</ambient><diffuse>0.9 0.7 0.55 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system" name="gz::sim::systems::TrajectoryFollower">
        <link_name>link</link_name>
        <loop>true</loop>
        <waypoints>
          <waypoint>-3.29 1.1</waypoint>
          <waypoint>-3.29 2.5</waypoint>
          <waypoint>-3.29 1.1</waypoint>
        </waypoints>
        <force>12</force>
        <torque>6</torque>
        <range_tolerance>0.08</range_tolerance>
      </plugin>
    </model>
"""

for m_xml in [car1_xml, car2_xml, ped1_xml, ped2_xml]:
    elem = ET.fromstring(m_xml)
    world.append(elem)

tree.write('worlds/duckietown_city.sdf', encoding='utf-8', xml_declaration=True)
print('Successfully added dynamic traffic models to duckietown_city.sdf')
