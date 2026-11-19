#!/usr/bin/python3
"""Offline generated-model checks, independent of Gazebo installation."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

share=Path(get_package_share_directory('fr5_act_sim'))
def module(name):
    spec=importlib.util.spec_from_file_location(name,share/'launch'/f'{name}.py')
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
cfg=MoveItConfigsBuilder('fairino5_v6_robot',package_name='fairino5_v6_moveit2_config').to_moveit_configs()
g=yaml.safe_load((share/'config/gripper.yaml').read_text());scene=yaml.safe_load((share/'config/gazebo_scene.yaml').read_text())
u,s=module('gripper_model').add_gripper(cfg.robot_description['robot_description'],cfg.robot_description_semantic['robot_description_semantic'],g)
physics=module('gazebo_model');u,s=physics.build(u,s,scene,share);w=physics.world(scene)
root=ET.fromstring(u);ET.fromstring(s);world=ET.fromstring(w)
assert root.findtext('ros2_control/hardware/plugin')=='gz_ros2_control/GazeboSimSystem'
assert root.find("joint[@name='table_mount_joint']/parent").get('link')=='world'
assert scene['robot_mount_xyz'][2]==scene['table_top']
for link in root.findall('link'):
    if link.find('collision') is not None:
        assert float(link.find('inertial/mass').get('value'))>0,link.get('name')
for mesh in root.findall('.//mesh'):
    assert Path(mesh.get('filename').removeprefix('file://')).is_file()
assert root.find("gazebo/sensor[@name='wrist_rgb']/camera/image/width").text=='640'
assert world.find("world/model[@name='workbench']/static").text=='true'
assert world.find("world/model[@name='collection_bin']/static").text=='true'
assert len(world.findall("world/model[@name='collection_bin']/link"))==5
for i in range(1,6):assert world.find(f"world/model[@name='object_{i}']/static").text=='false'
Path('/tmp/fr5_gazebo_robot.urdf').write_text(u);Path('/tmp/fr5_gazebo_world.sdf').write_text(w)
print('PASS: table mounting, Gazebo hardware, positive masses, mesh paths, camera and dynamic cube; runtime not tested')
