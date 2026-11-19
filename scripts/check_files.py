#!/usr/bin/python3
"""Checks that can run before ROS is installed."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'source_manifest.json').read_text())
for name, digest in manifest.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, name
for p in root.rglob('*.py'):
    ast.parse(p.read_text(), filename=str(p))
for p in (root / 'scripts').iterdir():
    if p.suffix in ('.sh', '.bash'):
        subprocess.run(['bash', '-n', str(p)], check=True)
for p in root.rglob('package.xml'):
    ET.parse(p)
model = ET.parse(root / 'src/fairino_description/urdf/fairino5_v6.urdf').getroot()
names = [j.attrib['name'] for j in model.findall('joint') if j.attrib['type'] == 'revolute']
assert names == [f'j{i}' for i in range(1, 7)], names
for mesh in model.findall('.//mesh'):
    path = mesh.attrib['filename'].replace('package://', 'src/')
    assert (root / path).is_file(), path
control = root / 'src/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro'
assert ET.parse(control).find('.//hardware/plugin').text.strip() == 'mock_components/GenericSystem'
assert not (root / 'src/fairino_hardware').exists()
print(f'PASS: {len(manifest)} source hashes, Python/shell syntax, package XML, FR5 joints and meshes, mock-only hardware')
