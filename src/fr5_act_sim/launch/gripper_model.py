"""Configurable geometric placeholder; no physical or tactile simulation."""
import math
import xml.etree.ElementTree as ET

JOINTS = ['gripper_left_joint', 'gripper_right_joint']


def add_gripper(urdf, srdf, cfg):
    for key in ('mount_xyz', 'mount_rpy', 'base_size'):
        if len(cfg[key]) != 3 or not all(math.isfinite(x) for x in cfg[key]):
            raise ValueError(f'Invalid {key}')
    for key in ('finger_length', 'finger_thickness', 'finger_width', 'max_width', 'finger_speed'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'Invalid {key}')
    if not all(x > 0 for x in cfg['base_size']) or not 0 <= cfg['initial_width'] <= cfg['max_width']:
        raise ValueError('Invalid dimensions/initial width')
    root, semantic = ET.fromstring(urdf), ET.fromstring(srdf)
    systems = root.findall('ros2_control')
    if len(systems) != 1 or systems[0].findtext('hardware/plugin').strip() != 'mock_components/GenericSystem':
        raise ValueError('Gripper placeholder requires mock hardware')
    if root.find("link[@name='wrist3_link']") is None:
        raise ValueError('Missing FR5 mounting link')
    def vec(values):
        return ' '.join(str(v) for v in values)
    def box_link(name, size, center, color):
        link = ET.SubElement(root, 'link', name=name)
        for kind in ('visual', 'collision'):
            element = ET.SubElement(link, kind)
            ET.SubElement(element, 'origin', xyz=vec(center), rpy='0 0 0')
            ET.SubElement(ET.SubElement(element, 'geometry'), 'box', size=vec(size))
            if kind == 'visual':
                ET.SubElement(ET.SubElement(element, 'material', name=name+'_color'), 'color', rgba=color)
    def fixed(name, parent, child, xyz, rpy=(0, 0, 0)):
        j = ET.SubElement(root, 'joint', name=name, type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=vec(xyz), rpy=vec(rpy))
    def ignore(a,b):
        ET.SubElement(semantic, 'disable_collisions', link1=a, link2=b, reason='Adjacent')
    length, thick, width = (cfg[k] for k in ('finger_length', 'finger_thickness', 'finger_width'))
    base_z = cfg['base_size'][2]
    sensor_size = cfg['sensor_envelope_xyz']
    depth = sensor_size[1]
    if len(sensor_size) != 3 or not all(math.isfinite(x) and x > 0 for x in sensor_size):
        raise ValueError('Invalid sensor envelope')
    box_link('gripper_base', cfg['base_size'], (0,0,base_z/2), '0.25 0.28 0.32 1')
    fixed('gripper_mount_joint', 'wrist3_link', 'gripper_base', cfg['mount_xyz'], cfg['mount_rpy'])
    ignore('wrist3_link', 'gripper_base')
    group = ET.SubElement(semantic, 'group', name='gripper')
    ET.SubElement(group, 'link', name='gripper_base')
    for side, sign, name in zip(('left','right'), (1,-1), JOINTS):
        link = f'gripper_{side}_finger'
        box_link(link, (width, thick, length), (0,0,length/2), '0.65 0.68 0.72 1')
        j = ET.SubElement(root,'joint', name=name,type='prismatic')
        ET.SubElement(j,'parent',link='gripper_base'); ET.SubElement(j,'child',link=link)
        ET.SubElement(j,'origin',xyz=vec((0,sign*(depth+thick/2),base_z)))
        ET.SubElement(j,'axis',xyz=vec((0,sign,0)))
        ET.SubElement(j,'limit',lower='0',upper=str(cfg['max_width']/2),effort='1',velocity=str(cfg['finger_speed']))
        control = ET.SubElement(systems[0],'joint',name=name)
        command = ET.SubElement(control,'command_interface',name='position')
        ET.SubElement(command,'param',name='min').text='0'
        ET.SubElement(command,'param',name='max').text=str(cfg['max_width']/2)
        state = ET.SubElement(control,'state_interface',name='position')
        ET.SubElement(state,'param',name='initial_value').text=str(cfg['initial_width']/2)
        ET.SubElement(group,'joint',name=name)
        ignore('gripper_base',link)
        # CAD visual at native metric scale; envelope includes modeled cable.
        # Provisional orientation, pending actual finger mounting drawing.
        pad=f'tactile_{side}_pad'
        box_link(pad, sensor_size, (0,0,0), '0.1 0.65 0.9 1')
        visual=root.find(f"link[@name='{pad}']/visual")
        visual.remove(visual.find('geometry'))
        ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename=cfg['sensor_mesh'])
        orientation = cfg['sensor_visual_rpy'][side]
        if len(orientation) != 3 or not all(math.isfinite(v) for v in orientation):
            raise ValueError('Invalid sensor visual orientation')
        visual.find('origin').set('rpy',vec(orientation))
        fixed(pad+'_joint',link,pad,(0,-sign*(thick/2+depth/2),length/2))
        ET.SubElement(group,'link',name=pad)
        ignore(link,pad)
    ET.SubElement(root,'link',name='gripper_tcp')
    fixed('gripper_tcp_joint','gripper_base','gripper_tcp',(0,0,base_z+length*0.6))
    ET.SubElement(group,'link',name='gripper_tcp')
    ET.SubElement(semantic,'end_effector',name='parallel_gripper',parent_link='wrist3_link',group='gripper',parent_group='fairino5_v6_group')
    for name, opening in [('open',cfg['max_width']),('closed',0.0)]:
        state=ET.SubElement(semantic,'group_state',name=name,group='gripper')
        for joint in JOINTS:
            ET.SubElement(state,'joint',name=joint,value=str(opening/2))
    return ET.tostring(root,encoding='unicode'), ET.tostring(semantic,encoding='unicode')
