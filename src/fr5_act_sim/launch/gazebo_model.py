"""Gazebo physical model plus visible wrist camera. Added masses are estimates."""
import xml.etree.ElementTree as ET
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def xyz(values):
    return ' '.join(str(v) for v in values)


def add_box(root, name, size, color, mass):
    link = ET.SubElement(root, 'link', name=name)
    for tag in ('visual', 'collision'):
        el=ET.SubElement(link,tag)
        ET.SubElement(ET.SubElement(el,'geometry'),'box',size=xyz(size))
        if tag=='visual':
            ET.SubElement(ET.SubElement(el,'material',name=name+'_material'),'color',rgba=color)
    add_inertia(link,mass,size)
    return link


def add_inertia(link,mass,size,center=(0,0,0)):
    a,b,c=size
    inertial=ET.SubElement(link,'inertial')
    ET.SubElement(inertial,'origin',xyz=xyz(center))
    ET.SubElement(inertial,'mass',value=str(mass))
    ET.SubElement(inertial,'inertia',ixx=str(mass*(b*b+c*c)/12),iyy=str(mass*(a*a+c*c)/12),izz=str(mass*(a*a+b*b)/12),ixy='0',ixz='0',iyz='0')


def fixed(root,name,parent,child,position,rotation=(0,0,0)):
    j=ET.SubElement(root,'joint',name=name,type='fixed')
    ET.SubElement(j,'parent',link=parent); ET.SubElement(j,'child',link=child)
    ET.SubElement(j,'origin',xyz=xyz(position),rpy=xyz(rotation))


def build(urdf,srdf,scene,share):
    root,sem=ET.fromstring(urdf),ET.fromstring(srdf)
    hardware=root.find('ros2_control/hardware/plugin')
    if hardware.text.strip()!='mock_components/GenericSystem':
        raise ValueError('Expected mock model as input')
    hardware.text='gz_ros2_control/GazeboSimSystem'
    for i,q in enumerate(scene['initial_arm'],1):
        root.find(f"ros2_control/joint[@name='j{i}']/state_interface/param[@name='initial_value']").text=str(q)
    # Add inertia to the previously visual-only gripper components.
    for link in root.findall('link'):
        if link.find('inertial') is not None or link.find('collision') is None:
            continue
        box=link.find('collision/geometry/box')
        if box is None: raise ValueError('Missing box inertia source')
        size=list(map(float,box.get('size').split()))
        origin=link.find('collision/origin')
        center=list(map(float,origin.get('xyz','0 0 0').split())) if origin is not None else (0,0,0)
        add_inertia(link,0.3 if link.get('name')=='gripper_base' else 0.08,size,center)
    # Root fixed to world, at the table surface. Not a static robot: joints still move.
    ET.SubElement(root,'link',name='world')
    fixed(root,'table_mount_joint','world','base_link',scene['robot_mount_xyz'])
    add_box(root,'wrist_camera_link',(0.035,0.05,0.035),'0.12 0.15 0.18 1',0.06)
    fixed(root,'wrist_camera_joint','gripper_base','wrist_camera_link',scene['camera_mount_xyz'],scene['camera_mount_rpy'])
    lens=ET.SubElement(root.find("link[@name='wrist_camera_link']"),'visual')
    ET.SubElement(lens,'origin',xyz='0.020 0 0',rpy='0 1.5707963267948966 0')
    ET.SubElement(ET.SubElement(lens,'geometry'),'cylinder',radius='0.009',length='0.008')
    ET.SubElement(ET.SubElement(lens,'material',name='camera_lens'),'color',rgba='0.02 0.07 0.13 1')
    ET.SubElement(root,'link',name='wrist_camera_optical_frame')
    fixed(root,'wrist_camera_optical_joint','wrist_camera_link','wrist_camera_optical_frame',(0.025,0,0),(-1.5707963267948966,0,-1.5707963267948966))
    ET.SubElement(sem,'disable_collisions',link1='gripper_base',link2='wrist_camera_link',reason='Adjacent')
    gazebo=ET.SubElement(root,'gazebo',reference='wrist_camera_link')
    sensor=ET.SubElement(gazebo,'sensor',name='wrist_rgb',type='camera')
    for tag,text in [('always_on','true'),('update_rate',str(scene['camera_fps'])),('visualize','true'),('topic','/wrist_camera/image'),('pose','0.025 0 0 0 0 0')]:
        ET.SubElement(sensor,tag).text=text
    camera=ET.SubElement(sensor,'camera')
    ET.SubElement(camera,'horizontal_fov').text=str(scene['camera_horizontal_fov'])
    image=ET.SubElement(camera,'image')
    for tag,value in [('width',scene['camera_width']),('height',scene['camera_height']),('format','R8G8B8')]:ET.SubElement(image,tag).text=str(value)
    clip=ET.SubElement(camera,'clip');ET.SubElement(clip,'near').text='0.02';ET.SubElement(clip,'far').text='5.0'
    plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',filename='gz_ros2_control-system',name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin,'parameters').text=str(share/'config/gazebo_controllers.yaml')
    for name in ['gripper_left_finger','gripper_right_finger','tactile_left_pad','tactile_right_pad']:
        contact=ET.SubElement(root,'gazebo',reference=name)
        ET.SubElement(contact,'mu1').text='0.8';ET.SubElement(contact,'mu2').text='0.8'
    # Resolve package meshes for Gazebo without depending on resource-path heuristics.
    for mesh in root.findall('.//mesh'):
        uri=mesh.get('filename')
        if uri.startswith('package://'):
            package,path=uri[len('package://'):].split('/',1)
            mesh.set('filename','file://'+str(Path(get_package_share_directory(package))/path))
    return ET.tostring(root,encoding='unicode'),ET.tostring(sem,encoding='unicode')


def world(scene):
    sdf=ET.Element('sdf',version='1.8');w=ET.SubElement(sdf,'world',name='fr5_lab')
    physics=ET.SubElement(w,'physics',name='physics',type='ignored')
    ET.SubElement(physics,'max_step_size').text='0.001'
    ET.SubElement(physics,'real_time_factor').text='1.0'
    for filename,name in [('physics-system','Physics'),('user-commands-system','UserCommands'),('scene-broadcaster-system','SceneBroadcaster'),('sensors-system','Sensors')]:
        plugin=ET.SubElement(w,'plugin',filename='ignition-gazebo-'+filename,name='ignition::gazebo::systems::'+name)
        if name=='Sensors':ET.SubElement(plugin,'render_engine').text='ogre2'
    ET.SubElement(w,'gravity').text='0 0 -9.81'
    light=ET.SubElement(w,'light',name='sun',type='directional')
    ET.SubElement(light,'pose').text='0 0 5 0 0 0';ET.SubElement(light,'diffuse').text='0.9 0.9 0.9 1';ET.SubElement(light,'direction').text='-0.5 -0.3 -1';ET.SubElement(light,'cast_shadows').text='true'
    def box_model(name,parts,static=True,mass=0.1):
        model=ET.SubElement(w,'model',name=name);ET.SubElement(model,'static').text=str(static).lower()
        for index,(pos,size,color) in enumerate(parts):
            link=ET.SubElement(model,'link',name=f'{name}_{index}');ET.SubElement(link,'pose').text=xyz(pos)+' 0 0 0'
            if not static:
                inertia=ET.SubElement(link,'inertial');ET.SubElement(inertia,'mass').text=str(mass)
                tensor=ET.SubElement(inertia,'inertia');a,b,c=size
                for tag,value in [('ixx',mass*(b*b+c*c)/12),('iyy',mass*(a*a+c*c)/12),('izz',mass*(a*a+b*b)/12),('ixy',0),('ixz',0),('iyz',0)]:ET.SubElement(tensor,tag).text=str(value)
            for tag in ['visual','collision']:
                el=ET.SubElement(link,tag,name=f'{tag}_{index}')
                ET.SubElement(ET.SubElement(ET.SubElement(el,'geometry'),'box'),'size').text=xyz(size)
                if tag=='visual':
                    material=ET.SubElement(el,'material');ET.SubElement(material,'ambient').text=color;ET.SubElement(material,'diffuse').text=color
    box_model('floor',[((0,0,-.05),(6,6,.1),'0.65 0.68 0.72 1')])
    cx=scene['table_center_x'];sx,sy,sz=scene['table_size'];top=scene['table_top']
    parts=[((cx,0,top-sz/2),(sx,sy,sz),'0.55 0.36 0.18 1')]
    for x in [cx-sx/2+.08,cx+sx/2-.08]:
        for y in [-sy/2+.08,sy/2-.08]:parts.append(((x,y,(top-sz)/2),(.06,.06,top-sz),'0.18 0.22 0.26 1'))
    box_model('workbench',parts)
    spec=importlib.util.spec_from_file_location('task_layout',Path(__file__).with_name('task_layout.py'))
    layout=importlib.util.module_from_spec(spec);spec.loader.exec_module(layout)
    box_model('collection_bin',[(pos,size,'0.12 0.48 0.55 1') for pos,size in layout.bin_parts(scene)])
    for item in layout.objects(scene):
        box_model(item['name'],[(item['xyz'],item['size'],item['color'])],False,mass=0.05)
    gui=ET.SubElement(w,'gui',fullscreen='0')
    view=ET.SubElement(gui,'plugin',filename='GzScene3D',name='3D View')
    ET.SubElement(view,'camera_pose').text='2 -2 1.8 0 0.35 2.4'
    # Use Gazebo's default GUI if no custom layout is provided.
    w.remove(gui)
    return ET.tostring(sdf,encoding='unicode')
