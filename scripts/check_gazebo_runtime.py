#!/usr/bin/python3
"""Check real Gazebo messages, never connects to physical hardware."""
import os,time,math
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from rosgraph_msgs.msg import Clock
from controller_manager_msgs.srv import ListHardwareComponents
from rcl_interfaces.srv import GetParameters
import xml.etree.ElementTree as ET
if os.environ.get('ROS_DOMAIN_ID')!='86':raise SystemExit('Use Gazebo simulation domain 86')
rclpy.init();n=rclpy.create_node('fr5_gazebo_check');seen={}
subs=[]
for topic,kind,key in [('/wrist_camera/image',Image,'image'),('/joint_states',JointState,'joints'),('/clock',Clock,'clock')]:
    subs.append(n.create_subscription(kind,topic,lambda msg,k=key:seen.__setitem__(k,msg),qos_profile_sensor_data))
try:
    client=n.create_client(ListHardwareComponents,'/controller_manager/list_hardware_components')
    if not client.wait_for_service(timeout_sec=45):raise RuntimeError('Gazebo controller manager missing')
    f=client.call_async(ListHardwareComponents.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=15)
    if not f.done() or len(f.result().component)!=1 or f.result().component[0].state.label!='active':raise RuntimeError('Not Gazebo hardware')
    model_client=n.create_client(GetParameters,'/robot_state_publisher/get_parameters')
    assert model_client.wait_for_service(timeout_sec=5)
    mf=model_client.call_async(GetParameters.Request(names=['robot_description']))
    rclpy.spin_until_future_complete(n,mf,timeout_sec=10)
    assert mf.done() and ET.fromstring(mf.result().values[0].string_value).findtext('ros2_control/hardware/plugin')=='gz_ros2_control/GazeboSimSystem'
    end=time.monotonic()+30
    while len(seen)<3 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
    if len(seen)!=3:raise RuntimeError('Missing messages: '+str({'image','joints','clock'}-seen.keys()))
    im=seen['image'];assert im.width==640 and im.height==480 and len(im.data)>=im.step*im.height
    assert len(set(im.data))>10,'Camera image is nearly uniform; inspect render/scene'
    assert all(math.isfinite(v) for v in seen['joints'].position)
    assert {'j1','j2','j3','j4','j5','j6','gripper_left_joint','gripper_right_joint'}<=set(seen['joints'].name)
    print(f'PASS: Gazebo hardware, simulation clock, eight joint states, camera {im.width}x{im.height} {im.encoding}')
finally:n.destroy_node();rclpy.shutdown()
