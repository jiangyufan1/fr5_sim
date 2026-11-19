#!/usr/bin/python3
"""Single-object Gazebo trial. Ground-truth targeting, not a learned ACT policy."""
import argparse,json,math,os,signal,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
import rclpy,yaml
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK,GetMotionPlan,GetPlanningScene,ApplyPlanningScene
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,PlanningSceneComponents,AllowedCollisionEntry
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rosidl_runtime_py.convert import message_to_ordereddict

p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');args=p.parse_args()
assert os.environ.get('ROS_DOMAIN_ID')=='86'
rclpy.init();n=rclpy.create_node('fr5_single_pick_trial');latest=[]
sub=n.create_subscription(JointState,'/joint_states',lambda m:latest.append((time.monotonic(),m)),10)
root=Path('/data/Projects/fr5_sim');cfg=yaml.safe_load((root/'src/fr5_act_sim/config/gazebo_scene.yaml').read_text())
out=root/'outputs'/('pick_trial_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
log=open(out/'events.jsonl','w');bag=None;active=None

def event(stage,**kw):
 record={'wall_time':time.time(),'stage':stage,**kw};log.write(json.dumps(record)+'\n');log.flush();print(stage,kw,flush=True)
def wait(f,timeout=20):
 rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
 if not f.done():raise RuntimeError('ROS operation timeout')
 return f.result()
def service(kind,name,req):
 c=n.create_client(kind,name)
 if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Missing '+name)
 return wait(c.call_async(req),25)
def state():
 end=time.monotonic()+5
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.05)
  if latest and time.monotonic()-latest[-1][0]<.5:return RobotState(joint_state=latest[-1][1])
 raise RuntimeError('No fresh joint feedback')
def ik(x,y,z,yaw,seed):
 req=GetPositionIK.Request();q=req.ik_request;q.group_name='fairino5_v6_group';q.ik_link_name='wrist3_link';q.robot_state=seed;q.avoid_collisions=True;q.timeout=Duration(sec=1)
 q.pose_stamped.header.frame_id='world';q.pose_stamped.pose.position.x=x;q.pose_stamped.pose.position.y=y;q.pose_stamped.pose.position.z=z+.214
 q.pose_stamped.pose.orientation.w=0.;q.pose_stamped.pose.orientation.z=0.;q.pose_stamped.pose.orientation.x=-math.sin(yaw/2);q.pose_stamped.pose.orientation.y=math.cos(yaw/2)
 r=service(GetPositionIK,'/compute_ik',req)
 return r.solution if r.error_code.val==1 else None

def plan(start,target):
 req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='fairino5_v6_group';m.pipeline_id='ompl';m.start_state=start;m.allowed_planning_time=5.;m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1
 values=dict(zip(target.joint_state.name,target.joint_state.position))
 m.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=f'j{i}',position=values[f'j{i}'],tolerance_above=.001,tolerance_below=.001,weight=1.) for i in range(1,7)])]
 r=service(GetMotionPlan,'/plan_kinematic_path',req).motion_plan_response
 if r.error_code.val!=1:raise RuntimeError(f'Planning failed {r.error_code.val}')
 return r.trajectory

def execute(trajectory,stage):
 global active
 c=ActionClient(n,ExecuteTrajectory,'/execute_trajectory');assert c.wait_for_server(timeout_sec=10)
 event(stage+'_command',trajectory=message_to_ordereddict(trajectory))
 active=wait(c.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)))
 if not active.accepted:raise RuntimeError('Execution rejected')
 result=wait(active.get_result_async(),90);active=None
 if result.result.error_code.val!=1:raise RuntimeError(f'Execution failed {result.result.error_code.val}')
 event(stage+'_complete')

def gripper(width):
 global active
 c=ActionClient(n,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory');assert c.wait_for_server(timeout_sec=10)
 g=FollowJointTrajectory.Goal();g.trajectory.joint_names=['gripper_left_joint','gripper_right_joint'];g.trajectory.points=[JointTrajectoryPoint(positions=[width/2]*2,time_from_start=Duration(sec=3))]
 event('gripper_command',width=width,trajectory=message_to_ordereddict(g.trajectory));active=wait(c.send_goal_async(g));assert active.accepted
 r=wait(active.get_result_async(),40);active=None
 if r.result.error_code!=0:raise RuntimeError(f'Gripper action failed {r.result.error_code}: {r.result.error_string}')

try:
 description=service(GetParameters,'/robot_state_publisher/get_parameters',GetParameters.Request(names=['robot_description'])).values[0].string_value
 assert ET.fromstring(description).findtext('ros2_control/hardware/plugin')=='gz_ros2_control/GazeboSimSystem'
 scene=service(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request(components=PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))).scene
 target=next(o for o in scene.world.collision_objects if o.id=='object_3');pos=target.primitive_poses[0].position
 base_pose=getattr(target,'pose',None)
 x=pos.x+(base_pose.position.x if base_pose else 0.);y=pos.y+(base_pose.position.y if base_pose else 0.);pos.z+=base_pose.position.z if base_pose else 0.
 event('target',name='object_3',xyz=[x,y,pos.z],source='initial_simulation_scene',tcp_grasp_height=.800)
 initial=state();selected=None
 for yaw in [0.,math.pi/2,-math.pi/2,math.pi]:
  pre=ik(x,y,.95,yaw,initial)
  if pre is None:continue
  near=ik(x,y,.80,yaw,pre)
  if near is not None:selected=(yaw,pre,near);break
 if selected is None:raise RuntimeError('No collision-free IK for both pregrasp and compensated grasp height')
 yaw,pre,near=selected
 t1=plan(initial,pre);t2=plan(pre,near)
 (out/'planned.json').write_text(json.dumps({'pregrasp':message_to_ordereddict(t1),'approach':message_to_ordereddict(t2)}))
 event('plans_ready',yaw=yaw,pregrasp_points=len(t1.joint_trajectory.points),approach_points=len(t2.joint_trajectory.points))
 if args.execute:
  baglog=open(out/'rosbag.log','w')
  bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'rosbag'),'/clock','/joint_states','/wrist_camera/image','/wrist_camera/camera_info','/fairino5_controller/controller_state','/gripper_controller/controller_state','/tf','/tf_static'],stdout=baglog,stderr=subprocess.STDOUT)
  time.sleep(2)
  if bag.poll() is not None:raise RuntimeError('Recorder failed')
  execute(t1,'pregrasp');execute(plan(state(),near),'approach')
  gripper(.039)
  event('contact_attempt_complete',success=False,reason='Lift verification pending; do not use as successful training episode')
 else:event('planning_only_complete')
except Exception as e:
 event('failed',error=str(e),success=False);raise
finally:
 if active is not None:
  try:wait(active.cancel_goal_async(),5)
  except Exception:pass
 if bag is not None:
  bag.send_signal(signal.SIGINT)
  try:bag.wait(timeout=15)
  except subprocess.TimeoutExpired:bag.terminate()
 log.close();n.destroy_node();rclpy.shutdown();print('OUTPUT',out,flush=True)
