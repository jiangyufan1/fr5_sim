import rclpy,time
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity, GetMotionPlan
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
rclpy.init(); n=rclpy.create_node('gripper_planning_check'); latest=[]
sub=n.create_subscription(JointState,'/joint_states',lambda m:latest.append(m),10)
end=time.monotonic()+10
while not latest and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
assert latest,'No joint state'
state=RobotState(joint_state=latest[-1])
client=n.create_client(GetStateValidity,'/check_state_validity');assert client.wait_for_service(timeout_sec=10)
f=client.call_async(GetStateValidity.Request(robot_state=state,group_name='fairino5_v6_group'));rclpy.spin_until_future_complete(n,f,timeout_sec=10)
r=f.result();assert r is not None;print('Current state valid:',r.valid,'Contacts:',[(x.contact_body_1,x.contact_body_2) for x in r.contacts]);assert r.valid
client=n.create_client(GetMotionPlan,'/plan_kinematic_path');assert client.wait_for_service(timeout_sec=10)
req=GetMotionPlan.Request();req.motion_plan_request.group_name='fairino5_v6_group';req.motion_plan_request.pipeline_id='ompl';req.motion_plan_request.start_state=state;req.motion_plan_request.allowed_planning_time=5.0;req.motion_plan_request.max_velocity_scaling_factor=.1;req.motion_plan_request.max_acceleration_scaling_factor=.1
q=dict(zip(state.joint_state.name,state.joint_state.position));q['j1']+=.1
req.motion_plan_request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=f'j{i}',position=q[f'j{i}'],tolerance_above=.001,tolerance_below=.001,weight=1.) for i in range(1,7)])]
f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=15);r=f.result();assert r is not None;print('Plan result:',r.motion_plan_response.error_code.val,'points:',len(r.motion_plan_response.trajectory.joint_trajectory.points));assert r.motion_plan_response.error_code.val==1
n.destroy_node();rclpy.shutdown()
