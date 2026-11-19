#!/usr/bin/python3
"""Integration check: mock identity, feedback, two trajectories and cancellation."""
import math
import argparse
import yaml
from pathlib import Path
import os
import time
import rclpy
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListHardwareComponents
from rcl_interfaces.srv import GetParameters
import xml.etree.ElementTree as ET
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use gazebo_gripper.sh with isolated simulation environment')
    parser = argparse.ArgumentParser(description='Mock gripper opening, in metres')
    parser.add_argument('width', type=float, nargs='?', help='Total inner opening in metres')
    parser.add_argument('--test', action='store_true', help='Open, close, restore and cancel test')
    args = parser.parse_args()
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / 'src/fr5_act_sim/config/gripper.yaml').read_text())
    if not args.test and (args.width is None or not math.isfinite(args.width) or not 0 <= args.width <= cfg['max_width']):
        parser.error(f'width must be between 0 and {cfg["max_width"]} metres')
    rclpy.init()
    node = rclpy.create_node('fr5_gripper_command')
    active = None
    def wait(future, timeout=15):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS operation timed out')
        return future.result()
    try:
        params = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        if not params.wait_for_service(timeout_sec=15):
            raise RuntimeError('Start the simulation first')
        response = wait(params.call_async(ListHardwareComponents.Request()))
        hardware = response.component
        if len(hardware) != 1 or hardware[0].state.label != 'active':
            raise RuntimeError('Refusing to send goals: hardware is not Gazebo')
        model_client=node.create_client(GetParameters,'/robot_state_publisher/get_parameters')
        if not model_client.wait_for_service(timeout_sec=5): raise RuntimeError('Model service unavailable')
        description=wait(model_client.call_async(GetParameters.Request(names=['robot_description'])))
        if ET.fromstring(description.values[0].string_value).findtext('ros2_control/hardware/plugin')!='gz_ros2_control/GazeboSimSystem':
            raise RuntimeError('Not Gazebo hardware')
        names = ['gripper_left_joint', 'gripper_right_joint']
        latest = {}
        def feedback(msg):
            latest.update({n: p for n, p in zip(msg.name, msg.position)})
        subscription = node.create_subscription(JointState, '/joint_states', feedback, 10)
        deadline = time.monotonic() + 10
        while not all(n in latest for n in names) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        start = [latest[n] for n in names]
        if not all(math.isfinite(v) for v in start):
            raise RuntimeError('Invalid feedback')
        client = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Trajectory action is unavailable')
        def send(target, seconds):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names
            goal.trajectory.points = [JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=seconds))]
            handle = wait(client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Trajectory rejected')
            return handle
        target = [cfg['max_width']/2]*2
        poses = [target, [0.002]*2, start] if args.test else [[args.width/2]*2]
        for pose in poses:
            seconds = max(2, math.ceil(max(abs(latest[n]-q) for n,q in zip(names,pose))/cfg['finger_speed'])+1)
            active = send(pose, seconds)
            result = wait(active.get_result_async(), seconds+10)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'Trajectory failed: {result}')
            active = None
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if max(abs(latest[n] - q) for n, q in zip(names, pose)) < 0.001:
                    break
            else:
                raise RuntimeError('Gripper feedback did not reach target')
            print(f'Gripper opening: {sum(latest[n] for n in names):.4f} m')
        if not args.test:
            return
        active = send(target, 10)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        cancel = wait(active.cancel_goal_async())
        result = wait(active.get_result_async())
        if not cancel.goals_canceling or result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('Trajectory cancellation failed')
        active = None
        print('PASS: Gazebo gripper identity, open/close/restore feedback, cancellation')
    finally:
        if active is not None:
            try:
                wait(active.cancel_goal_async(), 3)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
