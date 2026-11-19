#!/usr/bin/python3
"""Integration check: mock identity, feedback, two trajectories and cancellation."""
import math
import os
import time
import rclpy
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListHardwareComponents
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '85' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use test_sim.sh with isolated simulation environment')
    rclpy.init()
    node = rclpy.create_node('fr5_sim_check')
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
        if len(hardware) != 1 or hardware[0].class_type != 'mock_components/GenericSystem' or hardware[0].state.label != 'active':
            raise RuntimeError('Refusing to send goals: hardware is not mock')
        names = [f'j{i}' for i in range(1, 7)]
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
        client = ActionClient(node, FollowJointTrajectory, '/fairino5_controller/follow_joint_trajectory')
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
        target = start.copy()
        target[0] += 0.15 if start[0] < 2.8 else -0.15
        for pose in (target, start):
            active = send(pose, 3)
            result = wait(active.get_result_async())
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'Trajectory failed: {result}')
            active = None
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if max(abs(latest[n] - q) for n, q in zip(names, pose)) < 0.01:
                    break
            else:
                raise RuntimeError('Joint feedback did not reach target')
        active = send(target, 10)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        cancel = wait(active.cancel_goal_async())
        result = wait(active.get_result_async())
        if not cancel.goals_canceling or result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('Trajectory cancellation failed')
        active = None
        print('PASS: mock identity, joint feedback, two successive trajectories, cancellation')
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
