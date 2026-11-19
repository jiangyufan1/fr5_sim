"""Local mock hardware only; no vendor SDK or robot connection."""
import xml.etree.ElementTree as ET
import importlib.util
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg = MoveItConfigsBuilder('fairino5_v6_robot', package_name='fairino5_v6_moveit2_config').to_moveit_configs()
    share = Path(get_package_share_directory('fr5_act_sim'))
    spec = importlib.util.spec_from_file_location('gripper_model', share / 'launch/gripper_model.py')
    model = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(model)
    gripper = yaml.safe_load((share / 'config/gripper.yaml').read_text())
    xml, srdf = model.add_gripper(cfg.robot_description['robot_description'],
        cfg.robot_description_semantic['robot_description_semantic'], gripper)
    cfg.robot_description = {'robot_description': xml}
    cfg.robot_description_semantic = {'robot_description_semantic': srdf}
    limits = cfg.joint_limits['robot_description_planning']['joint_limits']
    for joint in model.JOINTS:
        limits[joint] = {'has_velocity_limits': True, 'max_velocity': gripper['finger_speed'],
                         'has_acceleration_limits': True, 'max_acceleration': 0.1}
    plugins = ET.fromstring(xml).findall('ros2_control/hardware/plugin')
    if len(plugins) != 1 or plugins[0].text.strip() != 'mock_components/GenericSystem':
        raise RuntimeError('Simulation requires exactly one mock_components/GenericSystem')
    cfg.trajectory_execution = {
        'moveit_manage_controllers': False,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['fairino5_controller', 'gripper_controller'],
            'gripper_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': model.JOINTS},
            'fairino5_controller': {
                'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': [f'j{i}' for i in range(1, 7)]}},
    }
    desc = {'robot_description': ParameterValue(xml, value_type=str), 'use_sim_time': False}
    share = Path(get_package_share_directory('fr5_act_sim'))
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[desc, str(share / 'config/controllers.yaml')], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[desc])
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'])
    controller = Node(package='controller_manager', executable='spawner',
                      arguments=['fairino5_controller', '--controller-manager-timeout', '60'])
    gripper_controller = Node(package='controller_manager', executable='spawner',
                      arguments=['gripper_controller', '--controller-manager-timeout', '60'])
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[cfg.to_dict(), {'use_sim_time': False,
                        'publish_robot_description_semantic': True,
                        'publish_planning_scene': True, 'publish_geometry_updates': True,
                        'publish_state_updates': True, 'publish_transforms_updates': True}])
    rviz = Node(package='rviz2', executable='rviz2',
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                arguments=['-d', str(cfg.package_path / 'config/moveit.rviz')],
                parameters=[desc, cfg.robot_description_semantic,
                            cfg.robot_description_kinematics, cfg.planning_pipelines, cfg.joint_limits])
    def after_success(actions):
        def callback(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Simulation controller startup failed'))]
            return actions
        return callback
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='Simulation controller manager exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after_success([controller]))),
        RegisterEventHandler(OnProcessExit(target_action=controller, on_exit=after_success([gripper_controller]))),
        RegisterEventHandler(OnProcessExit(target_action=gripper_controller, on_exit=after_success([move_group, rviz]))),
        rsp, manager, broadcaster,
    ])
