#!/usr/bin/python3
"""Add the Gazebo tabletop to MoveIt as a collision object."""
import time
import importlib.util
from pathlib import Path
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def main():
    rclpy.init();node=rclpy.create_node('table_planning_scene')
    try:
        cfg=yaml.safe_load((Path(get_package_share_directory('fr5_act_sim'))/'config/gazebo_scene.yaml').read_text())
        client=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        if not client.wait_for_service(timeout_sec=60):raise RuntimeError('MoveIt scene service unavailable')
        obj=CollisionObject();obj.id='workbench';obj.header.frame_id='world';obj.operation=CollisionObject.ADD
        size=cfg['table_size'];cx=cfg['table_center_x'];top=cfg['table_top']
        parts=[([cx,0,top-size[2]/2],size)]
        for x in [cx-size[0]/2+.08,cx+size[0]/2-.08]:
            for y in [-size[1]/2+.08,size[1]/2-.08]:parts.append(([x,y,(top-size[2])/2],[.06,.06,top-size[2]]))
        for pos,dims in parts:
            primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dims)
            pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=map(float,pos)
            obj.primitives.append(primitive);obj.primitive_poses.append(pose)
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.world.collision_objects=[obj]
        share=Path(get_package_share_directory('fr5_act_sim'))
        spec=importlib.util.spec_from_file_location('task_layout',share/'launch/task_layout.py')
        layout=importlib.util.module_from_spec(spec);spec.loader.exec_module(layout)
        entries=[('collection_bin',layout.bin_parts(cfg))]
        entries += [(item['name'],[(item['xyz'],item['size'])]) for item in layout.objects(cfg)]
        for name,boxes in entries:
            obstacle=CollisionObject();obstacle.id=name;obstacle.header.frame_id='world';obstacle.operation=CollisionObject.ADD
            for pos,dims in boxes:
                primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dims)
                pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=map(float,pos)
                obstacle.primitives.append(primitive);obstacle.primitive_poses.append(pose)
            req.scene.world.collision_objects.append(obstacle)
        f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=15)
        if not f.done() or not f.result().success:raise RuntimeError('Failed to add table collision object')
        node.get_logger().info('Table, open bin and five initial object poses added to MoveIt planning scene')
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
