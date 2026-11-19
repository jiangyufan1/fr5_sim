search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=fairino5_v6_robot.srdf
robot_name_in_srdf=fairino5_v6_robot
moveit_config_pkg=fairino5_v6_robot_moveit_config
robot_name=fairino5_v6_robot
planning_group_name=fairino5_v6_group
ikfast_plugin_pkg=fairino5_ikfast_plugin
base_link_name=base_link
eef_link_name=wrist3_link
ikfast_output_path=/data/Projects/fr5_ros2/outputs/ikfast_hybrid_20261113_194021/fairino5_ikfast_plugin/src/fairino5_v6_robot_fairino5_v6_group_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
