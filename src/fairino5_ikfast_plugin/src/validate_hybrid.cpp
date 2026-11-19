#include "hybrid_plugin.hpp"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <limits>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>

static std::string read(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot read " + path);
  return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}
struct Counts { size_t total=0, success=0; };
int main(int argc, char** argv) {
  if (argc!=5) { std::cerr << "usage: validator urdf srdf cases report\n"; return 2; }
  rclcpp::init(argc, argv);
  int status=1;
  try {
    auto node=std::make_shared<rclcpp::Node>("fr5_ikfast_offline_validation");
    auto urdf=urdf::parseURDF(read(argv[1]));
    if (!urdf) throw std::runtime_error("URDF parse failed");
    auto semantic=std::make_shared<srdf::Model>();
    if (!semantic->initString(*urdf, read(argv[2]))) throw std::runtime_error("SRDF parse failed");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf, semantic);
    const auto* group=model->getJointModelGroup("fairino5_v6_group");
    if (!group || group->getVariableNames()!=std::vector<std::string>{"j1","j2","j3","j4","j5","j6"})
      throw std::runtime_error("Unexpected joint order");
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    auto plugin=loader.createSharedInstance("fr5_kinematics/IKFastWithKDL");
    auto* hybrid=dynamic_cast<fr5_kinematics::Diagnostics*>(plugin.get());
    if (!hybrid || !plugin->initialize(node, *model, group->getName(), "base_link", {"wrist3_link"}, 0.005))
      throw std::runtime_error("Actual plugin loading/initialization failed");
    plugin->setDefaultTimeout(0.05);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::map<std::string, Counts> counts;
    double max_position=0, max_rotation=0;
    size_t invalid=0;
    std::ifstream input(argv[3]);
    if (!input) throw std::runtime_error("Missing test cases");
    std::string suite;
    geometry_msgs::msg::Pose example;
    std::vector<double> example_seed;
    while (input >> suite) {
      std::vector<double> seed(6), solution;
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      for (double& v:seed) input>>v;
      for (int i=0;i<3;++i) input>>translation[i];
      for (int i=0;i<3;++i) for (int j=0;j<3;++j) input>>rotation(i,j);
      if (!input) throw std::runtime_error("Truncated test case");
      Eigen::Quaterniond quaternion(rotation);
      quaternion.normalize();
      geometry_msgs::msg::Pose pose;
      pose.position.x=translation.x(); pose.position.y=translation.y(); pose.position.z=translation.z();
      pose.orientation.x=quaternion.x(); pose.orientation.y=quaternion.y();
      pose.orientation.z=quaternion.z(); pose.orientation.w=quaternion.w();
      if (example_seed.empty() && suite=="regular") { example=pose; example_seed=seed; }
      auto& count=counts[suite]; ++count.total;
      moveit_msgs::msg::MoveItErrorCodes error;
      bool solved=plugin->searchPositionIK(pose, seed, 0.05, solution, error);
      if (!solved) continue;
      if (solution.size()!=6 || error.val!=error.SUCCESS) { ++invalid; continue; }
      state.setJointGroupPositions(group, solution); state.update();
      auto actual=state.getGlobalLinkTransform("base_link").inverse()*state.getGlobalLinkTransform("wrist3_link");
      double dp=(actual.translation()-translation).norm();
      double dr=Eigen::Quaterniond(actual.rotation()).angularDistance(quaternion);
      if (!std::isfinite(dp) || !std::isfinite(dr) || !state.satisfiesBounds(group, 1e-9) || dp>2e-5 || dr>2e-5) {
        ++invalid; continue;
      }
      max_position=std::max(max_position, dp); max_rotation=std::max(max_rotation, dr);
      ++count.success;
    }
    const auto stats=hybrid->statistics();
    bool passed=invalid==0 && stats.primary>0 && stats.fallback>0;
    for (const auto& name: {"regular", "wrist_zero", "near_limits"}) {
      const auto& c=counts[name];
      passed = passed && c.total>=100 && double(c.success)/c.total>=0.99;
    }
    // A caller's collision rejection must remain a rejection through fallback.
    size_t callback_calls=0;
    kinematics::KinematicsBase::IKCallbackFn reject=[&](const geometry_msgs::msg::Pose&,
        const std::vector<double>&, moveit_msgs::msg::MoveItErrorCodes& e) { ++callback_calls; e.val=e.GOAL_IN_COLLISION; };
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error;
    bool rejected=!plugin->searchPositionIK(example, example_seed, 0.01, solution, reject, error) &&
                  callback_calls>0 && solution.empty();
    bool consistency=!plugin->searchPositionIK(example, example_seed, 0.01,
        std::vector<double>(6,0.0), solution, error);
    auto bad=example; bad.position.x=std::numeric_limits<double>::quiet_NaN();
    bool finite_rejected=!plugin->searchPositionIK(bad, example_seed, 0.01, solution, error);
    auto wrong_seed=example_seed; wrong_seed.pop_back();
    bool size_rejected=!plugin->searchPositionIK(example, wrong_seed, 0.01, solution, error);
    auto bad_seed=example_seed; bad_seed[0]=100.0;
    bool bounds_rejected=!plugin->searchPositionIK(example, bad_seed, 0.01, solution, error);
    bool timeout_rejected=!plugin->searchPositionIK(example, example_seed, 0.0, solution, error);
    auto unreachable=example; unreachable.position.x=100.0;
    bool unreachable_rejected=!plugin->searchPositionIK(unreachable, example_seed, 0.01, solution, error);
    std::vector<std::string> fk_links{"base_link", "shoulder_link", "wrist1_link", "wrist3_link"};
    std::vector<geometry_msgs::msg::Pose> fk_poses;
    bool fk_correct=plugin->getPositionFK(fk_links, example_seed, fk_poses) && fk_poses.size()==fk_links.size();
    state.setJointGroupPositions(group, example_seed); state.update();
    for (size_t i=0; fk_correct && i<fk_links.size(); ++i) {
      const Eigen::Isometry3d transform=state.getGlobalLinkTransform("base_link").inverse()*state.getGlobalLinkTransform(fk_links[i]);
      const auto& p=fk_poses[i];
      const Eigen::Vector3d delta=transform.translation()-Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
      const Eigen::Quaterniond rotation(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
      fk_correct=delta.norm()<1e-10 && Eigen::Quaterniond(transform.rotation()).angularDistance(rotation)<1e-10;
    }
    passed = passed && rejected && consistency && finite_rejected && size_rejected &&
             bounds_rejected && timeout_rejected && unreachable_rejected && fk_correct;
    std::ofstream report(argv[4]);
    report.precision(17);
    report << "{\n  \"passed\": " << (passed?"true":"false") << ",\n  \"suites\": {";
    bool first=true;
    for (const auto& kv:counts) {
      if (!first) report<<','; first=false;
      report << "\n    \""<<kv.first<<"\": {\"tested\": "<<kv.second.total
             <<", \"solved\": "<<kv.second.success<<"}";
    }
    report << "\n  },\n  \"invalid_solutions\": "<<invalid
           <<",\n  \"max_position_error_m\": "<<max_position
           <<",\n  \"max_rotation_error_rad\": "<<max_rotation
           <<",\n  \"ikfast_successes\": "<<stats.primary
           <<",\n  \"kdl_fallback_calls\": "<<stats.fallback_calls
           <<",\n  \"kdl_successes\": "<<stats.fallback
           <<",\n  \"callback_rejection_preserved\": "<<(rejected?"true":"false")
           <<",\n  \"consistency_limits_preserved\": "<<(consistency?"true":"false")
           <<",\n  \"invalid_inputs_rejected\": "<<((finite_rejected&&size_rejected&&bounds_rejected&&timeout_rejected)?"true":"false")
           <<",\n  \"unreachable_target_rejected\": "<<(unreachable_rejected?"true":"false")
           <<",\n  \"multiple_link_fk_correct\": "<<(fk_correct?"true":"false")<<"\n}\n";
    report.close();
    std::cout<<read(argv[4])<<std::endl;
    status=passed?0:1;
  } catch (const std::exception& e) { std::cerr<<"Offline validation failed: "<<e.what()<<std::endl; }
  rclcpp::shutdown();
  return status;
}
