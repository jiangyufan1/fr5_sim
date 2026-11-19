#include "hybrid_plugin.hpp"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace fr5_kinematics {
namespace {
bool finite(const std::vector<double>& q) {
  return std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); });
}
bool validPose(const geometry_msgs::msg::Pose& p) {
  const auto& v = p.position;
  const auto& q = p.orientation;
  const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
         std::isfinite(norm) && std::abs(norm-1.0) < 1e-6;
}
}
bool IKFastWithKDL::initialize(const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModel& model, const std::string& group,
    const std::string& base, const std::vector<std::string>& tips, double step) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
  // This plugin is specific to the validated FR5 chain.
  if (group != "fairino5_v6_group" || base != "base_link" ||
      tips != std::vector<std::string>{"wrist3_link"}) return false;
  node_ = node;
  storeValues(model, group, base, tips, step);
  try {
    // Release previous instances before replacing the loader on reinitialization.
    primary_.reset(); fallback_.reset(); loader_.reset();
    loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>(
        "moveit_core", "kinematics::KinematicsBase"));
    primary_ = loader_->createSharedInstance(
        "fairino5_v6_robot_fairino5_v6_group/IKFastKinematicsPlugin");
    fallback_ = loader_->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
    if (!primary_->initialize(node, model, group, base, tips, step) ||
        !fallback_->initialize(node, model, group, base, tips, step)) return false;
    names_ = primary_->getJointNames();
    if (names_ != std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"} ||
        names_ != fallback_->getJointNames()) return false;
    links_ = fallback_->getLinkNames();
    stats_ = Statistics{};
    active_ = true;
    RCLCPP_INFO(node_->get_logger(), "FR5 IK: IKFast first, KDL fallback; original URDF residual checks enabled");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "FR5 IK initialization failed: %s", e.what());
    return false;
  }
}
bool IKFastWithKDL::jointsValid(const std::vector<double>& q) const {
  if (q.size() != names_.size() || !finite(q)) return false;
  for (size_t i=0; i<q.size(); ++i) {
    const auto& b = robot_model_->getVariableBounds(names_[i]);
    if (b.position_bounded_ && (q[i] < b.min_position_-1e-9 || q[i] > b.max_position_+1e-9)) return false;
  }
  return true;
}
bool IKFastWithKDL::matches(const Pose& target, const std::vector<double>& q) const {
  if (!jointsValid(q)) return false;
  std::vector<Pose> poses;
  if (!originalFK(tip_frames_, q, poses) || poses.size()!=1 || !validPose(poses[0])) return false;
  const auto& got = poses[0];
  const Eigen::Vector3d delta(got.position.x-target.position.x,
      got.position.y-target.position.y, got.position.z-target.position.z);
  Eigen::Quaterniond a(got.orientation.w, got.orientation.x, got.orientation.y, got.orientation.z);
  Eigen::Quaterniond b(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
  return delta.norm() <= 2e-5 && a.normalized().angularDistance(b.normalized()) <= 2e-5;
}
bool IKFastWithKDL::originalFK(const std::vector<std::string>& links,
    const std::vector<double>& q, std::vector<Pose>& poses) const {
  if (!jointsValid(q)) return false;
  for (const auto& link : links) if (!robot_model_->getLinkModel(link)) return false;
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(group_name_, q);
  state.update();
  const Eigen::Isometry3d inverse_base = state.getGlobalLinkTransform(base_frame_).inverse();
  poses.clear();
  for (const auto& link : links) {
    const Eigen::Isometry3d value = inverse_base * state.getGlobalLinkTransform(link);
    const Eigen::Quaterniond rotation(value.rotation());
    Pose p;
    p.position.x=value.translation().x(); p.position.y=value.translation().y(); p.position.z=value.translation().z();
    p.orientation.x=rotation.x(); p.orientation.y=rotation.y(); p.orientation.z=rotation.z(); p.orientation.w=rotation.w();
    poses.push_back(p);
  }
  return true;
}
bool IKFastWithKDL::getPositionFK(const std::vector<std::string>& links,
    const std::vector<double>& q, std::vector<Pose>& poses) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return active_ && originalFK(links, q, poses);
}
bool IKFastWithKDL::getPositionIK(const Pose& p, const std::vector<double>& seed,
    std::vector<double>& result, Error& error, const Options& options) const {
  return searchPositionIK(p, seed, getDefaultTimeout(), {}, result, IKCallbackFn(), error, options);
}
bool IKFastWithKDL::searchPositionIK(const Pose& p, const std::vector<double>& seed,
    double timeout, std::vector<double>& result, Error& error, const Options& options) const {
  return searchPositionIK(p, seed, timeout, {}, result, IKCallbackFn(), error, options);
}
bool IKFastWithKDL::searchPositionIK(const Pose& p, const std::vector<double>& seed,
    double timeout, const std::vector<double>& limits, std::vector<double>& result,
    Error& error, const Options& options) const {
  return searchPositionIK(p, seed, timeout, limits, result, IKCallbackFn(), error, options);
}
bool IKFastWithKDL::searchPositionIK(const Pose& p, const std::vector<double>& seed,
    double timeout, std::vector<double>& result, const IKCallbackFn& callback,
    Error& error, const Options& options) const {
  return searchPositionIK(p, seed, timeout, {}, result, callback, error, options);
}
bool IKFastWithKDL::searchPositionIK(const Pose& p, const std::vector<double>& seed,
    double timeout, const std::vector<double>& limits, std::vector<double>& result,
    const IKCallbackFn& callback, Error& error, const Options& options) const {
  const auto start = std::chrono::steady_clock::now();
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  result.clear();
  error.val = Error::NO_IK_SOLUTION;
  if (!active_ || !validPose(p) || !jointsValid(seed) || !std::isfinite(timeout) || timeout <= 0 ||
      (!limits.empty() && (limits.size()!=6 || !finite(limits) ||
       std::any_of(limits.begin(), limits.end(), [](double v){ return v < 0; })))) return false;
  // Both backends get the same validity/collision callback. The generated
  // six-DOF plugin does not itself enforce consistency limits on this path.
  const IKCallbackFn checked = [&](const Pose& candidate_pose, const std::vector<double>& q, Error& ec) {
    ec.val = Error::NO_IK_SOLUTION;
    if (!matches(p, q)) return;
    for (size_t i=0; i<limits.size(); ++i)
      if (std::abs(q[i]-seed[i]) > limits[i]+1e-9) return;
    ec.val = Error::SUCCESS;
    if (callback) callback(candidate_pose, q, ec);
  };
  Options exact = options;
  exact.return_approximate_solution = false;
  auto remaining = [&]() {
    return timeout - std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  };
  std::vector<double> candidate;
  if (remaining() <= 0) { error.val=Error::TIMED_OUT; return false; }
  if (primary_->searchPositionIK(p, seed, remaining(), limits, candidate, checked, error, exact)) {
    result = candidate; ++stats_.primary; return true;
  }
  if (remaining() <= 0) { error.val=Error::TIMED_OUT; return false; }
  ++stats_.fallback_calls;
  candidate.clear();
  if (fallback_->searchPositionIK(p, seed, remaining(), limits, candidate, checked, error, exact)) {
    result = candidate; ++stats_.fallback; return true;
  }
  return false;
}
IKFastWithKDL::Statistics IKFastWithKDL::statistics() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stats_;
}
}
PLUGINLIB_EXPORT_CLASS(fr5_kinematics::IKFastWithKDL, kinematics::KinematicsBase)
