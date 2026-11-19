#pragma once
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.hpp>
#include <mutex>

namespace fr5_kinematics {
// A small diagnostic interface lets tests inspect routing without linking the
// plugin library ahead of pluginlib's actual dynamic-load test.
class Diagnostics {
public:
  struct Statistics { unsigned long primary = 0, fallback_calls = 0, fallback = 0; };
  virtual ~Diagnostics() = default;
  virtual Statistics statistics() const = 0;
};
class IKFastWithKDL : public kinematics::KinematicsBase, public Diagnostics {
public:
  using Pose = geometry_msgs::msg::Pose;
  using Error = moveit_msgs::msg::MoveItErrorCodes;
  using Options = kinematics::KinematicsQueryOptions;
  bool initialize(const rclcpp::Node::SharedPtr&, const moveit::core::RobotModel&,
                  const std::string&, const std::string&, const std::vector<std::string>&, double) override;
  const std::vector<std::string>& getJointNames() const override { return names_; }
  const std::vector<std::string>& getLinkNames() const override { return links_; }
  bool getPositionFK(const std::vector<std::string>&, const std::vector<double>&,
                     std::vector<Pose>&) const override;
  bool getPositionIK(const Pose&, const std::vector<double>&, std::vector<double>&,
                     Error&, const Options& = Options()) const override;
  bool searchPositionIK(const Pose&, const std::vector<double>&, double,
                        std::vector<double>&, Error&, const Options& = Options()) const override;
  bool searchPositionIK(const Pose&, const std::vector<double>&, double, const std::vector<double>&,
                        std::vector<double>&, Error&, const Options& = Options()) const override;
  bool searchPositionIK(const Pose&, const std::vector<double>&, double, std::vector<double>&,
                        const IKCallbackFn&, Error&, const Options& = Options()) const override;
  bool searchPositionIK(const Pose&, const std::vector<double>&, double, const std::vector<double>&,
                        std::vector<double>&, const IKCallbackFn&, Error&,
                        const Options& = Options()) const override;
  Statistics statistics() const override;
private:
  // Backend objects must be destroyed before their library loader.
  std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader_;
  std::shared_ptr<kinematics::KinematicsBase> primary_, fallback_;
  std::vector<std::string> names_, links_;
  mutable std::recursive_mutex mutex_;
  mutable Statistics stats_;
  bool active_ = false;
  bool jointsValid(const std::vector<double>&) const;
  bool originalFK(const std::vector<std::string>&, const std::vector<double>&, std::vector<Pose>&) const;
  bool matches(const Pose&, const std::vector<double>&) const;
};
}  // namespace fr5_kinematics
