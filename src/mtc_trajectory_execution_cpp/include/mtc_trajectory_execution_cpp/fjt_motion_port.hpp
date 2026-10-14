// Copyright 2026 MTC contributors
#pragma once
#include <memory>
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <mtc_manipulation_cpp/pick_place_types.hpp>
#include "mtc_trajectory_execution_cpp/trajectory_validator.hpp"
#include "mtc_trajectory_execution_cpp/execution_state.hpp"
namespace mtc::trajectory
{
// A one-use, node/domain/endpoint-bound credential, minted before controlled bringup.
// Not DDS authentication: callers must own the launch sequence and trust the domain.
class ControlledFakeStart final
{
public:
  // Call before rclcpp::init so an unreviewed discovery server is never contacted.
  static void validate_environment();
  static std::shared_ptr<ControlledFakeStart> verify_empty_domain(
    const rclcpp::Node::SharedPtr & node, const Config & config);
private:
  friend class FjtMotionPort;
  ControlledFakeStart() = default;
  std::weak_ptr<rclcpp::Node> node_;
  size_t domain_ = 0;
  std::string endpoint_;
  std::chrono::steady_clock::time_point issued_;
  std::atomic<bool> consumed_{false};
};
// The supplied node must be spun by its owner's executor. No FSM callback is invoked here.
// Default disabled. Runtime GenericSystem, active controller and fresh state proofs required.
class FjtMotionPort final : public mtc::manipulation::MotionPort
{
public:
  FjtMotionPort(rclcpp::Node::SharedPtr node, Config config);
  // Explicit controlled-cold-start mode. The default constructor stays strict.
  FjtMotionPort(rclcpp::Node::SharedPtr node, Config config,
    const std::shared_ptr<ControlledFakeStart> & authority);
  ~FjtMotionPort() override;
  FjtMotionPort(const FjtMotionPort &) = delete;
  FjtMotionPort & operator=(const FjtMotionPort &) = delete;
  std::optional<mtc::core::MotionResult> execute_joint_move(
    const mtc::core::MotionCommand & command) override;
  std::optional<mtc::core::MotionResult> execute_trajectory(
    const mtc::core::MotionCommand & command) override;
  // Polling stop: direct FSM use conservatively faults if confirmation isn't ready yet.
  mtc::core::StopResult request_stop(const std::string & reason) override;
  // Additional scoped API; never upgrades mismatched/old IDs to global cancel.
  mtc::core::StopResult cancel_command(const std::string & id, const std::string & reason);
  ExecutionSnapshot snapshot() const;
private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace mtc::trajectory
