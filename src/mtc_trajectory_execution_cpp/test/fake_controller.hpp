#pragma once
// Real ROS Action transport, entirely local software. No robot SDK or driver.
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <action_msgs/msg/goal_status_array.hpp>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "mtc_trajectory_execution_cpp/fjt_motion_port.hpp"
using namespace std::chrono_literals;
using namespace mtc::trajectory;
using Fjt=control_msgs::action::FollowJointTrajectory;
using ServerHandle=rclcpp_action::ServerGoalHandle<Fjt>;
using Hw=controller_manager_msgs::srv::ListHardwareComponents;
using Cs=controller_manager_msgs::srv::ListControllers;
class FakeController : public rclcpp::Node
{
public:
  Config cfg;
  std::atomic<size_t> goals{0}, cancels{0}, last_state_count{0};
  std::atomic<bool> reject{false}, aborted{false}, ack_only{false}, no_result{false};
  std::atomic<bool> publish_state{true}, stale_state{false}, missing_velocity{false};
  std::atomic<bool> moving{false}, wrong_final{false}, feedback_enabled{true}, active_controller{true};
  std::atomic<bool> generic{true}, repeated_stamp{false};
  std::atomic<bool> publish_status{true}, hold_response{false}, pending_status{false};
  std::atomic<bool> hide_active_status{false}, invalid_status{false}, reject_cancel{false}, hold_terminal_status{false};
  std::atomic<size_t> status_publications{0}, pending_requests{0};
  void release_response() {pending_status=false; {std::lock_guard<std::mutex> lock(gate_mutex); released=true;} gate.notify_all();}
  void release_terminal_status() {std::lock_guard<std::mutex> lock(mutex); hold_terminal_status=false; status_cache=latest_status;}
  size_t active_statuses() {std::lock_guard<std::mutex> lock(mutex); return std::count_if(status_cache.status_list.begin(),status_cache.status_list.end(),[](const auto & s) {return s.status>=1 && s.status<=3;});}
  void publish_latched_empty() {status_pub->publish(action_msgs::msg::GoalStatusArray{});}
  size_t terminal_statuses() {std::lock_guard<std::mutex> lock(mutex); return std::count_if(status_cache.status_list.begin(),status_cache.status_list.end(),[](const auto & s) {return s.status>=4;});}
  rclcpp_action::GoalUUID request_id() {std::lock_guard<std::mutex> lock(mutex); return pending_uuid;}
  FakeController(Config c):Node("fake_controller"),cfg(std::move(c))
  {
    hw=create_service<Hw>(cfg.controller_manager+"/list_hardware_components",
      [this](std::shared_ptr<Hw::Request>,std::shared_ptr<Hw::Response> r) {
        controller_manager_msgs::msg::HardwareComponentState h; h.name=cfg.hardware_component;
        h.plugin_name=generic ? "mock_components/GenericSystem" : "aubo_driver/AuboHardwareInterface";
        h.state.id=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        for (auto & n:cfg.joints) {for (auto suffix:{"/position","/velocity"}) {
          controller_manager_msgs::msg::HardwareInterface i; i.name=n+suffix; i.is_available=true; h.state_interfaces.push_back(i);
        }} r->component.push_back(h);
      });
    cs=create_service<Cs>(cfg.controller_manager+"/list_controllers",
      [this](std::shared_ptr<Cs::Request>,std::shared_ptr<Cs::Response> r) {
        controller_manager_msgs::msg::ControllerState c; c.name=cfg.controller;
        c.type="joint_trajectory_controller/JointTrajectoryController"; c.state=active_controller ? "active" : "inactive";
        for (auto & j:cfg.joints) {c.claimed_interfaces.push_back(j+"/position");} r->controller.push_back(c);
        c=controller_manager_msgs::msg::ControllerState(); c.name="joint_state_broadcaster"; c.state="active"; c.type="joint_state_broadcaster/JointStateBroadcaster"; r->controller.push_back(c);
      });
    pub=create_publisher<sensor_msgs::msg::JointState>(cfg.joint_state_topic,rclcpp::SensorDataQoS());
    status_pub=create_publisher<action_msgs::msg::GoalStatusArray>(cfg.action_name()+"/test_status_evidence",
      rclcpp::QoS(1).reliable().transient_local());
    native_status=create_subscription<action_msgs::msg::GoalStatusArray>(cfg.action_name()+"/_action/status",
      rclcpp::QoS(1).reliable().transient_local(),[this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex); latest_status=*msg;
        if(!hold_terminal_status || std::none_of(msg->status_list.begin(),msg->status_list.end(),
          [](const auto & s) {return s.status>=4;})) {status_cache=*msg;}
      });
    action_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server=rclcpp_action::create_server<Fjt>(this,cfg.action_name(),
      [this](const auto & uuid,auto goal) {
        ++goals;
        if(hold_response && goal->trajectory.header.frame_id!="foreign") {
          {std::lock_guard<std::mutex> lock(mutex); pending_uuid=uuid;}
          ++pending_requests;
          std::unique_lock<std::mutex> lock(gate_mutex);
          gate.wait_for(lock,2s,[this]() {return released;});
        }
        return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto h) {++cancels; std::lock_guard<std::mutex> lock(mutex); canceled_uuid=h->get_goal_id(); return reject_cancel ? rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto h) {std::lock_guard<std::mutex> lock(mutex); handle=h; received=h->get_goal()->trajectory; began=std::chrono::steady_clock::now();},rcl_action_server_get_default_options(),action_group);
    timer=create_wall_timer(10ms,[this]() {tick();});
  }
  trajectory_msgs::msg::JointTrajectory trajectory() {std::lock_guard<std::mutex> lock(mutex); return received;}
  rclcpp_action::GoalUUID goal_id() {std::lock_guard<std::mutex> lock(mutex); return handle ? handle->get_goal_id() : rclcpp_action::GoalUUID{};}
  rclcpp_action::GoalUUID cancel_id() {std::lock_guard<std::mutex> lock(mutex); return canceled_uuid;}
private:
  std::mutex gate_mutex;
  std::condition_variable gate;
  bool released=false;
  rclcpp_action::GoalUUID pending_uuid{};
  rclcpp::CallbackGroup::SharedPtr action_group;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr native_status;
  action_msgs::msg::GoalStatusArray status_cache,latest_status;
  std::mutex mutex;
  std::shared_ptr<ServerHandle> handle;
  trajectory_msgs::msg::JointTrajectory received;
  rclcpp_action::GoalUUID canceled_uuid{};
  std::vector<double> position=std::vector<double>(6,0.0);
  std::chrono::steady_clock::time_point began;
  rclcpp::Time last_stamp{0,0,RCL_ROS_TIME};
  rclcpp::Service<Hw>::SharedPtr hw;
  rclcpp::Service<Cs>::SharedPtr cs;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
  rclcpp_action::Server<Fjt>::SharedPtr server;
  rclcpp::TimerBase::SharedPtr timer;
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex);
    auto stamp=now(); if (stale_state) {stamp=stamp-rclcpp::Duration::from_seconds(2);}
    if (repeated_stamp && last_stamp.nanoseconds()) {stamp=last_stamp;} else {last_stamp=stamp;}
    if (handle) {
      auto f=std::make_shared<Fjt::Feedback>(); f->header.stamp=stamp; f->joint_names=cfg.joints;
      f->actual.positions=position; f->actual.velocities.assign(6,moving ? 0.1 : 0.0);
      if (feedback_enabled) {handle->publish_feedback(f);}
      auto r=std::make_shared<Fjt::Result>();
      if (handle->is_canceling() && !ack_only) {r->error_code=Fjt::Result::SUCCESSFUL; handle->canceled(r); handle.reset();}
      else if (!no_result && !ack_only && std::chrono::steady_clock::now()-began>150ms) {
        if (aborted) {r->error_code=Fjt::Result::PATH_TOLERANCE_VIOLATED; r->error_string="simulated controller abort"; handle->abort(r);}
        else {
          if (!wrong_final) {position=received.points.back().positions;}
          r->error_code=Fjt::Result::SUCCESSFUL; handle->succeed(r);
        } handle.reset();
      }
    }
    if(publish_status) {
      auto evidence=status_cache;
      if(pending_status && pending_requests) {
        action_msgs::msg::GoalStatus status; status.goal_info.goal_id.uuid=pending_uuid;
        status.goal_info.stamp=now(); status.status=action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
        if(std::none_of(evidence.status_list.begin(),evidence.status_list.end(),[&](const auto & x) {
          return x.goal_info.goal_id.uuid==pending_uuid;
        })) {evidence.status_list.push_back(status);}
      }
      if(hide_active_status) {
        evidence.status_list.erase(std::remove_if(evidence.status_list.begin(),evidence.status_list.end(),
          [](const auto & s) {return s.status>=1 && s.status<=3;}),evidence.status_list.end());
      }
      if(invalid_status) {
        action_msgs::msg::GoalStatus bad; bad.status=action_msgs::msg::GoalStatus::STATUS_EXECUTING;
        bad.goal_info.stamp=now(); evidence.status_list.push_back(bad); // Invalid zero UUID.
      }
      status_pub->publish(evidence); ++status_publications;
    }
    if (publish_state) {
      sensor_msgs::msg::JointState s; s.header.stamp=stamp; s.name=cfg.joints; s.position=position;
      if (!missing_velocity) {s.velocity.assign(6,moving ? 0.1 : 0.0);} pub->publish(s); ++last_state_count;
    }
  }
};
