#pragma once
#include <moveit/controller_manager/controller_manager.hpp>
namespace mtc::integration {
// MoveItCpp constructs TEM even in planning-only mode. This plugin exposes no
// controllers/handles/clients and cannot execute or switch any controller.
class PlanningOnlyManager final : public moveit_controller_manager::MoveItControllerManager {
public:
  void initialize(const rclcpp::Node::SharedPtr &) override {}
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &) override {return nullptr;}
  void getControllersList(std::vector<std::string> & names) override {names.clear();}
  void getActiveControllers(std::vector<std::string> & names) override {names.clear();}
  void getControllerJoints(const std::string &, std::vector<std::string> & names) override {names.clear();}
  ControllerState getControllerState(const std::string &) override {return {};}
  bool switchControllers(const std::vector<std::string> &,const std::vector<std::string> &) override {return false;}
};
}
