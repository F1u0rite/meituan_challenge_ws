// Non-contact, fresh-measured-state integration; no perception/tool success is fabricated.
#include "mtc_aubo_fake_integration_cpp/aubo_fjt_motion_port.hpp"
#include <mtc_moveit_planner/move_group_backend.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <nlohmann/json.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <set>
#include <filesystem>
#include <urdf/model.h>
using namespace std::chrono_literals;
using Json=nlohmann::json;
namespace a=mtc::aubo_fake;
namespace p=mtc::moveit_planner;
namespace t=mtc::trajectory;
static double steady() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static bool wait(const std::function<bool()> & ready,double timeout) {
  const auto end=steady()+timeout;
  while(rclcpp::ok()&&steady()<end) {if(ready())return true;std::this_thread::sleep_for(5ms);}return ready();
}
static Json snapshot(a::AuboFjtMotionPort & port) {
  auto s=port.snapshot();
  Json j={{"goal_count",s.goals_sent},{"cancel_count",s.cancel_requests},{"goal_uuid",s.goal_uuid},
    {"command_id",s.command_id},{"feedback_count",s.feedback_samples},{"hardware_plugin",s.hardware_plugin},
    {"hardware_verified",s.hardware_verified},{"controller_verified",s.controller_verified},{"ready",s.ready},
    {"fault_locked",s.fault_locked},{"cancel_acknowledged",s.cancel_acknowledged},{"terminal_received",s.terminal_received},
    {"status_received",s.status_received},{"status_fresh",s.status_fresh},{"foreign_goals",s.foreign_active_goals},
    {"stationary_samples",s.stationary_samples},{"stationary_seconds",s.stationary_observed_seconds},{"detail",s.detail}};
  j["stop_confirmed"]=s.result?Json(s.result->stop_confirmed):Json(nullptr);
  return j;
}
struct State {
  std::mutex mutex;std::optional<t::JointSample> sample;double received=0;
  std::ofstream trace;std::set<std::string> status_uuids;
  std::vector<double> minimum,maximum,maximum_velocity;
};
// Planner ROS callbacks and potentially blocking MoveGroup state/service calls
// must not share the execution owner's Node/default callback group/Clock.
struct PlannerRuntime {
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread worker;
  explicit PlannerRuntime(const rclcpp::Node::SharedPtr & owner)
  :node(std::make_shared<rclcpp::Node>("aubo_fake_planner",
    rclcpp::NodeOptions(owner->get_node_options()).automatically_declare_parameters_from_overrides(true))),
    executor(rclcpp::ExecutorOptions(),2) {
    executor.add_node(node);worker=std::thread([this]{executor.spin();});
  }
  ~PlannerRuntime() {executor.cancel();if(worker.joinable())worker.join();}
};
static bool run(rclcpp::Node::SharedPtr node,a::AuboFjtMotionPort & port,t::Config cfg,
  rclcpp::executors::MultiThreadedExecutor & executor,Json & report) {
  auto state=std::make_shared<State>();state->trace.open(node->get_parameter("output_dir").as_string()+"/joint_states.jsonl");
  auto sub=node->create_subscription<sensor_msgs::msg::JointState>(cfg.joint_state_topic,rclcpp::SensorDataQoS(),
    [state,cfg](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      auto mapped=t::map_sample(*msg,cfg);std::lock_guard<std::mutex> lock(state->mutex);state->sample=mapped;state->received=steady();
      if(!mapped)return;
      if(state->minimum.empty()) {state->minimum=state->maximum=mapped->positions;state->maximum_velocity.assign(6,0);}
      for(size_t i=0;i<6;++i) {state->minimum[i]=std::min(state->minimum[i],mapped->positions[i]);state->maximum[i]=std::max(state->maximum[i],mapped->positions[i]);
        state->maximum_velocity[i]=std::max(state->maximum_velocity[i],std::abs(mapped->velocities[i]));}
      state->trace<<Json{{"stamp_ns",mapped->stamp_ns},{"monotonic_s",state->received},{"names",cfg.joints},
        {"positions",mapped->positions},{"velocities",mapped->velocities}}.dump()<<std::endl;
    });
  auto statuses=node->create_subscription<action_msgs::msg::GoalStatusArray>(cfg.action_name()+"/_action/status",
    rclcpp::QoS(1).reliable().transient_local(),[state](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mutex);
      for(const auto & s:msg->status_list) {std::string id;const char * hex="0123456789abcdef";
        for(auto b:s.goal_info.goal_id.uuid) {id+=hex[b>>4];id+=hex[b&15];}state->status_uuids.insert(id);}
    });
  auto current=[&]()->std::optional<t::JointSample> {std::lock_guard<std::mutex> lock(state->mutex);
    if(steady()-state->received>cfg.feedback_max_age) {return {};}
    return state->sample;};
  if(!wait([&]{return port.snapshot().ready&&current().has_value();},35))throw std::runtime_error("Fake hardware/controller/owner/state proof did not become ready");
  report["cold_start"]=snapshot(port);report["mode"]=node->get_parameter("mode").as_string();
  size_t servers=0,clients=0;
  for(const auto & n:node->get_node_graph_interface()->get_node_names_and_namespaces()) {
    servers+=node->get_service_names_and_types_by_node(n.first,n.second).count(cfg.action_name()+"/_action/send_goal");
    clients+=node->get_node_graph_interface()->get_client_names_and_types_by_node(n.first,n.second).count(cfg.action_name()+"/_action/send_goal");
  }
  report["graph"]={{"fjt_servers",servers},{"fjt_client_nodes",clients},
    {"direct_trajectory_publishers",node->count_publishers("/"+cfg.controller+"/joint_trajectory")},
    {"joint_state_publishers",node->count_publishers(cfg.joint_state_topic)},
    {"limitation","graph counts are deployment evidence, not authentication or cross-process atomic mutual exclusion"}};
  PlannerRuntime planning(node);
  report["planner_node"]="aubo_fake_planner";report["execution_node"]="aubo_fake_owner";
  robot_model_loader::RobotModelLoader loader(planning.node,"robot_description",false);auto model=loader.getModel();
  if(!model||model->getModelFrame()!="world"||model->getVariableCount()!=6)throw std::runtime_error("MoveIt model/frame does not match reviewed Fake model");
  auto backend=std::make_shared<p::MoveGroupBackend>(planning.node,p::MoveGroupBackendOptions{});
  std::cout<<"STAGE backend_ready"<<std::endl;
  if(backend->planning_frame()!=model->getModelFrame()||backend->end_effector_link()!="ee_link")throw std::runtime_error("Planner model/frame/TCP mismatch");
  std::cout<<"STAGE current_state_begin"<<std::endl;
  auto start=backend->current_state();if(!start||!start->shape_is_valid())throw std::runtime_error("no current measured planning state");
  std::cout<<"STAGE current_state_end"<<std::endl;
  auto observed=current();if(!observed)throw std::runtime_error("stale measured state before planning");
  std::map<std::string,double> measured;
  for(size_t i=0;i<cfg.joints.size();++i)measured[cfg.joints[i]]=observed->positions[i];
  for(size_t i=0;i<start->names.size();++i)if(std::abs(start->positions[i]-measured.at(start->names[i]))>cfg.start_tolerance)throw std::runtime_error("Planner/current state mismatch");
  report["measured_start"]={{"names",start->names},{"positions",start->positions},{"stamp_ns",observed->stamp_ns}};
  // Every target derives from this run's measured state; no home/old Gazebo trajectory.
  auto goal_state=*start;const std::map<std::string,double> offset={{"shoulder_joint",.02},{"upperArm_joint",-.04},{"foreArm_joint",.08},{"wrist1_joint",-.03},{"wrist2_joint",.02},{"wrist3_joint",.01}};
  for(size_t i=0;i<goal_state.names.size();++i)goal_state.positions[i]+=offset.at(goal_state.names[i]);
  auto goal=backend->end_effector_pose(goal_state);if(!goal)throw std::runtime_error("FK target unavailable");
  std::cout<<"STAGE fk_end"<<std::endl;
  p::BackendRequest request;request.label="aubo_fake_noncontact_probe";request.start=*start;request.ee_goal=*goal;
  request.velocity_scaling=.1;request.acceleration_scaling=.05;request.planning_time_s=3;
  // FREE_SPACE probe does not need the optional Cartesian service optimization.
  // Installed 2.12.4 may drop its response and the upstream wrapper waits without
  // a deadline. Joint-space IK/OMPL retains collision, path bounds and EE limits.
  request.prefer_straight_line=false;request.max_linear_speed_m_s=.03;request.max_angular_speed_rad_s=.1;
  report["optional_cartesian_preference"]=false;
  for(const auto & name:cfg.joints) {const auto & b=cfg.limits.at(name);request.joint_bounds[name]={std::max(b.minimum,measured.at(name)-.15),std::min(b.maximum,measured.at(name)+.15)};}
  std::cout<<"STAGE plan_begin"<<std::endl;
  const auto plan=backend->plan(request);const double generated=steady();
  std::cout<<"STAGE plan_end"<<std::endl;
  report["plan"]={{"ok",plan.ok},{"reason",plan.reason},{"message",plan.message},{"method",plan.method},{"collision_check_enabled",true}};
  if(!plan.ok)throw std::runtime_error("planning failed: "+plan.reason+" "+plan.message);
  auto command=a::command_from_plan(plan.trajectory,cfg,backend->planning_frame(),node->get_parameter("session_id").as_string()+"/command/1",3.0);
  Json saved={{"command_id",command.command_id},{"trajectory_id",command.trajectory->trajectory_id},{"frame",command.trajectory->header_frame_id},
    {"generated_monotonic_s",generated},{"ros_execution_stamp_s",command.trajectory->stamp_s},{"explicit_uniform_slowdown",3.0},
    {"joint_names",command.trajectory->joint_names},{"timeout_ms",command.timeout_ms},{"points",Json::array()}};
  for(const auto & q:command.trajectory->points)saved["points"].push_back({{"t",q.time_from_start_s},{"positions",q.positions_rad},{"velocities",q.velocities_rad_s},{"accelerations",q.accelerations_rad_s2}});
  std::ofstream(node->get_parameter("output_dir").as_string()+"/trajectory.json")<<saved.dump(2);
  report["trajectory_points"]=command.trajectory->points.size();report["duration_s"]=command.trajectory->duration_s();
  auto mode=node->get_parameter("mode").as_string();
  if(mode=="plan") {report["execution"]=snapshot(port);return port.snapshot().goals_sent==0&&!port.snapshot().fault_locked;}
  if(mode=="expired") {
    const bool rejected=!a::plan_is_fresh(generated-10,steady());
    report["expired_plan_record"]={{"generated_monotonic_s",generated-10},{"checked_monotonic_s",steady()},{"rejected",rejected}};
    report["execution"]=snapshot(port);return rejected&&port.snapshot().goals_sent==0;
  }
  if(mode=="foreign") {
    auto foreign_node=std::make_shared<rclcpp::Node>("aubo_fake_foreign_client");executor.add_node(foreign_node);
    auto foreign=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(foreign_node,cfg.action_name());
    const bool locked=wait([&]{return port.snapshot().fault_locked;},3);auto r=port.execute_trajectory(command);
    report["execution"]=snapshot(port);executor.remove_node(foreign_node);
    return locked&&r.has_value()&&port.snapshot().goals_sent==0&&port.snapshot().cancel_requests==0;
  }
  if(mode=="mismatch")for(auto & point:command.trajectory->points)point.positions_rad[0]+=.04;
  if(!a::plan_is_fresh(generated,steady())||!current())throw std::runtime_error("plan/current-state expired before dispatch");
  auto result=port.execute_trajectory(command);
  if(mode=="mismatch") {report["execution"]=snapshot(port);report["rejection"]=result?result->message:"not rejected";
    return result&&result->exec_state==mtc::core::ExecState::REJECTED&&port.snapshot().goals_sent==0;}
  if(result)throw std::runtime_error("dispatch rejected: "+result->message);
  if(mode=="cancel") {
    if(!wait([&]{return port.snapshot().feedback_samples>0;},2))throw std::runtime_error("no associated feedback before cancel");
    report["cancel_immediate"]=port.cancel_command(command.command_id,"own Fake Goal only").stop_confirmed;
  }
  std::shared_ptr<rclcpp::Node> intruder; rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr direct;
  if(mode=="fault") {
    if(!wait([&]{return port.snapshot().feedback_samples>0;},2))throw std::runtime_error("no associated feedback before fault");
    intruder=std::make_shared<rclcpp::Node>("aubo_fake_conflicting_publisher");executor.add_node(intruder);
    direct=intruder->create_publisher<trajectory_msgs::msg::JointTrajectory>("/"+cfg.controller+"/joint_trajectory",10); // NO message is sent.
  }
  if(!wait([&]{result=port.execute_trajectory(command);return result.has_value();},command.timeout_ms/1000.0+2))throw std::runtime_error("no terminal outcome before bounded deadline");
  if(mode=="fault") {wait([&]{return port.snapshot().terminal_received;},2);executor.remove_node(intruder);}
  report["execution"]=snapshot(port);report["result"]={{"success",result->success},{"stop_confirmed",result->stop_confirmed},
    {"status_unknown",result->status_unknown},{"timed_out",result->timed_out},{"error_code",result->error_code},{"message",result->message},
    {"final_positions",result->final_position_rad}};
  if(auto final=current()) {
    report["final_positions"]=final->positions;report["final_velocities"]=final->velocities;
    report["final_error_rad"]=Json::array();double maximum=0;
    auto wire=t::convert(command,cfg);for(size_t i=0;i<cfg.joints.size();++i) {const double error=std::abs(final->positions[i]-wire.trajectory->points.back().positions[i]);report["final_error_rad"].push_back(error);maximum=std::max(maximum,error);}report["max_final_error_rad"]=maximum;
  }
  {std::lock_guard<std::mutex> lock(state->mutex);report["joint_range_rad"]=Json::array();
    for(size_t i=0;i<6;++i)report["joint_range_rad"].push_back(state->maximum[i]-state->minimum[i]);
    report["max_reported_velocity_rad_s"]=state->maximum_velocity;report["native_status_goal_uuids"]=state->status_uuids;}
  if(mode=="cancel")return !result->success&&result->stop_confirmed&&port.snapshot().cancel_requests==1&&!port.snapshot().fault_locked;
  if(mode=="fault") {auto next=command;next.command_id+="-next";
    return result->status_unknown&&port.snapshot().fault_locked&&port.execute_trajectory(next).has_value()&&port.snapshot().goals_sent==1;}
  if(mode=="identity") {auto changed=command;changed.timeout_ms++;auto r=port.execute_trajectory(changed);
    report["identity_rejection"]=r?r->message:"none";return result->success&&result->stop_confirmed&&r&&r->exec_state==mtc::core::ExecState::REJECTED&&port.snapshot().goals_sent==1;}
  if(mode=="silence") {std::this_thread::sleep_for(900ms);report["after_status_silence"]=snapshot(port);return result->success&&result->stop_confirmed&&port.snapshot().ready;}
  return result->success&&result->stop_confirmed&&!port.snapshot().fault_locked&&port.snapshot().goals_sent==1;
}
int main(int argc,char ** argv) {
  Json report={{"functional_result","FAIL"},{"real_hardware",false},{"contact_motion",false}};
  std::string directory;int code=2;bool output_allowed=false;
  try {
    t::ControlledFakeStart::validate_environment();rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("aubo_fake_owner");
    directory=node->declare_parameter<std::string>("output_dir","");node->declare_parameter<std::string>("mode","plan");
    node->declare_parameter<std::string>("session_id","");
    const bool enabled=node->declare_parameter<bool>("enable_fake_execution",false);
    if(!enabled||directory.empty()||node->get_parameter("session_id").as_string().empty())throw std::runtime_error("explicit Fake mode/session/output required");
    for(const auto * file:{"functional.json","trajectory.json","joint_states.jsonl"}) {
      if(std::filesystem::exists(std::filesystem::path(directory)/file))throw std::runtime_error("refusing to overwrite existing probe evidence");
    }
    output_allowed=true;
    // All MoveIt loaders (including their logger helpers) are deferred until
    // the credential is consumed. URDF parsing here creates no ROS entities.
    urdf::Model model;
    const auto xml=node->declare_parameter<std::string>("robot_description","");
    if(!model.initString(xml)||!model.getLink("ee_link"))throw std::runtime_error("reviewed S3 URDF/TCP required");
    t::Config cfg;cfg.enable_fake_execution=true;cfg.settle_duration=.25;
    const double controller_max[]={2*M_PI,M_PI,2.41,2.66,2*M_PI,2.23};
    for(size_t i=0;i<cfg.joints.size();++i) {const auto & name=cfg.joints[i];const auto joint=model.getJoint(name);
      if(!joint||joint->type!=urdf::Joint::REVOLUTE||!joint->limits)throw std::runtime_error("model joint limits absent");
      const auto & bounds=*joint->limits;
      cfg.limits[name]={std::max(bounds.lower,-controller_max[i]),std::min(bounds.upper,controller_max[i]),std::min(bounds.velocity,.2),.2};}
    cfg.validate();report["planning_frame"]="world";report["tcp"]="ee_link (explicit bare-tool Fake FK; not calibrated V1 TCP)";
    report["limits"]=Json::object();
    for(const auto & entry:cfg.limits) {const auto & b=entry.second;report["limits"][entry.first]={{"min_rad",b.minimum},{"max_rad",b.maximum},{"max_velocity_rad_s",b.velocity},{"max_acceleration_rad_s2",b.acceleration}};}
    auto credential=t::ControlledFakeStart::verify_empty_domain(node,cfg);
    auto port=std::make_unique<a::AuboFjtMotionPort>(node,cfg,credential,"world");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);executor.add_node(node);
    std::thread worker([&]{executor.spin();});
    std::cout<<"AUBO_FAKE_COLD_START_READY"<<std::endl;
    try {const bool pass=run(node,*port,cfg,executor,report);report["functional_result"]=pass?"PASS":"FAIL";code=pass?0:2;}
    catch(const std::exception & e) {report["error"]=e.what();report["execution"]=snapshot(*port);std::cerr<<e.what()<<std::endl;}
    port.reset();executor.cancel();worker.join();
  } catch(const std::exception & e) {report["error"]=e.what();std::cerr<<e.what()<<std::endl;}
  std::cout<<report.dump()<<std::endl;
  if(output_allowed)std::ofstream(directory+"/functional.json")<<report.dump(2);
  if(rclcpp::ok()) {rclcpp::shutdown();}
  return code;
}
