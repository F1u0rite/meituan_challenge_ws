// Synthetic, non-contact software integration. Never instantiate Task/PickPlace FSMs.
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <mtc_core_cpp/clock.hpp>
#include <mtc_moveit_planner/move_group_backend.hpp>
#include <mtc_moveit_planner/moveit_planner_port.hpp>
#include <mtc_trajectory_execution_cpp/fjt_motion_port.hpp>
#include "mtc_phase2c_integration_cpp/dispatch_adapter.hpp"
using namespace std::chrono_literals;
using namespace mtc::integration;
using Port=mtc::trajectory::FjtMotionPort;
using Fjt=control_msgs::action::FollowJointTrajectory;
using Steady=std::chrono::steady_clock;
static double now_s() {return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
static bool wait_for(const std::function<bool()> & condition,double seconds) {
  const double end=now_s()+seconds;
  while(rclcpp::ok() && now_s()<end) {if(condition()) {return true;} std::this_thread::sleep_for(5ms);}
  return condition();
}
static void graph(const rclcpp::Node::SharedPtr & node,Port & port,const mtc::trajectory::Config & cfg) {
  size_t servers=0,clients=0;
  for(const auto & n:node->get_node_graph_interface()->get_node_names_and_namespaces()) {
    servers+=node->get_service_names_and_types_by_node(n.first,n.second).count(cfg.action_name()+"/_action/send_goal");
    if(node->get_node_graph_interface()->get_client_names_and_types_by_node(n.first,n.second).count(cfg.action_name()+"/_action/send_goal")) {
      ++clients;std::cout<<"FJT_CLIENT="<<n.second<<"/"<<n.first<<std::endl;
    }
  }
  auto s=port.snapshot();
  std::cout<<"HARDWARE plugin="<<s.hardware_plugin<<" verified="<<s.hardware_verified<<" controllers_verified="<<s.controller_verified<<std::endl;
  std::cout<<"GRAPH servers="<<servers<<" clients="<<clients<<" direct_publishers="<<node->count_publishers("/"+cfg.controller+"/joint_trajectory")
    <<" foreign="<<s.foreign_active_goals<<" status_received="<<s.status_received<<" status_fresh="<<s.status_fresh<<" ready="<<s.ready<<" controlled="<<s.controlled_start<<std::endl;
}
static int run(const rclcpp::Node::SharedPtr & node,Port & port,mtc::trajectory::Config cfg,
 rclcpp::executors::MultiThreadedExecutor & executor) {
  struct State {std::mutex mutex;std::optional<mtc::trajectory::JointSample> sample;};
  auto state_store=std::make_shared<State>();
  auto subscription=node->create_subscription<sensor_msgs::msg::JointState>(cfg.joint_state_topic,rclcpp::SensorDataQoS(),
    [state_store,cfg](sensor_msgs::msg::JointState::ConstSharedPtr msg) {auto v=mtc::trajectory::map_sample(*msg,cfg);std::lock_guard<std::mutex> lock(state_store->mutex);state_store->sample=v;});
  // Callback owns shared storage, including during exceptional teardown.
  auto current=[&]() {std::lock_guard<std::mutex> lock(state_store->mutex);return state_store->sample;};
  const std::string mode=node->get_parameter("mode").as_string();
  std::cout<<"COLD_START_CREDENTIAL_CONSUMED"<<std::endl;
  if(!wait_for([&]() {return port.snapshot().ready && current().has_value();},40)) {
    graph(node,port,cfg);throw std::runtime_error("BLOCKED: controlled Fake environment did not become ready");
  }
  graph(node,port,cfg);
  if(!wait_for([&]() {for(const auto & n:node->get_node_graph_interface()->get_node_names_and_namespaces()) {if(n.first=="move_group") {return true;}}return false;},20)) {
    throw std::runtime_error("planning-only move_group not discovered");
  }
  auto backend=std::make_shared<mtc::moveit_planner::MoveGroupBackend>(node,mtc::moveit_planner::MoveGroupBackendOptions{});
  auto start=backend->current_state();if(!start) {throw std::runtime_error("no current planning state");}
  // Small, explicit synthetic joint offset FK; no calibrated ring/slot or perception result.
  auto synthetic=start;
  for(size_t i=0;i<synthetic->names.size();++i) {
    if(synthetic->names[i]=="shoulder_joint") {synthetic->positions[i]+=0.02;}
    if(synthetic->names[i]=="upperArm_joint") {synthetic->positions[i]-=0.05;}
    if(synthetic->names[i]=="foreArm_joint") {synthetic->positions[i]+=0.08;}
  }
  if(!synthetic) {throw std::runtime_error("synthetic SRDF home state absent");}
  auto goal=backend->end_effector_pose(*synthetic);if(!goal) {throw std::runtime_error("synthetic FK failed");}
  auto pc=mtc::moveit_planner::default_planner_config();
  pc.joint_names=cfg.joints;pc.velocity_scaling=0.1;pc.acceleration_scaling=0.05;
  pc.planning_time_s=3;pc.state_timeouts.clear();pc.force_simulation_only=true;
  pc.primitives.clear();pc.tool_owned_primitives.clear();
  mtc::moveit_planner::PrimitiveSpec spec;spec.goal=mtc::moveit_planner::GoalKind::RING_STANDOFF;spec.distance_m=0;
  pc.primitives["synthetic_free_space"]=spec;
  for(const auto & j:cfg.joints) {pc.joint_position_limits[j]={cfg.limits.at(j).minimum,cfg.limits.at(j).maximum};}
  auto clock=std::make_shared<mtc::core::SteadyClock>();
  mtc::moveit_planner::MoveItPlannerPort planner(backend,pc,clock,false);
  mtc::manipulation::PlanTarget target;target.phase="synthetic_phase2c";
  mtc::core::MotionPrimitive primitive;primitive.name="synthetic_free_space";primitive.target_pose=goal;
  primitive.max_linear_speed=0.03;primitive.max_angular_speed=0.1;target.primitives={primitive};
  mtc::core::MotionCommand planned;
  bool compatible=false;
  DispatchAdapter capability_check(cfg,"pre-dispatch-capability-check",5.0);
  for(int attempt=0;attempt<3;++attempt) {
    if(port.snapshot().fault_locked) {throw std::runtime_error("authority fault before planning; no retry permitted");}
    auto result=planner.plan_blocking(target,start);
    if(!result.ok || result.commands.size()!=1 || !result.commands[0].trajectory) {
      std::cout<<"PLAN_REJECTED attempt="<<attempt<<" reason="<<result.message<<" Goals=0"<<std::endl;continue;
    }
    planned=result.commands[0];const auto state=current();
    if(!state) {throw std::runtime_error("current state unavailable during capability check");}
    const PlanningRecord original{planned,GenerationClock::STEADY,planned.trajectory->stamp_s};
    const auto capability=capability_check.prepare(original,DispatchPolicy::IMMEDIATE,*state,clock->now(),node->now().seconds(),0.0,3.0);
    if(capability.command) {compatible=true;break;}
    std::cout<<"PLAN_CAPABILITY_REJECTED attempt="<<attempt<<" duration="<<planned.trajectory->duration_s()<<" reason="<<capability.error<<" Goals=0"<<std::endl;
  }
  if(!compatible) {throw std::runtime_error("BLOCKED: no plan within reviewed execution capability; no Goal dispatched");}
  std::cout<<"PLAN_ONLY_OK points="<<planned.trajectory->points.size()<<" duration="<<planned.trajectory->duration_s()
    <<" generation_steady="<<planned.trajectory->stamp_s<<" ROS_now="<<node->now().seconds()<<" goals="<<port.snapshot().goals_sent<<std::endl;
  graph(node,port,cfg);
  // MGI requires this compatibility Action to exist at construction. Its capability
  // has no execution manager when allow_trajectory_execution=false; verify real denial.
  using Execute=moveit_msgs::action::ExecuteTrajectory;
  auto deny_client=rclcpp_action::create_client<Execute>(node,"/execute_trajectory");
  if(!deny_client->wait_for_action_server(3s)) {throw std::runtime_error("planning-only compatibility Action missing");}
  Execute::Goal denied_goal;
  auto & denied_wire=denied_goal.trajectory.joint_trajectory;
  denied_wire.joint_names=planned.trajectory->joint_names;denied_wire.header.frame_id=planned.trajectory->header_frame_id;
  for(const auto & p:planned.trajectory->points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;point.positions=p.positions_rad;
    point.velocities=p.velocities_rad_s;point.accelerations=p.accelerations_rad_s2;
    point.time_from_start=rclcpp::Duration::from_seconds(p.time_from_start_s);denied_wire.points.push_back(point);
  }
  auto response=deny_client->async_send_goal(denied_goal);
  if(response.wait_for(3s)!=std::future_status::ready || !response.get()) {throw std::runtime_error("execution denial did not respond");}
  auto denied_result=deny_client->async_get_result(response.get());
  if(denied_result.wait_for(3s)!=std::future_status::ready) {throw std::runtime_error("execution denial did not terminate");}
  const auto denied=denied_result.get();
  if(denied.code!=rclcpp_action::ResultCode::ABORTED || !denied.result ||
    denied.result->error_code.val!=moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED || port.snapshot().goals_sent!=0) {
    throw std::runtime_error("MoveIt unexpectedly exposes execution");
  }
  std::cout<<"MOVEIT_EXECUTION_DENIED controller_failed Goals=0"<<std::endl;
  if(mode=="plan") {std::cout<<"PASS plan Goals=0 Cancel=0"<<std::endl;executor.cancel();return 0;}
  if(mode=="expired") {planned.trajectory->stamp_s-=10;}
  if(mode=="mismatch") {for(auto & point:planned.trajectory->points) {point.positions_rad[0]+=0.1;}}
  PlanningRecord record{planned,GenerationClock::STEADY,planned.trajectory->stamp_s};
  DispatchAdapter dispatch(cfg,"phase2c-"+std::to_string(node->now().nanoseconds()),5.0);
  auto state=current();if(!state) {throw std::runtime_error("current state lost");}
  // Explicitly reviewed uniform slowdown: spatial path unchanged, t*=3, v/=3, a/=9.
  // Revalidate every point, Bernstein segment, actual start and recomputed timeout.
  auto prepared=dispatch.prepare(record,DispatchPolicy::IMMEDIATE,*state,clock->now(),node->now().seconds(),0.0,3.0);
  if(mode=="expired" || mode=="mismatch") {
    const std::string expected=mode=="expired" ? "plan generation time is future or expired" : "planned start differs from current named state";
    bool pass=!prepared.command && prepared.error==expected && port.snapshot().goals_sent==0;
    std::cout<<(pass ? "PASS " : "FAIL ")<<mode<<" Goals="<<port.snapshot().goals_sent<<" Cancel="<<port.snapshot().cancel_requests<<" reason="<<prepared.error<<std::endl;
    executor.cancel();return pass ? 0 : 1;
  }
  if(!prepared.command) {throw std::runtime_error("DISPATCH REJECTED: "+prepared.error);}
  const auto command=*prepared.command;
  std::cout<<"DISPATCH policy=IMMEDIATE original_stamp="<<record.original.trajectory->stamp_s<<" dispatch_stamp="<<command.trajectory->stamp_s
    <<" retime_factor=3 duration="<<command.trajectory->duration_s()<<" timeout_ms="<<command.timeout_ms<<" command_id="<<command.command_id<<std::endl;
  rclcpp::Node::SharedPtr foreign_node;
  rclcpp_action::Client<Fjt>::SharedPtr foreign;
  if(mode=="foreign") {
    foreign_node=std::make_shared<rclcpp::Node>("unauthorized_fake_client");executor.add_node(foreign_node);
    foreign=rclcpp_action::create_client<Fjt>(foreign_node,cfg.action_name());
    if(!wait_for([&]() {return !port.snapshot().ready;},3)) {throw std::runtime_error("extra Client not rejected");}
    auto rejected=port.execute_trajectory(command);graph(node,port,cfg);
    const bool pass=rejected && !rejected->success && port.snapshot().goals_sent==0 && port.snapshot().cancel_requests==0;
    std::cout<<(pass ? "PASS " : "FAIL ")<<"foreign Goals=0 Cancel=0"<<std::endl;
    executor.cancel();return pass ? 0 : 1;
  }
  auto execution=port.execute_trajectory(command);bool canceled=false;
  const double begin=now_s();
  while(!execution && rclcpp::ok() && now_s()-begin<command.timeout_ms/1000.0+2) {
    const auto s=port.snapshot();
    if(mode=="cancel" && !canceled && s.feedback_samples>0) {
      auto stop=port.cancel_command(command.command_id,"explicit synthetic asynchronous cancellation");
      std::cout<<"CANCEL_REQUEST initial_stop_confirmed="<<stop.stop_confirmed<<" uuid="<<s.goal_uuid<<std::endl;canceled=true;
    }
    if(mode=="fault" && !foreign && s.feedback_samples>0) {
      foreign_node=std::make_shared<rclcpp::Node>("fault_injection_fake_client");executor.add_node(foreign_node);
      foreign=rclcpp_action::create_client<Fjt>(foreign_node,cfg.action_name());
    }
    execution=port.execute_trajectory(command);std::this_thread::sleep_for(5ms);
  }
  const auto s=port.snapshot();
  if(!execution) {throw std::runtime_error("execution did not conclude; no safety claim");}
  std::cout<<"RESULT success="<<execution->success<<" stop_confirmed="<<execution->stop_confirmed<<" status_unknown="<<execution->status_unknown
    <<" fault_locked="<<s.fault_locked<<" Goals="<<s.goals_sent<<" Cancel="<<s.cancel_requests<<" UUID="<<s.goal_uuid
    <<" feedback="<<s.feedback_samples<<" stationary_samples="<<s.stationary_samples<<" stationary_seconds="<<s.stationary_observed_seconds
    <<" message="<<execution->message<<std::endl;
  if(execution->final_position_rad.size()==cfg.joints.size()) {
    std::cout<<"FINAL_NAMED_JOINTS actual/goal ";
    for(size_t i=0;i<cfg.joints.size();++i) {
      const auto & t=*command.trajectory;const auto it=std::find(t.joint_names.begin(),t.joint_names.end(),cfg.joints[i]);
      std::cout<<cfg.joints[i]<<"="<<execution->final_position_rad[i]<<"/"<<t.points.back().positions_rad[std::distance(t.joint_names.begin(),it)]<<" ";
    }
    std::cout<<std::endl;
  }
  bool pass=s.goals_sent==1;
  if(mode=="fault") {
    auto changed=command;changed.command_id+="/forbidden";
    auto refusal=port.execute_trajectory(changed);
    pass=pass && s.fault_locked && refusal && !refusal->success && port.snapshot().goals_sent==1;
    // Keep the executor alive briefly for owned cancel/result callbacks, without clearing FAULT.
    std::this_thread::sleep_for(500ms);
  } else if(mode=="cancel") {pass=pass && execution->stop_confirmed && !execution->success && !s.fault_locked && s.cancel_requests==1;}
  else {
    pass=pass && execution->success && execution->stop_confirmed && !s.fault_locked && s.cancel_requests==0;
    auto changed=command;changed.trajectory->points.back().positions_rad[0]+=0.001;
    auto refusal=port.execute_trajectory(changed);
    pass=pass && refusal && refusal->exec_state==mtc::core::ExecState::REJECTED && port.snapshot().goals_sent==1;
    if(mode=="silence") {
      const auto count=s.status_messages;std::this_thread::sleep_for(1200ms);auto later=port.snapshot();
      graph(node,port,cfg);pass=pass && later.ready && !later.fault_locked && !later.status_fresh && later.status_messages==count;
      std::cout<<"STATUS_SILENCE ready="<<later.ready<<" status_messages="<<later.status_messages<<" previous="<<count<<std::endl;
    }
  }
  std::cout<<(pass ? "PASS " : "FAIL ")<<mode<<std::endl;executor.cancel();return pass ? 0 : 1;
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);int code=1;
  auto node=std::make_shared<rclcpp::Node>("mtc_phase2c_single_owner",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  mtc::trajectory::Config cfg;cfg.enable_fake_execution=true;
  // Reviewed intersection of S3 URDF and ros2_control command position bounds.
  // run_fake_gate verifies these source bounds before any controller starts.
  const double pi=std::acos(-1.0);
  const std::vector<double> maxima={2*pi,pi,2.41,2.66,2*pi,2.23};
  for(size_t i=0;i<cfg.joints.size();++i) {cfg.limits[cfg.joints[i]]={-maxima[i],maxima[i],0.2,0.5};}
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);std::thread spin;
  try {
    auto credential=mtc::trajectory::ControlledFakeStart::verify_empty_domain(node,cfg);
    Port port(node,cfg,credential);executor.add_node(node);spin=std::thread([&]() {executor.spin();});
    try {code=run(node,port,cfg,executor);} catch(...) {executor.cancel();if(spin.joinable()) {spin.join();}throw;}
    if(spin.joinable()) {spin.join();}
  } catch(const std::exception & e) {std::cerr<<"BLOCKED/FAIL: "<<e.what()<<std::endl;}
  executor.cancel();if(spin.joinable()) {spin.join();}rclcpp::shutdown();return code;
}
