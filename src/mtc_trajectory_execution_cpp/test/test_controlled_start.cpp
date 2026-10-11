#include "fake_controller.hpp"
class ControlledStart : public ::testing::Test {
protected:
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<FakeController> server;
  std::unique_ptr<FjtMotionPort> port;
  std::shared_ptr<ControlledFakeStart> credential;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spin;
  Config cfg;
  static bool wait(const std::function<bool()> & f,std::chrono::milliseconds timeout=3s) {
    auto end=std::chrono::steady_clock::now()+timeout;
    while(std::chrono::steady_clock::now()<end) {if(f()) {return true;}std::this_thread::sleep_for(5ms);}return f();
  }
  void SetUp() override {
    // FakeController uses the default context; each test starts a fresh isolated domain.
    static int domain=30;
    setenv("ROS_DOMAIN_ID",std::to_string(++domain).c_str(),1);
    setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp",1);setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);
    setenv("ROS_LOG_DIR","/tmp/mtc_phase2c_tests",1);
    for(auto k:{"ROS_STATIC_PEERS","ROS_LOCALHOST_ONLY","FASTDDS_DEFAULT_PROFILES_FILE","FASTRTPS_DEFAULT_PROFILES_FILE","CYCLONEDDS_URI"}) {unsetenv(k);}
    rclcpp::init(0,nullptr);
    node=std::make_shared<rclcpp::Node>("cold_owner");cfg.enable_fake_execution=true;cfg.settle_duration=0.05;cfg.max_start_delay=0.05;
    for(const auto & j:cfg.joints) {cfg.limits[j]={-2,2,1,100};}
    executor=std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),4);
    executor->add_node(node);
  }
  void start() {
    credential=ControlledFakeStart::verify_empty_domain(node,cfg);
    port=std::make_unique<FjtMotionPort>(node,cfg,credential);
    server=std::make_shared<FakeController>(cfg);server->publish_status=false;
    executor->add_node(server);spin=std::thread([&]() {executor->spin();});
    ASSERT_TRUE(wait([&]() {return port->snapshot().ready;}));
  }
  void TearDown() override {
    if(server) {server->release_response();}
    port.reset();executor->cancel();if(spin.joinable()) {spin.join();}
    executor.reset();server.reset();node.reset();credential.reset();rclcpp::shutdown();
  }
  mtc::core::MotionCommand command(std::string id="controlled") {
    mtc::core::MotionCommand c;c.command_id=id;c.simulation_only=true;c.timeout_ms=1500;
    mtc::core::TimedTrajectory t;t.trajectory_id=id;t.header_frame_id="base_link";t.joint_names=cfg.joints;
    for(int i=0;i<3;++i) {mtc::core::TrajectoryPoint p;p.time_from_start_s=i*0.1;p.positions_rad.assign(6,i*0.01);t.points.push_back(p);}c.trajectory=t;return c;
  }
  mtc::core::MotionResult result(const mtc::core::MotionCommand & c) {
    std::optional<mtc::core::MotionResult> r;
    EXPECT_TRUE(wait([&]() {r=port->execute_trajectory(c);return r.has_value();}));return r.value_or(mtc::core::MotionResult{});
  }
};
TEST_F(ControlledStart, ExistingGraphCannotMintAuthority) {
  auto other=std::make_shared<rclcpp::Node>("preexisting_untrusted_node");
  ASSERT_TRUE(wait([&]() {return node->get_node_names().size()>1;}));
  EXPECT_THROW(ControlledFakeStart::verify_empty_domain(node,cfg),std::invalid_argument);
}
TEST_F(ControlledStart, CredentialCannotBeReused) {
  start();EXPECT_THROW(FjtMotionPort(node,cfg,credential),std::invalid_argument);
}
TEST_F(ControlledStart, ColdNativeServerWithoutInitialStatusExecutesOnce) {
  start();EXPECT_FALSE(port->snapshot().status_received);auto c=command();
  EXPECT_FALSE(port->execute_trajectory(c));auto r=result(c);EXPECT_TRUE(r.success);EXPECT_TRUE(r.stop_confirmed);
  EXPECT_EQ(server->goals,1u);EXPECT_EQ(server->cancels,0u);
  auto changed=c;changed.timeout_ms++;auto rejected=port->execute_trajectory(changed);
  ASSERT_TRUE(rejected);EXPECT_EQ(rejected->exec_state,mtc::core::ExecState::REJECTED);EXPECT_EQ(server->goals,1u);
}
TEST_F(ControlledStart, NativeStatusSilenceDoesNotRevokeAuthority) {
  start();auto c=command();port->execute_trajectory(c);ASSERT_TRUE(result(c).success);
  std::this_thread::sleep_for(800ms);EXPECT_FALSE(port->snapshot().status_fresh);EXPECT_TRUE(port->snapshot().ready);EXPECT_FALSE(port->snapshot().fault_locked);
}
TEST_F(ControlledStart, NewForeignClientLocksAndReconstructionCannotBypass) {
  start();auto foreign_node=std::make_shared<rclcpp::Node>("foreign_client");auto foreign=rclcpp_action::create_client<Fjt>(foreign_node,cfg.action_name());
  ASSERT_TRUE(wait([&]() {return port->snapshot().fault_locked;}));
  EXPECT_TRUE(port->execute_trajectory(command()));EXPECT_EQ(server->goals,0u);EXPECT_EQ(server->cancels,0u);
  foreign.reset();foreign_node.reset();port.reset();
  EXPECT_THROW(FjtMotionPort(node,cfg),std::invalid_argument);
  EXPECT_THROW(ControlledFakeStart::verify_empty_domain(node,cfg),std::invalid_argument);
}
TEST_F(ControlledStart, PendingOwnHandleNeverClaimsForeignUuid) {
  start();server->hold_response=true;auto c=command();port->execute_trajectory(c);
  ASSERT_TRUE(wait([&]() {return server->pending_requests==1;}));
  auto foreign=rclcpp_action::create_client<Fjt>(node,cfg.action_name());Fjt::Goal foreign_goal;
  auto wire=convert(command("foreign"),cfg);ASSERT_TRUE(wire.trajectory);foreign_goal.trajectory=*wire.trajectory;foreign_goal.trajectory.header.frame_id="foreign";
  auto response=foreign->async_send_goal(foreign_goal);ASSERT_EQ(response.wait_for(1s),std::future_status::ready);auto handle=response.get();ASSERT_TRUE(handle);
  ASSERT_TRUE(wait([&]() {return port->snapshot().unknown_active_goals==1;}));
  EXPECT_FALSE(port->snapshot().ready);EXPECT_EQ(server->cancels,0u);server->release_response();
  ASSERT_TRUE(wait([&]() {return port->snapshot().fault_locked && server->cancels==1;}));
  EXPECT_EQ(server->cancel_id(),server->request_id());EXPECT_NE(server->cancel_id(),handle->get_goal_id());
  EXPECT_EQ(server->goals,2u);EXPECT_EQ(port->snapshot().goals_sent,1u);
}
TEST_F(ControlledStart, AssociatedFeedbackLossLocksEvenWithFreshJointState) {
  start();server->no_result=true;server->feedback_enabled=false;auto c=command();port->execute_trajectory(c);
  EXPECT_TRUE(result(c).status_unknown);EXPECT_TRUE(port->snapshot().fault_locked);EXPECT_EQ(port->snapshot().goals_sent,1u);
}
TEST_F(ControlledStart, CancelAcknowledgementRequiresIndependentSettlement) {
  start();server->no_result=true;auto c=command();port->execute_trajectory(c);
  ASSERT_TRUE(wait([&]() {return port->snapshot().feedback_samples>0;}));
  auto owned_uuid=server->goal_id();
  EXPECT_FALSE(port->cancel_command(c.command_id,"test").stop_confirmed);auto r=result(c);
  EXPECT_TRUE(r.stop_confirmed);EXPECT_FALSE(r.success);EXPECT_EQ(server->cancel_id(),owned_uuid);EXPECT_EQ(server->cancels,1u);
}
TEST_F(ControlledStart, DiscoveryOverrideCannotMintAuthority) {
  setenv("CYCLONEDDS_URI","test-untrusted-discovery-override",1);
  EXPECT_THROW(ControlledFakeStart::verify_empty_domain(node,cfg),std::invalid_argument);
  unsetenv("CYCLONEDDS_URI");
}
TEST_F(ControlledStart, CredentialBoundToOriginalNode) {
  credential=ControlledFakeStart::verify_empty_domain(node,cfg);
  auto wrong=std::make_shared<rclcpp::Node>("different_owner");
  EXPECT_THROW(FjtMotionPort(wrong,cfg,credential),std::invalid_argument);
}
TEST_F(ControlledStart, ConcurrentPollingAndRepeatedScopedCancelRemainBounded) {
  start();server->no_result=true;
  for(int iteration=0;iteration<5;++iteration) {
    auto c=command("stress-"+std::to_string(iteration));
    auto initial=port->execute_trajectory(c);
    ASSERT_FALSE(initial) << (initial ? initial->message : "");
    ASSERT_TRUE(wait([&]() {return port->snapshot().command_id==c.command_id && port->snapshot().feedback_samples>0;}));
    const auto own=server->goal_id();
    std::thread cancel([&]() {for(int i=0;i<30;++i) {port->cancel_command(c.command_id,"stress scoped cancel");}});
    std::thread poll([&]() {for(int i=0;i<100;++i) {port->execute_trajectory(c);port->snapshot();}});
    cancel.join();poll.join();const auto r=result(c);
    ASSERT_TRUE(r.stop_confirmed);EXPECT_FALSE(r.success);EXPECT_FALSE(port->snapshot().fault_locked);
    EXPECT_EQ(server->cancel_id(),own);
    EXPECT_EQ(server->goals,static_cast<size_t>(iteration+1));EXPECT_EQ(server->cancels,static_cast<size_t>(iteration+1));
  }
}
