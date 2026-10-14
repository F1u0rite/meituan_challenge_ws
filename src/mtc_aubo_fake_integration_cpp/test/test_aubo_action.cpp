#include "fake_controller.hpp"
#include "mtc_aubo_fake_integration_cpp/aubo_fjt_motion_port.hpp"
class AuboAction : public testing::Test {
protected:
  Config cfg;rclcpp::Node::SharedPtr node;std::shared_ptr<FakeController> server;
  std::unique_ptr<mtc::aubo_fake::AuboFjtMotionPort> port;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;std::thread worker;
  bool wait(const std::function<bool()> & ready,std::chrono::milliseconds timeout=3s) {
    const auto end=std::chrono::steady_clock::now()+timeout;
    while(std::chrono::steady_clock::now()<end) {if(ready())return true;std::this_thread::sleep_for(5ms);}return ready();
  }
  void SetUp() override {
    static int domain=110;setenv("ROS_DOMAIN_ID",std::to_string(++domain).c_str(),1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp",1);
    rclcpp::init(0,nullptr);node=std::make_shared<rclcpp::Node>("aubo_action_test");
    cfg.enable_fake_execution=true;cfg.settle_duration=.05;
    for(const auto & name:cfg.joints)cfg.limits[name]={-2,2,1,100};
    auto ticket=ControlledFakeStart::verify_empty_domain(node,cfg);
    port=std::make_unique<mtc::aubo_fake::AuboFjtMotionPort>(node,cfg,ticket,"base_link");
    server=std::make_shared<FakeController>(cfg);server->publish_status=false;
    executor=std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),4);
    executor->add_node(node);executor->add_node(server);worker=std::thread([this]{executor->spin();});
    ASSERT_TRUE(wait([this]{return port->snapshot().ready;}));
  }
  void TearDown() override {server->release_response();port.reset();executor->cancel();if(worker.joinable())worker.join();executor.reset();server.reset();node.reset();rclcpp::shutdown();}
  mtc::core::MotionCommand command(std::string id="aubo-owned") {
    mtc::core::MotionCommand c;c.command_id=id;c.simulation_only=true;c.timeout_ms=1500;
    mtc::core::TimedTrajectory t;t.trajectory_id=id+"/trajectory";t.header_frame_id="base_link";t.joint_names=cfg.joints;
    for(int i=0;i<3;++i) {mtc::core::TrajectoryPoint p;p.time_from_start_s=.1*i;p.positions_rad.assign(6,.01*i);t.points.push_back(p);}c.trajectory=t;return c;
  }
  mtc::core::MotionResult result(const mtc::core::MotionCommand & c) {
    std::optional<mtc::core::MotionResult> r;EXPECT_TRUE(wait([&]{r=port->execute_trajectory(c);return r.has_value();}));return r.value_or(mtc::core::MotionResult{});
  }
};
TEST_F(AuboAction, FullTrajectoryAndIndependentStopEvidence) {
  auto c=command();EXPECT_FALSE(port->execute_trajectory(c));auto r=result(c);ASSERT_TRUE(r.success);EXPECT_TRUE(r.stop_confirmed);
  EXPECT_EQ(server->trajectory().points.size(),3u);EXPECT_EQ(server->goals,1u);EXPECT_EQ(server->cancels,0u);
}
TEST_F(AuboAction, CancelBeforeResponsePreservesOwnUuid) {
  server->hold_response=true;server->no_result=true;auto c=command();port->execute_trajectory(c);
  ASSERT_TRUE(wait([&]{return server->pending_requests==1;}));EXPECT_FALSE(port->cancel_command(c.command_id,"pending").stop_confirmed);
  server->release_response();auto r=result(c);EXPECT_TRUE(r.stop_confirmed);EXPECT_FALSE(r.success);
  EXPECT_EQ(server->cancel_id(),server->request_id());EXPECT_EQ(server->goals,1u);EXPECT_EQ(server->cancels,1u);
}
TEST_F(AuboAction, LateResponseAfterTimeoutCannotBecomeOrphanOrUnlock) {
  server->hold_response=true;server->no_result=true;auto c=command();c.timeout_ms=600;
  // Must clear the legitimate trajectory/settlement deadline check before we
  // test transport-response timeout; an admission rejection is not that race.
  ASSERT_FALSE(port->execute_trajectory(c));
  ASSERT_TRUE(wait([&]{return port->snapshot().fault_locked;}));server->release_response();
  ASSERT_TRUE(wait([&]{return server->cancels==1;}));EXPECT_EQ(server->cancel_id(),server->request_id());
  auto next=command("next");EXPECT_TRUE(port->execute_trajectory(next));EXPECT_EQ(server->goals,1u);EXPECT_TRUE(port->snapshot().fault_locked);
}
TEST_F(AuboAction, CancelAcknowledgementWhileMovingCannotConfirmStop) {
  server->ack_only=true;auto c=command();port->execute_trajectory(c);
  ASSERT_TRUE(wait([&]{return port->snapshot().feedback_samples>0;}));server->moving=true;port->cancel_command(c.command_id,"moving");
  auto r=result(c);EXPECT_FALSE(r.stop_confirmed);EXPECT_TRUE(r.status_unknown);EXPECT_TRUE(port->snapshot().fault_locked);EXPECT_EQ(server->cancels,1u);
}
TEST_F(AuboAction, StaleJointStatePreventsGoal) {
  server->stale_state=true;ASSERT_TRUE(wait([&]{return !port->snapshot().ready;}));
  auto r=port->execute_trajectory(command());ASSERT_TRUE(r);EXPECT_EQ(server->goals,0u);
}
TEST_F(AuboAction, ChangedCompletedIdentityNeverSendsAgain) {
  auto c=command();port->execute_trajectory(c);ASSERT_TRUE(result(c).success);c.trajectory->points.back().positions_rad[0]+=.01;
  auto r=port->execute_trajectory(c);ASSERT_TRUE(r);EXPECT_EQ(r->exec_state,mtc::core::ExecState::REJECTED);EXPECT_EQ(server->goals,1u);EXPECT_EQ(server->cancels,0u);
}
TEST_F(AuboAction, WrongFrameAndStartDoNotSendGoal) {
  auto c=command();c.trajectory->header_frame_id="unverified_frame";EXPECT_TRUE(port->execute_trajectory(c));
  c=command();for(auto & p:c.trajectory->points)p.positions_rad[0]+=.1;EXPECT_TRUE(port->execute_trajectory(c));EXPECT_EQ(server->goals,0u);
}
TEST_F(AuboAction, ConcurrentRequestsHaveSingleOwner) {
  auto a=command("one"),b=command("two");std::optional<mtc::core::MotionResult> ra,rb;
  std::thread first([&]{ra=port->execute_trajectory(a);});std::thread second([&]{rb=port->execute_trajectory(b);});first.join();second.join();
  EXPECT_NE(ra.has_value(),rb.has_value());auto r=result(ra?b:a);EXPECT_TRUE(r.success);EXPECT_EQ(server->goals,1u);
}
