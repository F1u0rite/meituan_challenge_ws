// Copyright 2026 meituan_challenge_ws contributors
// Offline races with bounded waits. No ROS graph, RPC, IO, or hardware.
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>
#include "mtc_motion_execution_cpp/executor_core.hpp"
#include "mtc_motion_execution_cpp/in_flight_guard.hpp"

namespace {
using namespace std::chrono_literals;
using mtc::core::ErrorCodes;
using mtc::core::ExecState;

mtc::core::MotionCommand command(const std::string & id)
{
  mtc::core::MotionCommand c;
  c.command_id = id;
  c.joint_names = mtc::motion::ValidationConfig().expected_joint_names;
  c.target_rad.assign(6, 0.05);
  return c;
}

class BlockingAdapter : public mtc::motion::MotionAdapter
{
public:
  std::shared_ptr<mtc::core::ManualClock> clock = std::make_shared<mtc::core::ManualClock>(100);
  std::atomic<int> executions{0}, stops{0};
  bool report_timeout = false;
  bool stop_has_evidence = false;
  bool throw_on_stop = false;
  bool moving_feedback = false;
  bool wrong_identity = false;
  bool queue_busy = false;
  bool old_feedback = false;
  bool short_interval = false;
  bool disconnected = false;
  bool invalid_feedback = false;
  bool entered = false, released = false;
  std::string active;
  mutable std::mutex mutex;
  std::condition_variable cv;

  mtc::core::MotionCapabilities capabilities() const override
  {
    mtc::core::MotionCapabilities c;
    c.simulation_only = true;
    c.joint_goal_supported = c.cancel_supported = true;
    return c;
  }
  mtc::core::MotionResult execute_joint_move(const mtc::core::MotionCommand & c) override
  {
    ++executions;
    std::unique_lock<std::mutex> lock(mutex);
    active = c.command_id;
    entered = true;
    cv.notify_all();
    cv.wait_for(lock, 2s, [this] {return released;});
    mtc::core::MotionResult result;
    result.exec_state = report_timeout ? ExecState::TIMEOUT :
      (stops ? ExecState::STOPPED : ExecState::SETTLED);
    result.timed_out = report_timeout;
    result.stop_confirmed = !report_timeout;
    result.success = result.arrived = !report_timeout && stops == 0;
    return result;
  }
  bool wait_entered()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 1s, [this] {return entered;});
  }
  void release()
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    cv.notify_all();
  }
  mtc::core::StopResult request_stop(const std::string &) override
  {
    ++stops;
    if (throw_on_stop) {throw std::runtime_error("stop link failure");}
    mtc::core::StopResult result;
    result.request_delivered = true;
    result.stop_confirmed = true; // Boolean alone is deliberately insufficient.
    return result;
  }
  mtc::core::JointFeedback joint_feedback() const override {return {};}
  std::optional<mtc::core::StopEvidence> stop_evidence() const override
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (active.empty() || (stops && !stop_has_evidence)) {return std::nullopt;}
    mtc::core::StopEvidence e;
    e.command_id = wrong_identity ? "old-goal" : active;
    e.exec_state = moving_feedback ? ExecState::MOVING : (stops ? ExecState::STOPPED : ExecState::SETTLED);
    e.communication_ok = !disconnected;
    e.queue_empty = !queue_busy;
    e.stationary_interval_verified = true;
    e.feedback.stamp_s = clock->now() - (old_feedback ? 1.0 : 0.0);
    e.stationary_since_s = e.feedback.stamp_s - (short_interval ? 0.01 : 0.2);
    e.feedback.position_rad.assign(6, 0.05);
    e.feedback.velocity_rad_s.assign(6, moving_feedback ? 0.1 : 0.0);
    if (invalid_feedback) {e.feedback.velocity_rad_s[0] = std::numeric_limits<double>::quiet_NaN();}
    return e;
  }
};
}

TEST(ExecutorConcurrency, SixteenThreadsHaveOneOwner)
{
  mtc::motion::InFlightGuard guard;
  std::atomic<bool> go{false};
  std::atomic<int> owners{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < 16; ++i) {
    threads.emplace_back([&, i] {
      while (!go.load()) {std::this_thread::yield();}
      if (guard.acquire("thread-" + std::to_string(i)).first) {++owners;}
      (void)guard.in_flight();
      guard.release("not-the-owner");
    });
  }
  go = true;
  for (auto & t : threads) {t.join();}
  EXPECT_EQ(owners, 1);
  ASSERT_TRUE(guard.in_flight());
  guard.release(*guard.in_flight());
  EXPECT_TRUE(guard.is_free());
}

TEST(ExecutorConcurrency, GoalsAndStopRespondWhileExecutionWaits)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
  const bool entered = adapter->wait_entered();
  EXPECT_TRUE(entered);
  std::vector<std::future<mtc::core::MotionResult>> goals;
  for (int i = 0; i < 8; ++i) {
    goals.push_back(std::async(std::launch::async, [&, i] {
      return core.execute_joint_move(command("other-" + std::to_string(i)));
    }));
  }
  for (auto & goal : goals) {EXPECT_EQ(goal.get().exec_state, ExecState::REJECTED);}
  EXPECT_EQ(core.snapshot().active_command_id, "owner");
  auto stopping = std::async(std::launch::async, [&] {return core.request_stop("cancel", "owner");});
  EXPECT_EQ(stopping.wait_for(200ms), std::future_status::ready);
  EXPECT_FALSE(stopping.get().stop_confirmed);
  EXPECT_THROW(core.clear_unknown_after_manual_confirmation("operator"), std::logic_error);
  adapter->release();
  EXPECT_TRUE(running.get().status_unknown);
  EXPECT_EQ(adapter->executions, 1);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_EQ(core.execute_joint_move(command("new")).error_code, ErrorCodes::MOTION_STATUS_UNKNOWN);
}

TEST(ExecutorConcurrency, DuplicateStopsAndLateResultCannotUnlock)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->report_timeout = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
  EXPECT_TRUE(adapter->wait_entered());
  std::vector<std::future<mtc::core::StopResult>> stops;
  for (int i = 0; i < 8; ++i) {
    stops.push_back(std::async(std::launch::async, [&] {return core.request_stop("cancel", "owner");}));
  }
  for (auto & stop : stops) {EXPECT_FALSE(stop.get().stop_confirmed);}
  adapter->release();
  const auto result = running.get();
  EXPECT_TRUE(result.timed_out);
  EXPECT_TRUE(result.status_unknown);
  EXPECT_TRUE(core.motion_state_unknown());
  EXPECT_EQ(adapter->executions, 1);
}

TEST(ExecutorConcurrency, ConfirmedStopRetainsOwnerUntilResultDrains)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->stop_has_evidence = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
  EXPECT_TRUE(adapter->wait_entered());
  EXPECT_TRUE(core.request_stop("cancel", "owner").stop_confirmed);
  EXPECT_EQ(core.execute_joint_move(command("next")).exec_state, ExecState::REJECTED);
  adapter->release();
  const auto result = running.get();
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_FALSE(core.motion_state_unknown());
}

TEST(ExecutorConcurrency, InvalidEvidenceCannotConfirmStop)
{
  for (int fault = 0; fault < 8; ++fault) {
    auto adapter = std::make_shared<BlockingAdapter>();
    adapter->stop_has_evidence = true;
    adapter->moving_feedback = fault == 0;
    adapter->wrong_identity = fault == 1;
    adapter->queue_busy = fault == 2;
    adapter->old_feedback = fault == 3;
    adapter->short_interval = fault == 4;
    adapter->disconnected = fault == 5;
    adapter->invalid_feedback = fault == 6;
    adapter->throw_on_stop = fault == 7;
    mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
    auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("owner"));});
    EXPECT_TRUE(adapter->wait_entered());
    const auto stop = core.request_stop("cancel", "owner");
    EXPECT_FALSE(stop.stop_confirmed) << fault;
    adapter->release();
    EXPECT_TRUE(running.get().status_unknown) << fault;
    EXPECT_TRUE(core.motion_state_unknown());
  }
}

TEST(ExecutorConcurrency, OldGoalCancelCannotReplaceNewOwner)
{
  auto adapter = std::make_shared<BlockingAdapter>();
  adapter->stop_has_evidence = true;
  mtc::motion::MotionExecutorCore core(adapter, {}, adapter->clock);
  auto running = std::async(std::launch::async, [&] {return core.execute_joint_move(command("current"));});
  EXPECT_TRUE(adapter->wait_entered());
  EXPECT_FALSE(core.request_stop("late cancel", "old-goal").stop_confirmed);
  EXPECT_EQ(core.snapshot().active_command_id, "current");
  adapter->release();
  EXPECT_TRUE(running.get().status_unknown);
}
