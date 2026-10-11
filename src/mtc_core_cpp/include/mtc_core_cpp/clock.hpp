// Copyright 2026 meituan_challenge_ws contributors
//
// 可注入时钟抽象（迁移自 Python ClockPort / FakeClock / backends.Clock）。
//
// 设计约束：状态机的超时判定必须完全依赖注入时钟，
// 不得在核心逻辑中直接调用 std::chrono::steady_clock，
// 否则单元测试无法在不睡眠的前提下验证超时与看门狗语义。

#ifndef MTC_CORE_CPP__CLOCK_HPP_
#define MTC_CORE_CPP__CLOCK_HPP_

#include <chrono>
#include <memory>

namespace mtc
{
namespace core
{

/// 时钟接口。now() 返回单调递增的秒数。
class Clock
{
public:
  virtual ~Clock() = default;
  virtual double now() const = 0;
};

using ClockPtr = std::shared_ptr<Clock>;

/// 生产用时钟：基于 std::chrono::steady_clock。
class SteadyClock : public Clock
{
public:
  double now() const override
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
};

/// 测试用假时钟：不随真实时间流逝，由测试显式推进。
class ManualClock : public Clock
{
public:
  explicit ManualClock(double start_s = 0.0)
  : now_s_(start_s) {}

  double now() const override {return now_s_;}

  void advance(double seconds)
  {
    if (seconds > 0.0) {
      now_s_ += seconds;
    }
  }

  void set(double seconds) {now_s_ = seconds;}

private:
  double now_s_ = 0.0;
};

}  // namespace core
}  // namespace mtc

#endif  // MTC_CORE_CPP__CLOCK_HPP_
