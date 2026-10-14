#pragma once
#include <mtc_manipulation_cpp/pick_place_fsm.hpp>
#include "mtc_aubo_fake_integration_cpp/aubo_fjt_motion_port.hpp"
namespace mtc::aubo_fake {
// Original FSM/Planner/Perception/Tool instances remain owned by the existing host.
// Contact primitives are still rejected by AuboFjtMotionPort; GenericSystem does
// not provide battery/tool evidence and this helper never manufactures it.
inline std::unique_ptr<mtc::manipulation::PickPlaceFsm> bind_aubo_fake_fsm(
  mtc::manipulation::PerceptionPortPtr perception,mtc::manipulation::PlannerPortPtr planner,
  std::shared_ptr<AuboFjtMotionPort> motion,mtc::manipulation::ToolPortPtr tool,
  mtc::core::ClockPtr clock,mtc::manipulation::ToolStrategyPtr strategy,
  mtc::manipulation::PickPlaceConfig config) {
  if(!perception||!planner||!motion||!tool||!clock||!strategy)throw std::invalid_argument("existing FSM dependencies required");
  return std::make_unique<mtc::manipulation::PickPlaceFsm>(std::move(perception),std::move(planner),
    std::move(motion),std::move(tool),std::move(clock),std::move(strategy),std::move(config));
}
}
