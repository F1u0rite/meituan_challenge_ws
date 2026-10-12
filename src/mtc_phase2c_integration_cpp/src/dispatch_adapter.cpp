#include "mtc_phase2c_integration_cpp/dispatch_adapter.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
namespace mtc::integration {
DispatchAdapter::DispatchAdapter(mtc::trajectory::Config c, std::string s, double age)
:config_(std::move(c)),session_(std::move(s)),max_age_(age) {
  config_.validate();
  if(session_.empty() || !std::isfinite(age) || age<=0 || age>5) {
    throw std::invalid_argument("explicit session and bounded positive planning age required");
  }
}
DispatchResult DispatchAdapter::prepare(const PlanningRecord & p, DispatchPolicy policy,
  const mtc::trajectory::JointSample & current, double steady, double ros, double scheduled, double factor) {
  auto reject=[](const std::string & why) {return DispatchResult{std::nullopt,why};};
  if((policy!=DispatchPolicy::IMMEDIATE && policy!=DispatchPolicy::ROS_SCHEDULED) || !p.original.trajectory ||
    (p.clock!=GenerationClock::STEADY && p.clock!=GenerationClock::ROS) || !std::isfinite(p.generated_s) || p.generated_s<=0 ||
    !std::isfinite(steady) || !std::isfinite(ros) || steady<=0 || ros<=0 ||
    p.original.trajectory->stamp_s!=p.generated_s) {return reject("explicit generation clock, stamp and dispatch policy required");}
  const double age=(p.clock==GenerationClock::STEADY ? steady : ros)-p.generated_s;
  if(age<0 || age>max_age_) {return reject("plan generation time is future or expired");}
  if(!p.original.simulation_only || p.original.requires_contact ||
    (p.original.intent && p.original.intent->requires_contact)) {return reject("only non-contact Fake plans supported");}
  if(policy==DispatchPolicy::ROS_SCHEDULED && (!std::isfinite(scheduled) ||
    scheduled<=ros || scheduled-ros>config_.max_start_delay)) {return reject("invalid ROS scheduled execution time");}
  if(policy==DispatchPolicy::IMMEDIATE && scheduled!=0) {return reject("IMMEDIATE does not accept an implicit scheduled stamp");}
  if(!std::isfinite(factor) || factor<1 || factor>4 ||
    !p.original.trajectory->shape_is_valid() || p.original.timeout_ms==0) {
    return reject("invalid explicit uniform retiming capability");
  }
  if(sequence_==std::numeric_limits<uint64_t>::max()) {return reject("dispatch identity space exhausted");}
  auto c=p.original;  // Original record is never modified; only the explicit dispatch copy.
  const std::string id=session_+"/dispatch/"+std::to_string(++sequence_);
  c.command_id=id; c.trajectory->trajectory_id=id+"/trajectory";
  c.trajectory->stamp_s=policy==DispatchPolicy::IMMEDIATE ? 0.0 : scheduled;
  if(factor!=1) {
    for(auto & point:c.trajectory->points) {
      point.time_from_start_s*=factor;
      for(auto & v:point.velocities_rad_s) {v/=factor;}
      for(auto & a:point.accelerations_rad_s2) {a/=factor*factor;}
    }
    const double deadline=std::ceil(c.trajectory->duration_s()*1.5*1000)+1000;
    if(!std::isfinite(deadline) || deadline>config_.max_timeout_ms) {return reject("retimed deadline exceeds capability");}
    c.timeout_ms=static_cast<uint32_t>(deadline);
  }
  const auto validation=mtc::trajectory::convert(c,config_);
  if(!validation.trajectory) {return reject(validation.error);}
  if(current.positions.size()!=config_.joints.size() || current.velocities.size()!=config_.joints.size() ||
    current.stamp_ns<=0 || (ros-current.stamp_ns/1e9)<-0.01 ||
    (ros-current.stamp_ns/1e9)>config_.feedback_max_age) {return reject("current state unavailable or stale");}
  for(size_t j=0;j<current.positions.size();++j) {
    if(!std::isfinite(current.positions[j]) || !std::isfinite(current.velocities[j]) ||
      std::abs(current.velocities[j])>config_.stationary_velocity) {return reject("current state invalid or moving");}
  }
  if(!mtc::trajectory::near(current.positions,validation.trajectory->points.front().positions,config_.start_tolerance)) {
    return reject("planned start differs from current named state");
  }
  return DispatchResult{c,{}};
}
}
