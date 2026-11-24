#ifndef EC_CONTROLLER_MIMIC_HANDOVER_H_
#define EC_CONTROLLER_MIMIC_HANDOVER_H_

#include "ec_controller/MimicPolicy.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ec::mimic
{
struct PdCommand
{
  Joints position{Joints::Zero()}, velocity{Joints::Zero()}, effort{Joints::Zero()};
  Joints kp{Joints::Zero()}, kd{Joints::Zero()};
};

inline double handoverWeight(double elapsed, double duration)
{
  if (!std::isfinite(elapsed) || elapsed < 0 || !std::isfinite(duration) || duration < 0) {
    throw std::runtime_error("Invalid activation handover timing");
  }
  const double t = duration == 0 ? 1. : std::clamp(elapsed / duration, 0., 1.);
  return t * t * (3. - 2. * t);
}

// At any measured q/dq, the resulting PD torque is exactly the weighted sum
// of the two commands' torques (before actuator saturation). Weighting target
// angles alone while also varying gains would introduce unwanted cross terms.
inline PdCommand blendPd(const PdCommand & previous, const PdCommand & next, double weight)
{
  if (!std::isfinite(weight) || weight < 0 || weight > 1) {
    throw std::runtime_error("Invalid activation handover weight");
  }
  if (weight == 0) {return previous;}
  if (weight == 1) {return next;}
  const double before = 1. - weight;
  PdCommand result;
  result.kp = before * previous.kp + weight * next.kp;
  result.kd = before * previous.kd + weight * next.kd;
  result.effort = before * previous.effort + weight * next.effort;
  for (size_t j = 0; j < kJoints; ++j) {
    result.position[j] = result.kp[j] > 0 ?
      (before * previous.kp[j] * previous.position[j] + weight * next.kp[j] * next.position[j]) / result.kp[j] :
      before * previous.position[j] + weight * next.position[j];
    result.velocity[j] = result.kd[j] > 0 ?
      (before * previous.kd[j] * previous.velocity[j] + weight * next.kd[j] * next.velocity[j]) / result.kd[j] : 0.;
  }
  return result;
}
}  // namespace ec::mimic
#endif
