#include "quad/gait_scheduler.h"

#include <cmath>

namespace quad {

void GaitScheduler::set_gait(const GaitParams &params)
{
  params_ = params;
  // 步态切换时重置相位，避免跳变
  global_phase_ = 0.0;
}

ContactSchedule GaitScheduler::step(double dt)
{
  // STAND 不推进相位
  if (params_.type != GaitType::STAND) {
    global_phase_ += k2PI * params_.frequency * dt;
    while (global_phase_ >= k2PI) global_phase_ -= k2PI;
  }

  ContactSchedule cs{};
  cs.global_phase = global_phase_;

  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    // 每条腿的局部相位 = 全局相位 + 该腿的相位偏移
    double leg_phase = std::fmod(global_phase_ + params_.phase_offsets[i], k2PI);
    if (leg_phase < 0) leg_phase += k2PI;

    double u = leg_phase / k2PI;  // 归一化到 [0, 1)
    cs.phase_in_cycle[i] = u;

    if (params_.type == GaitType::STAND) {
      cs.in_stance[i] = true;
      cs.stance_progress[i] = 0.5;
      cs.swing_progress[i] = -1;
    } else {
      // duty_factor 部分是支撑相，剩余是摆动相
      cs.in_stance[i] = (u < params_.duty_factor);

      if (cs.in_stance[i]) {
        cs.stance_progress[i] = u / params_.duty_factor;
        cs.swing_progress[i] = -1;
      } else {
        cs.stance_progress[i] = -1;
        cs.swing_progress[i] = (u - params_.duty_factor) / (1.0 - params_.duty_factor);
      }
    }
  }

  return cs;
}

}  // namespace quad
