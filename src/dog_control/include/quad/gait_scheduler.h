#pragma once

#include "quad/types.h"

namespace quad {

class GaitScheduler {
public:
  GaitScheduler() = default;

  void set_gait(const GaitParams &params);
  ContactSchedule step(double dt);
  const GaitParams &gait_params() const { return params_; }
  void reset_phase() { global_phase_ = 0.0; }

private:
  GaitParams params_;
  double global_phase_{0};
};

}  // namespace quad
