#pragma once

#include <array>

#include <rclcpp/rclcpp.hpp>

#include "motor_ros2/leg_model.h"
#include "motor_ros2/locomotion_core_types.h"

namespace motor_ros2 {
namespace locomotion {

namespace lm = motor_ros2::leg_model;

class LocomotionPlanner
{
public:
  explicit LocomotionPlanner(const LocomotionParams &params);

  void bind_logger(const rclcpp::Logger &logger, const rclcpp::Clock::SharedPtr &clock);
  bool initialize_stand_offsets();

  lm::QuadJoints<double> compute(
    GaitMode mode,
    double phase,
    double vx,
    double vy,
    double yaw,
    double clearance) const;

  bool foot_ik(double x, double y, double z_up, double &t1, double &t2, double &t3_leg) const;

  static double stride_offset(double phase, double step_len);
  static double walk_phase_offset_for_leg(lm::LegId leg);
  static double walk_stride_offset(
    double phase,
    double step_len,
    double duty_factor,
    double pre_shift_ratio);
  static double walk_swing_lift(
    double phase,
    double step_height,
    double duty_factor,
    double pre_shift_ratio);
  static double swing_lift(double phase, double step_height);

private:
  friend class LocomotionPlannerTestPeer;

  static bool is_left_leg(lm::LegId leg);
  static bool is_front_leg(lm::LegId leg);

  double stand_foot_x_for_leg(lm::LegId leg) const;
  double stand_foot_y_for_leg(lm::LegId leg) const;
  double foot_x_min_for_leg(lm::LegId leg) const;
  double swing_scale_for_leg(lm::LegId leg) const;

  struct FootTarget
  {
    double x{0.0};
    double y{0.0};
    double z_up{0.0};
    double x_trans{0.0};
    bool in_swing{false};
  };

  std::array<FootTarget, lm::kLegCount> compute_foot_targets(
    GaitMode mode,
    double phase,
    double vx,
    double vy,
    double yaw,
    double clearance) const;

  struct LegOffset
  {
    double t1{0.0};
    double t2{0.0};
    double t3{0.0};
  };

private:
  LocomotionParams params_;
  std::array<LegOffset, lm::kLegCount> stand_offsets_{};
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace locomotion
}  // namespace motor_ros2
