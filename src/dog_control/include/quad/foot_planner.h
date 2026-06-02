#pragma once

#include "quad/types.h"
#include "quad/leg_kinematics.h"

#include <array>

namespace quad {

// ---------------------------------------------------------------------------
// 足端轨迹规划参数
// ---------------------------------------------------------------------------
struct FootPlannerParams {
  // 步长
  double max_step_length{0.12};   // 单步步长上限, m
  double max_step_length_walk{0.06};

  // 摆动 Bezier 参数
  double step_height{0.05};       // 抬腿高度, m (≥ 0.04 机械虚位约束)
  double bezier_lift_ratio{0.4};  // Bezier 控制点抬升比例 (0~1)
  double min_step_height{0.05};   // 最小抬腿高度, m

  // yaw 切向步幅缩放: yaw_rate 会先按 gait_frequency 换成单步转角
  double yaw_step_gain{1.0};
  double yaw_step_gain_in_place{1.2};
  double yaw_lateral_gain{1.0};  // yaw 对 step_y 的保留比例; TROT 可设 0 减少横向拖地
  double in_place_turn_vx_threshold{0.03};
  double in_place_turn_vy_threshold{0.02};

  // 前后腿步幅缩放
  double swing_scale_front{1.0};
  double swing_scale_rear{1.0};

  // J1 锁定: 开启时前进不驱动 J1, 横移/转向仍正常
  bool j1_lock_forward{false};

  // 运行时 (由控制器设置)
  double gait_frequency{2.0};
};

struct FootPlannerLegDebug {
  bool in_stance{true};
  double stance_progress{0.0};
  double swing_progress{-1.0};
  Vec3 foot_pos_j1{};
  lm::LegJoints<double> joints{};
  bool ik_success{false};
};

// ---------------------------------------------------------------------------
// 足端规划器
// ---------------------------------------------------------------------------
class FootPlanner {
public:
  FootPlanner() = default;
  explicit FootPlanner(const FootPlannerParams &params);

  void set_params(const FootPlannerParams &params);

  // 核心: 给定接触计划 + 运动指令, 计算每条腿关节角
  // cmd: 机体速度指令 (vx, vy 前进/横移 m/s, yaw_rate rad/s)
  // foot_z_offset_j1: 基于每条腿 stand_foot_from_j3 的统一 z 偏移, 0 表示标定站姿
  lm::QuadJoints<double> compute(
    const ContactSchedule &cs,
    const MotionCommand &cmd,
    double foot_z_offset_j1,
    const LegKinematics &kin,
    std::array<FootPlannerLegDebug, lm::kLegCount> *debug = nullptr) const;

  // Bezier 摆动轨迹 (0→1 归一化参数 → 归一化高度 0~1)
  static double bezier_swing_height(double t, double lift_ratio);

  // 支撑相后推 (归一化参数 0→1 → 偏移)
  static double stance_offset(double t, double step_len);

private:
  FootPlannerParams params_;

  // 计算某条腿的单步足端位移 (x/y)
  void compute_step(lm::LegId leg,
                    const MotionCommand &cmd,
                    const Vec3 &nominal_foot_body,
                    double &step_x, double &step_y) const;

  // 是否前腿
  static bool is_front_leg(lm::LegId leg);
};

}  // namespace quad
