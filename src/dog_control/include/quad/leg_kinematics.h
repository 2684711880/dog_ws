#pragma once

#include "quad/types.h"

#include <array>

namespace quad {

// ---------------------------------------------------------------------------
// 站立姿态的原始 IK 输出 (stand 偏移)
// ---------------------------------------------------------------------------
struct StandRawIk {
  double j1{0};
  double j2{0};
  double j3{0};
};

// ---------------------------------------------------------------------------
// 每条腿的几何配置
// ---------------------------------------------------------------------------
struct LegConfig {
  Vec3 j1_origin_in_body;       // J1 中心在 body 系下的位置, m
  Vec3 j1_to_j3_at_zero;        // 零位时 J1 到 J3 的向量, 会随 J1 绕 X 轴旋转
  double thigh_length{0.21};
  double shank_length{0.21};
  double knee_offset_rad{0.3316};  // 大小腿并拢时的初始 J3, 19°
  double knee_gear_ratio{2.0};     // J3 输出端 = 关节命令 × 2

  // joint-cmd 空间限幅 (rad)
  lm::LegJoints<double> joint_min{-1.2, -1.2, -0.6};
  lm::LegJoints<double> joint_max{+1.2, +1.2, +2.4};

  // 站立标定: 零位时 J3 到足端的向量, 每条腿可独立设置 (J3 局部系, X前 Z下)
  Vec3 stand_foot_from_j3{0.0, 0.0, -0.290};
};

// ---------------------------------------------------------------------------
// 腿运动学
// ---------------------------------------------------------------------------
class LegKinematics {
public:
  explicit LegKinematics(std::array<LegConfig, lm::kLegCount> cfgs);

  // FK: 关节角 (joint-cmd 空间) → 足端位置
  Vec3 forward_kinematics(lm::LegId leg, const lm::LegJoints<double> &q) const;
  Vec3 forward_kinematics_body(lm::LegId leg, const lm::LegJoints<double> &q) const;

  // IK: 足端位置 (J1 系) → 关节命令角
  // j1_ignore_step_x: J1 计算时忽略该步长偏移 (前进时髋关节锁0)
  bool inverse_kinematics(lm::LegId leg, const Vec3 &foot_pos_j1,
                          lm::LegJoints<double> &q_out,
                          double j1_ignore_step_x = 0.0) const;
  bool inverse_kinematics_body(lm::LegId leg, const Vec3 &foot_pos_body,
                               lm::LegJoints<double> &q_out) const;

  // 工作空间
  bool is_reachable(lm::LegId leg, const Vec3 &foot_pos_j1) const;
  Vec3 clamp_to_workspace(lm::LegId leg, const Vec3 &foot_pos_j1) const;

  // 坐标变换
  Vec3 j1_to_body(lm::LegId leg, const Vec3 &p_j1) const;
  Vec3 body_to_j1(lm::LegId leg, const Vec3 &p_body) const;

  // 配置
  const LegConfig &config(lm::LegId leg) const;

  // 自检: FK(0,0,0) ≈ j1_to_j3_at_zero + stand_foot_from_j3
  bool self_check(double tol_m = 5e-3) const;

private:
  std::array<LegConfig, lm::kLegCount> configs_;
  std::array<StandRawIk, lm::kLegCount> stand_raw_;
};

// ---------------------------------------------------------------------------
// 构造默认配置 (按 docs/机械结构说明.md 实测)
// ---------------------------------------------------------------------------
std::array<LegConfig, lm::kLegCount> make_default_leg_configs();

}  // namespace quad
