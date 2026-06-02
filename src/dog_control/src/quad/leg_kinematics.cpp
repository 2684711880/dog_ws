#include "quad/leg_kinematics.h"

#include <algorithm>
#include <cmath>

namespace quad {

// ---------------------------------------------------------------------------
// 内部辅助: 计算站立姿态的原始 IK 输出 (stand 偏移)
// ---------------------------------------------------------------------------
namespace {

StandRawIk compute_stand_raw_ik(const LegConfig &cfg)
{
  // J3 到足端的向量就是 2-link 的直接目标
  const double sx = cfg.stand_foot_from_j3.x;
  const double sz = cfg.stand_foot_from_j3.z;

  // J1 = 0 (站立时无外展)
  const double j1 = 0.0;

  // 2-link 矢状面: D = sqrt(x² + z²), 站立时足端在 J3 正下方
  const double D = std::sqrt(sx * sx + sz * sz);

  double cos_j3 = (D * D - cfg.thigh_length * cfg.thigh_length
                   - cfg.shank_length * cfg.shank_length)
                  / (2.0 * cfg.thigh_length * cfg.shank_length);
  cos_j3 = std::min(std::max(cos_j3, -1.0), 1.0);
  const double j3 = std::acos(cos_j3);

  const double alpha = std::atan2(sx, -sz);
  double sin_beta = cfg.shank_length * std::sin(j3) / std::max(D, 1e-9);
  sin_beta = std::min(std::max(sin_beta, -1.0), 1.0);
  const double j2 = alpha - std::asin(sin_beta);

  return {j1, j2, j3};
}

struct J1PlaneState {
  double j1{0.0};
  double sagittal_down{0.0};
  bool valid{false};
};

J1PlaneState recover_j1_plane(const LegConfig &cfg, const Vec3 &foot_pos_j1)
{
  const double lateral = cfg.j1_to_j3_at_zero.y;
  const double yz_radius = std::hypot(foot_pos_j1.y, foot_pos_j1.z);
  const double lateral_abs = std::abs(lateral);

  if (yz_radius + 1e-9 < lateral_abs) {
    return {};
  }

  const double sagittal_down = std::sqrt(
    std::max(yz_radius * yz_radius - lateral * lateral, 0.0));
  const double cross = lateral * foot_pos_j1.z + sagittal_down * foot_pos_j1.y;
  const double dot = lateral * foot_pos_j1.y - sagittal_down * foot_pos_j1.z;

  return {std::atan2(cross, dot), sagittal_down, true};
}

Vec3 rotate_about_x(double angle, const Vec3 &v)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {v.x, v.y * c - v.z * s, v.y * s + v.z * c};
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造 + 预计算 stand 偏移
// ---------------------------------------------------------------------------
LegKinematics::LegKinematics(std::array<LegConfig, lm::kLegCount> cfgs)
{
  // 先赋值 configs_, 再计算 stand_raw_ (顺序重要)
  configs_ = std::move(cfgs);
  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    stand_raw_[i] = compute_stand_raw_ik(configs_[i]);
  }
}

// ---------------------------------------------------------------------------
// FK: 关节命令角 (joint-cmd 空间) → 足端在 J1 系的位置
//
// 运动链: J1 原点 → [J1 绕 X 轴旋转]
//         → J1->J3 零位向量 + [2-link 矢状面: thigh(J2) + shank(J2+J3)]
//         → foot
// ---------------------------------------------------------------------------
Vec3 LegKinematics::forward_kinematics(lm::LegId leg,
                                        const lm::LegJoints<double> &q) const
{
  const auto &cfg = configs_[static_cast<std::size_t>(leg)];
  const auto &sr = stand_raw_[static_cast<std::size_t>(leg)];

  const double j1_geom = q.j1 + sr.j1;
  const double j2_geom = q.j2 + sr.j2;
  const double j3_geom = q.j3 / cfg.knee_gear_ratio + sr.j3;

  // 2-link 矢状面 FK (在 J3 局部系: X 前, Z 下)
  const double fk_x = cfg.thigh_length * std::sin(j2_geom)
                      + cfg.shank_length * std::sin(j2_geom + j3_geom);
  const double fk_z = cfg.thigh_length * std::cos(j2_geom)
                      + cfg.shank_length * std::cos(j2_geom + j3_geom);

  const Vec3 chain_at_zero = cfg.j1_to_j3_at_zero + Vec3(fk_x, 0.0, -fk_z);
  return rotate_about_x(j1_geom, chain_at_zero);
}

// ---------------------------------------------------------------------------
// FK (body 系)
// ---------------------------------------------------------------------------
Vec3 LegKinematics::forward_kinematics_body(lm::LegId leg,
                                             const lm::LegJoints<double> &q) const
{
  return j1_to_body(leg, forward_kinematics(leg, q));
}

// ---------------------------------------------------------------------------
// IK: 足端在 J1 系的位置 → 关节命令角 (joint-cmd 空间)
//
// 1. 由目标 yz 半径恢复 J1 局部矢状面长度和 J1 角
// 2. 把目标点逆旋回 J1 零位系
// 3. 减去 J1->J3 零位向量后做 2-link IK
// ---------------------------------------------------------------------------
bool LegKinematics::inverse_kinematics(lm::LegId leg, const Vec3 &foot_pos_j1,
                                        lm::LegJoints<double> &q_out,
                                        double j1_ignore_step_x) const
{
  const auto &cfg = configs_[static_cast<std::size_t>(leg)];
  const auto &sr = stand_raw_[static_cast<std::size_t>(leg)];

  const auto plane = recover_j1_plane(cfg, foot_pos_j1);
  if (!plane.valid) {
    return false;
  }

  const double j1_geom = plane.j1;
  const Vec3 local = rotate_about_x(-j1_geom, foot_pos_j1);
  const double sag_x = local.x - cfg.j1_to_j3_at_zero.x;
  const double d_sag = -local.z - (-cfg.j1_to_j3_at_zero.z);
  const double D = std::sqrt(sag_x * sag_x + d_sag * d_sag);

  // 可达性检查
  const double max_reach = cfg.thigh_length + cfg.shank_length;
  const double min_reach = std::abs(cfg.thigh_length - cfg.shank_length);
  if (D > max_reach || D < min_reach) {
    return false;
  }

  // J3: law of cosines
  double cos_j3 = (D * D - cfg.thigh_length * cfg.thigh_length
                   - cfg.shank_length * cfg.shank_length)
                  / (2.0 * cfg.thigh_length * cfg.shank_length);
  cos_j3 = std::min(std::max(cos_j3, -1.0), 1.0);
  const double j3_geom = std::acos(cos_j3);

  // J2: 在矢状面内, 从竖直方向量
  const double alpha = std::atan2(sag_x - j1_ignore_step_x, d_sag);
  double sin_beta = cfg.shank_length * std::sin(j3_geom) / std::max(D, 1e-9);
  sin_beta = std::min(std::max(sin_beta, -1.0), 1.0);
  const double j2_geom = alpha - std::asin(sin_beta);

  // 转换为 joint-cmd 空间
  q_out.j1 = j1_geom - sr.j1;
  q_out.j2 = j2_geom - sr.j2;
  q_out.j3 = (j3_geom - sr.j3) * cfg.knee_gear_ratio;

  return true;
}

// ---------------------------------------------------------------------------
// IK (body 系)
// ---------------------------------------------------------------------------
bool LegKinematics::inverse_kinematics_body(lm::LegId leg,
                                             const Vec3 &foot_pos_body,
                                             lm::LegJoints<double> &q_out) const
{
  return inverse_kinematics(leg, body_to_j1(leg, foot_pos_body), q_out);
}

// ---------------------------------------------------------------------------
// 可达性检查
// ---------------------------------------------------------------------------
bool LegKinematics::is_reachable(lm::LegId leg, const Vec3 &foot_pos_j1) const
{
  const auto &cfg = configs_[static_cast<std::size_t>(leg)];
  const auto plane = recover_j1_plane(cfg, foot_pos_j1);
  if (!plane.valid) {
    return false;
  }

  const Vec3 local = rotate_about_x(-plane.j1, foot_pos_j1);
  const double dx = local.x - cfg.j1_to_j3_at_zero.x;
  const double dz = local.z - cfg.j1_to_j3_at_zero.z;
  const double D = std::hypot(dx, dz);

  const double max_reach = cfg.thigh_length + cfg.shank_length;
  const double min_reach = std::abs(cfg.thigh_length - cfg.shank_length);
  return (D >= min_reach + 1e-4) && (D <= max_reach - 1e-4);
}

// ---------------------------------------------------------------------------
// clamp_to_workspace
// ---------------------------------------------------------------------------
Vec3 LegKinematics::clamp_to_workspace(lm::LegId leg,
                                        const Vec3 &foot_pos_j1) const
{
  const auto &cfg = configs_[static_cast<std::size_t>(leg)];

  constexpr double eps = 0.005;
  const double max_reach = cfg.thigh_length + cfg.shank_length - eps;
  const double min_reach = std::abs(cfg.thigh_length - cfg.shank_length) + eps;
  const double lateral = cfg.j1_to_j3_at_zero.y;
  const double lateral_abs = std::abs(lateral);
  const double yz_radius = std::hypot(foot_pos_j1.y, foot_pos_j1.z);

  double j1 = 0.0;
  double sagittal_down = min_reach;
  if (yz_radius > lateral_abs + 1e-9) {
    const auto plane = recover_j1_plane(cfg, foot_pos_j1);
    j1 = plane.j1;
    sagittal_down = plane.sagittal_down;
  }

  const double sag_x = foot_pos_j1.x - cfg.j1_to_j3_at_zero.x;
  const double D = std::hypot(sag_x, sagittal_down);

  if (D < 1e-9) {
    return rotate_about_x(j1, cfg.j1_to_j3_at_zero + Vec3(0, 0, -min_reach));
  }

  const double D_clamped = std::min(std::max(D, min_reach), max_reach);
  const double scale = D_clamped / D;
  const Vec3 local = cfg.j1_to_j3_at_zero + Vec3(sag_x * scale, 0.0, -sagittal_down * scale);
  return rotate_about_x(j1, local);
}

// ---------------------------------------------------------------------------
// 坐标变换
// ---------------------------------------------------------------------------
Vec3 LegKinematics::j1_to_body(lm::LegId leg, const Vec3 &p_j1) const
{
  const auto &cfg = configs_[static_cast<std::size_t>(leg)];
  return cfg.j1_origin_in_body + p_j1;
}

Vec3 LegKinematics::body_to_j1(lm::LegId leg, const Vec3 &p_body) const
{
  const auto &cfg = configs_[static_cast<std::size_t>(leg)];
  return p_body - cfg.j1_origin_in_body;
}

// ---------------------------------------------------------------------------
// 配置
// ---------------------------------------------------------------------------
const LegConfig &LegKinematics::config(lm::LegId leg) const
{
  return configs_[static_cast<std::size_t>(leg)];
}

// ---------------------------------------------------------------------------
// 自检: FK(0,0,0) ≈ j1_to_j3_at_zero + stand_foot_from_j3
// ---------------------------------------------------------------------------
bool LegKinematics::self_check(double tol_m) const
{
  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    const auto leg = static_cast<lm::LegId>(i);
    lm::LegJoints<double> zero{};
    const Vec3 fk = forward_kinematics(leg, zero);
    const Vec3 expected = configs_[i].j1_to_j3_at_zero + configs_[i].stand_foot_from_j3;
    const double err = (fk - expected).norm();
    if (err > tol_m) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 默认配置 (按 docs/机械结构说明.md 实测)
// ---------------------------------------------------------------------------
std::array<LegConfig, lm::kLegCount> make_default_leg_configs()
{
  LegConfig base{};
  base.thigh_length = 0.21;
  base.shank_length = 0.21;
  base.knee_offset_rad = 19.0 * kPI / 180.0;  // 0.3316
  base.knee_gear_ratio = 2.0;

  std::array<LegConfig, 4> c;

  // LF: 左前
  c[0] = base;
  c[0].j1_origin_in_body = {+0.150, +0.050, 0.0};
  c[0].j1_to_j3_at_zero = {+0.060, +0.095, 0.0};
  c[0].stand_foot_from_j3 = {+0.04, 0.0, -0.295};

  // RF: 右前
  c[1] = base;
  c[1].j1_origin_in_body = {+0.150, -0.050, 0.0};
  c[1].j1_to_j3_at_zero = {+0.060, -0.095, 0.0};
  c[1].stand_foot_from_j3 = {+0.04, 0.0, -0.295};

  // LB: 左后
  c[2] = base;
  c[2].j1_origin_in_body = {-0.150, +0.050, 0.0};
  c[2].j1_to_j3_at_zero = {-0.060, +0.095, 0.0};
  c[2].stand_foot_from_j3 = {-0.04, 0.0, -0.295};

  // RB: 右后
  c[3] = base;
  c[3].j1_origin_in_body = {-0.150, -0.050, 0.0};
  c[3].j1_to_j3_at_zero = {-0.060, -0.095, 0.0};
  c[3].stand_foot_from_j3 = {-0.04, 0.0, -0.295};

  return c;
}

}  // namespace quad
