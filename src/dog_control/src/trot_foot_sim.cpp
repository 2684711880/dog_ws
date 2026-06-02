#include "quad/foot_planner.h"
#include "quad/gait_scheduler.h"
#include "quad/leg_kinematics.h"

#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

namespace lm = quad::lm;

const char *leg_name(std::size_t i)
{
  switch (static_cast<lm::LegId>(i)) {
    case lm::LegId::LF: return "LF";
    case lm::LegId::RF: return "RF";
    case lm::LegId::LB: return "LB";
    case lm::LegId::RB: return "RB";
    default: return "??";
  }
}

bool read_arg(int argc, char **argv, const std::string &name, double &out)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      out = std::strtod(argv[i + 1], nullptr);
      return true;
    }
  }
  return false;
}

bool read_arg(int argc, char **argv, const std::string &name, int &out)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      out = std::atoi(argv[i + 1]);
      return true;
    }
  }
  return false;
}

bool has_flag(int argc, char **argv, const std::string &name)
{
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) return true;
  }
  return false;
}

void print_usage(const char *prog)
{
  std::cout
    << "Usage: " << prog << " [options]\n"
    << "Options:\n"
    << "  --vx <m/s>          forward velocity, default 0.10\n"
    << "  --vy <m/s>          lateral velocity, default 0.00\n"
    << "  --yaw <rad/s>       yaw rate, default 0.00\n"
    << "  --height <m>        clearance/body height value, default 0.290\n"
    << "  --freq <hz>         trot frequency, default 1.6\n"
    << "  --duty <ratio>      trot duty factor, default 0.55\n"
    << "  --step-height <m>   swing height, default 0.060\n"
    << "  --max-step <m>      max step length, default 0.080\n"
    << "  --yaw-lateral <r>   yaw y-component gain, default 1.0\n"
    << "  --samples <n>       samples over one cycle, default 16\n"
    << "  --with-joints       also print IK joint commands for reference\n"
    << "  --help              print this help\n";
}

}  // namespace

int main(int argc, char **argv)
{
  if (has_flag(argc, argv, "--help")) {
    print_usage(argv[0]);
    return 0;
  }

  double vx = 0.10;
  double vy = 0.0;
  double yaw = 0.0;
  double height = 0.290;
  double freq = 1.6;
  double duty = 0.55;
  double step_height = 0.060;
  double max_step = 0.080;
  double yaw_lateral = 1.0;
  int samples = 16;

  read_arg(argc, argv, "--vx", vx);
  read_arg(argc, argv, "--vy", vy);
  read_arg(argc, argv, "--yaw", yaw);
  read_arg(argc, argv, "--height", height);
  read_arg(argc, argv, "--freq", freq);
  read_arg(argc, argv, "--duty", duty);
  read_arg(argc, argv, "--step-height", step_height);
  read_arg(argc, argv, "--max-step", max_step);
  read_arg(argc, argv, "--yaw-lateral", yaw_lateral);
  read_arg(argc, argv, "--samples", samples);
  if (samples < 2) samples = 2;
  const bool with_joints = has_flag(argc, argv, "--with-joints");

  quad::LegKinematics kin(quad::make_default_leg_configs());

  quad::GaitParams gp;
  gp.type = quad::GaitType::TROT;
  gp.frequency = freq;
  gp.duty_factor = duty;
  gp.phase_offsets = quad::default_phase_offsets(quad::GaitType::TROT);

  quad::FootPlannerParams fp;
  fp.step_height = step_height;
  fp.min_step_height = 0.050;
  fp.max_step_length = max_step;
  fp.yaw_step_gain = 1.0;
  fp.yaw_step_gain_in_place = 1.2;
  fp.yaw_lateral_gain = yaw_lateral;
  fp.gait_frequency = freq;

  quad::FootPlanner planner(fp);
  quad::GaitScheduler scheduler;
  scheduler.set_gait(gp);

  quad::MotionCommand cmd;
  cmd.vx = vx;
  cmd.vy = vy;
  cmd.yaw_rate = yaw;
  cmd.body_height = height;

  const double dt = 1.0 / (freq * static_cast<double>(samples));
  std::array<quad::FootPlannerLegDebug, lm::kLegCount> debug{};
  double stand_avg_z = 0.0;
  for (std::size_t i = 0; i < lm::kLegCount; ++i) {
    const auto leg = static_cast<lm::LegId>(i);
    const auto &cfg = kin.config(leg);
    stand_avg_z += (cfg.j1_to_j3_at_zero + cfg.stand_foot_from_j3).z;
  }
  stand_avg_z /= static_cast<double>(lm::kLegCount);
  const double foot_z_offset = -std::abs(height) - stand_avg_z;

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "cmd vx=" << vx << " vy=" << vy << " yaw=" << yaw
            << " freq=" << freq << " duty=" << duty
            << " yaw_lateral=" << yaw_lateral << "\n";
  std::cout << "coordinate note: j1_xyz is the planner target before IK/motor signs; body_xyz adds the J1 origin offset.\n";
  std::cout << "t,leg,phase,contact,j1_x,j1_y,j1_z,body_x,body_y,body_z";
  if (with_joints) {
    std::cout << ",j1_cmd,j2_cmd,j3_cmd,ik";
  }
  std::cout << "\n";

  for (int s = 0; s <= samples; ++s) {
    const auto cs = scheduler.step(s == 0 ? 0.0 : dt);
    planner.compute(cs, cmd, foot_z_offset, kin, &debug);

    const double t = static_cast<double>(s) * dt;
    for (std::size_t i = 0; i < lm::kLegCount; ++i) {
      const auto &d = debug[i];
      const auto leg = static_cast<lm::LegId>(i);
      const auto body = kin.j1_to_body(leg, d.foot_pos_j1);
      std::cout << t << ","
                << leg_name(i) << ","
                << cs.phase_in_cycle[i] << ","
                << (d.in_stance ? "ST" : "SW") << ","
                << d.foot_pos_j1.x << ","
                << d.foot_pos_j1.y << ","
                << d.foot_pos_j1.z << ","
                << body.x << ","
                << body.y << ","
                << body.z;
      if (with_joints) {
        std::cout << ","
                  << d.joints.j1 << ","
                  << d.joints.j2 << ","
                  << d.joints.j3 << ","
                  << (d.ik_success ? 1 : 0);
      }
      std::cout << "\n";
    }
  }

  return 0;
}
