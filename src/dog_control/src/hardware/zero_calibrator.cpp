#include "motor_ros2/motor_cfg.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cctype>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

struct MotorEndpoint {
  const char *joint_name;
  const char *can_if;
  uint8_t motor_id;
};

using LegMotors = std::array<MotorEndpoint, 3>;

std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return s;
}

const std::unordered_map<std::string, LegMotors> &leg_table() {
  static const std::unordered_map<std::string, LegMotors> kTable = {
      {"LF", {{{"LF_Joint_1", "can0", 0x01},
               {"LF_Joint_2", "can0", 0x02},
               {"LF_Joint_3", "can0", 0x03}}}},
      {"RF", {{{"RF_Joint_1", "can1", 0x04},
               {"RF_Joint_2", "can1", 0x05},
               {"RF_Joint_3", "can1", 0x06}}}},
      {"LB", {{{"LB_Joint_1", "can3", 0x0A},
               {"LB_Joint_2", "can3", 0x0B},
               {"LB_Joint_3", "can3", 0x0C}}}},
      {"RB", {{{"RB_Joint_1", "can2", 0x07},
               {"RB_Joint_2", "can2", 0x08},
               {"RB_Joint_3", "can2", 0x09}}}},
  };
  return kTable;
}

void print_usage(const char *prog) {
  std::cout
      << "Usage:\n"
      << "  " << prog << " --leg <LF|RF|LB|RB> [--yes]\n\n"
      << "Options:\n"
      << "  --leg  Target leg to calibrate.\n"
      << "  --yes  Skip interactive confirmation token.\n"
      << "  -h, --help  Show this help message.\n\n"
      << "Example:\n"
      << "  ros2 run dog_control zero_calibrator --leg LF\n";
}

bool parse_args(int argc, char **argv, std::string &leg, bool &skip_confirm) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--leg") {
      if (i + 1 >= argc) {
        std::cerr << "[ERROR] --leg requires a value.\n";
        return false;
      }
      leg = argv[++i];
    } else if (arg.rfind("--leg=", 0) == 0) {
      leg = arg.substr(std::string("--leg=").size());
    } else if (arg == "--yes") {
      skip_confirm = true;
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "[ERROR] Unknown argument: " << arg << "\n";
      return false;
    }
  }

  if (leg.empty()) {
    std::cerr << "[ERROR] Missing required argument: --leg\n";
    return false;
  }
  return true;
}

bool require_confirmation(const std::string & /*leg*/) {
  std::cout << "\nConfirm that this leg is manually placed at MECHANICAL zero.\n";
  std::cout << "Type single letter y to continue: y\n> ";

  std::string input;
  std::getline(std::cin, input);

  if (input != "y") {
    std::cerr << "[ABORT] Confirmation mismatch (expected: y). No zero command sent.\n";
    return false;
  }
  return true;
}

void print_leg_info(const std::string &leg, const LegMotors &motors) {
  std::cout << "============================================\n";
  std::cout << "Single-leg mechanical zero calibration tool\n";
  std::cout << "Target leg: " << leg << "\n";
  std::cout << "Motors:\n";
  for (const auto &m : motors) {
    std::cout << "  - " << m.joint_name << "  iface=" << m.can_if
              << "  id=" << static_cast<int>(m.motor_id) << "\n";
  }
  std::cout << "Safety:\n";
  std::cout << "  1) Lift robot feet off ground / remove load.\n";
  std::cout << "  2) Ensure real_motor_controller is NOT running.\n";
  std::cout << "  3) Keep emergency stop ready.\n";
  std::cout << "============================================\n";
}

} // namespace

int main(int argc, char **argv) {
  std::string leg;
  bool skip_confirm = false;
  if (!parse_args(argc, argv, leg, skip_confirm)) {
    print_usage(argv[0]);
    return 1;
  }

  leg = to_upper(leg);
  const auto &table = leg_table();
  const auto it = table.find(leg);
  if (it == table.end()) {
    std::cerr << "[ERROR] Invalid leg: " << leg << ". Expected LF/RF/LB/RB.\n";
    print_usage(argv[0]);
    return 1;
  }
  const LegMotors &selected = it->second;

  print_leg_info(leg, selected);

  if (!skip_confirm && !require_confirmation(leg)) {
    return 2;
  }
  if (skip_confirm) {
    std::cout << "[WARN] --yes enabled: skipping confirmation token.\n";
  }

  std::vector<std::unique_ptr<RobStrideMotor>> motors;
  motors.reserve(selected.size());
  for (const auto &m : selected) {
    motors.emplace_back(std::make_unique<RobStrideMotor>(
        m.can_if, static_cast<uint8_t>(0xFF), m.motor_id, 0));
  }

  bool all_ok = true;
  for (size_t i = 0; i < motors.size(); ++i) {
    const auto &cfg = selected[i];
    std::cout << "[STEP " << (i + 1) << "/3] Set mechanical zero: "
              << cfg.joint_name << " (id=" << static_cast<int>(cfg.motor_id)
              << ")\n";
    try {
      motors[i]->Set_ZeroPos();
      std::cout << "[OK] Set_ZeroPos sent for " << cfg.joint_name << "\n";

      // Keep motor in move mode after calibration so real_motor_controller can enter follow mode directly.
      motors[i]->Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      motors[i]->Get_RobStrite_Motor_parameter(0x7005);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      std::cout << "[OK] Restored move_control_mode for " << cfg.joint_name << "\n";
    } catch (const std::exception &e) {
      all_ok = false;
      std::cerr << "[ERROR] Zero/mode-restore failed for " << cfg.joint_name << ": "
                << e.what() << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }

  std::cout << "Forcing final DISABLE on the selected 3 motors...\n";
  for (size_t i = 0; i < motors.size(); ++i) {
    const auto &cfg = selected[i];
    try {
      motors[i]->Disenable_Motor(0);
      std::cout << "[OK] Disabled " << cfg.joint_name << "\n";
    } catch (const std::exception &e) {
      all_ok = false;
      std::cerr << "[ERROR] Disable failed for " << cfg.joint_name << ": "
                << e.what() << "\n";
    }
  }

  if (all_ok) {
    std::cout << "[DONE] Leg " << leg
              << " mechanical zero workflow finished. Motors are disabled.\n";
    return 0;
  }

  std::cerr << "[DONE-WITH-ERRORS] Workflow ended with failures. "
               "Please inspect motor logs and retry carefully.\n";
  return 3;
}
