#include "motor_ros2/dji_3508_controller.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace {

namespace dji = motor_ros2::dji_3508;

constexpr int kDefaultLoopHz = 500;
constexpr int kStatusPrintMs = 100;
constexpr int kSafeStopMs = 200;
constexpr int kDefaultHoldPrintMs = 200;

std::atomic<bool> g_stop_requested{false};

void on_signal(int) { g_stop_requested.store(true); }

void install_signal_handlers() {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
}

enum class RunMode {
  kCurrent,
  kPosition,
};

struct Config {
  std::string can_if = "can4";
  int motor_id = 1;
  bool skip_confirm = false;
  RunMode mode = RunMode::kCurrent;

  // current mode
  int target_current = 1000;
  double duration_s = 2.0;
  int ramp_ms = 800;

  // position mode
  bool has_target_turn = false;
  double target_turn = 0.0;
  double kp = 2600.0;
  double kd = 6.0;
  int max_current = 3000;
  int move_bias_current = 0;
  double tol_turn = 0.01;
  double timeout_s = 4.0;
  int settle_ms = 300;
  bool hold_after_reach = false;
  int hold_bias_current = 0;
  int hold_print_ms = kDefaultHoldPrintMs;
};

enum class ParseResult {
  kOk,
  kHelp,
  kError,
};

class SafeStopGuard {
 public:
  SafeStopGuard(dji::Dji3508Controller &controller, int safe_stop_ms)
      : controller_(controller), safe_stop_ms_(safe_stop_ms) {}

  ~SafeStopGuard() { controller_.stop_and_relax(safe_stop_ms_); }

 private:
  dji::Dji3508Controller &controller_;
  int safe_stop_ms_;
};

bool parse_int_str(const std::string &s, int &out) {
  try {
    size_t pos = 0;
    long v = std::stol(s, &pos, 10);
    if (pos != s.size()) {
      return false;
    }
    if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) {
      return false;
    }
    out = static_cast<int>(v);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_double_str(const std::string &s, double &out) {
  try {
    size_t pos = 0;
    out = std::stod(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

bool parse_string_value(int argc, char **argv, int &i, const std::string &arg_name,
                        std::string &out) {
  if (i + 1 >= argc) {
    std::cerr << "[ERROR] " << arg_name << " requires a value.\n";
    return false;
  }
  out = argv[++i];
  return true;
}

bool parse_int_value(int argc, char **argv, int &i, const std::string &arg_name, int &out) {
  if (i + 1 >= argc) {
    std::cerr << "[ERROR] " << arg_name << " requires a value.\n";
    return false;
  }
  const std::string value = argv[++i];
  if (!parse_int_str(value, out)) {
    std::cerr << "[ERROR] Invalid integer for " << arg_name << ": " << value << "\n";
    return false;
  }
  return true;
}

bool parse_double_value(int argc, char **argv, int &i, const std::string &arg_name,
                        double &out) {
  if (i + 1 >= argc) {
    std::cerr << "[ERROR] " << arg_name << " requires a value.\n";
    return false;
  }
  const std::string value = argv[++i];
  if (!parse_double_str(value, out)) {
    std::cerr << "[ERROR] Invalid number for " << arg_name << ": " << value << "\n";
    return false;
  }
  return true;
}

bool parse_mode(const std::string &value, RunMode &mode_out) {
  if (value == "current") {
    mode_out = RunMode::kCurrent;
    return true;
  }
  if (value == "position") {
    mode_out = RunMode::kPosition;
    return true;
  }
  return false;
}

void print_usage(const char *prog) {
  std::cout
      << "Usage:\n"
      << "  " << prog
      << " [--if can4] [--id 1] [--mode current|position] [mode options] [--yes]\n\n"
      << "Common options:\n"
      << "  --if <canX>          CAN interface (default: can4)\n"
      << "  --id <1..8>          Motor CAN id (default: 1)\n"
      << "  --mode <name>        current or position (default: current)\n"
      << "  --yes                Skip interactive safety confirmation\n"
      << "  -h, --help           Show this help message\n\n"
      << "Current mode options:\n"
      << "  --current <int>      Target current, range [-16384, 16384] (default: 1000)\n"
      << "  --duration <sec>     Duration > 0 (default: 2.0)\n"
      << "  --ramp-ms <ms>       Ramp time >= 0 (default: 800)\n\n"
      << "Position mode options:\n"
      << "  --target-turn <d>    Required. Relative turns from current position.\n"
      << "  --kp <d>             Position P gain (default: 2600)\n"
      << "  --kd <d>             Speed D gain on rpm (default: 6)\n"
      << "  --max-current <int>  Output current limit in [1, 16384] (default: 3000)\n"
      << "  --move-bias-current  Extra drive during move for stiction breakaway (default: 0)\n"
      << "  --tol-turn <d>       Reach tolerance in turns > 0 (default: 0.01)\n"
      << "  --timeout <sec>      Move timeout > 0 (default: 4.0)\n"
      << "  --settle-ms <ms>     Hold-inside-tolerance time >= 0 (default: 300)\n\n"
      << "Hold options (position mode):\n"
      << "  --hold-after-reach   Keep holding target position until Ctrl+C\n"
      << "  --hold-bias-current  Extra bias current for anti-backdrive (default: 0)\n"
      << "  --hold-print-ms <ms> Hold status print period > 0 (default: 200)\n\n"
      << "Examples:\n"
      << "  ros2 run dog_control dji_3508_spin_test --mode current --if can4 --id 1 "
         "--current 600 --duration 1.5 --ramp-ms 1000\n"
      << "  ros2 run dog_control dji_3508_spin_test --mode position --if can4 --id 1 "
         "--target-turn 0.25 --kp 2600 --kd 6 --max-current 3000 --tol-turn 0.01 "
         "--timeout 4 --settle-ms 300\n"
      << "  ros2 run dog_control dji_3508_spin_test --mode position --if can4 --id 1 "
         "--target-turn 0.25 --kp 9000 --kd 1 --max-current 6000 --move-bias-current 600 --hold-after-reach "
         "--hold-bias-current 800\n";
}

ParseResult parse_args(int argc, char **argv, Config &cfg) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      return ParseResult::kHelp;
    }
    if (arg == "--yes") {
      cfg.skip_confirm = true;
      continue;
    }
    if (arg == "--hold-after-reach") {
      cfg.hold_after_reach = true;
      continue;
    }
    if (arg == "--if") {
      if (!parse_string_value(argc, argv, i, "--if", cfg.can_if)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--if=", 0) == 0) {
      cfg.can_if = arg.substr(std::string("--if=").size());
      continue;
    }
    if (arg == "--id") {
      if (!parse_int_value(argc, argv, i, "--id", cfg.motor_id)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--id=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--id=").size()), cfg.motor_id)) {
        std::cerr << "[ERROR] Invalid integer for --id.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--mode") {
      std::string value;
      if (!parse_string_value(argc, argv, i, "--mode", value)) {
        return ParseResult::kError;
      }
      if (!parse_mode(value, cfg.mode)) {
        std::cerr << "[ERROR] Invalid --mode: " << value << ", expected current|position.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--mode=", 0) == 0) {
      if (!parse_mode(arg.substr(std::string("--mode=").size()), cfg.mode)) {
        std::cerr << "[ERROR] Invalid --mode, expected current|position.\n";
        return ParseResult::kError;
      }
      continue;
    }

    if (arg == "--current") {
      if (!parse_int_value(argc, argv, i, "--current", cfg.target_current)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--current=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--current=").size()), cfg.target_current)) {
        std::cerr << "[ERROR] Invalid integer for --current.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--duration") {
      if (!parse_double_value(argc, argv, i, "--duration", cfg.duration_s)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--duration=", 0) == 0) {
      if (!parse_double_str(arg.substr(std::string("--duration=").size()), cfg.duration_s)) {
        std::cerr << "[ERROR] Invalid number for --duration.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--ramp-ms") {
      if (!parse_int_value(argc, argv, i, "--ramp-ms", cfg.ramp_ms)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--ramp-ms=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--ramp-ms=").size()), cfg.ramp_ms)) {
        std::cerr << "[ERROR] Invalid integer for --ramp-ms.\n";
        return ParseResult::kError;
      }
      continue;
    }

    if (arg == "--target-turn") {
      if (!parse_double_value(argc, argv, i, "--target-turn", cfg.target_turn)) {
        return ParseResult::kError;
      }
      cfg.has_target_turn = true;
      continue;
    }
    if (arg.rfind("--target-turn=", 0) == 0) {
      if (!parse_double_str(arg.substr(std::string("--target-turn=").size()), cfg.target_turn)) {
        std::cerr << "[ERROR] Invalid number for --target-turn.\n";
        return ParseResult::kError;
      }
      cfg.has_target_turn = true;
      continue;
    }
    if (arg == "--kp") {
      if (!parse_double_value(argc, argv, i, "--kp", cfg.kp)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--kp=", 0) == 0) {
      if (!parse_double_str(arg.substr(std::string("--kp=").size()), cfg.kp)) {
        std::cerr << "[ERROR] Invalid number for --kp.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--kd") {
      if (!parse_double_value(argc, argv, i, "--kd", cfg.kd)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--kd=", 0) == 0) {
      if (!parse_double_str(arg.substr(std::string("--kd=").size()), cfg.kd)) {
        std::cerr << "[ERROR] Invalid number for --kd.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--max-current") {
      if (!parse_int_value(argc, argv, i, "--max-current", cfg.max_current)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--max-current=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--max-current=").size()), cfg.max_current)) {
        std::cerr << "[ERROR] Invalid integer for --max-current.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--move-bias-current") {
      if (!parse_int_value(argc, argv, i, "--move-bias-current", cfg.move_bias_current)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--move-bias-current=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--move-bias-current=").size()),
                         cfg.move_bias_current)) {
        std::cerr << "[ERROR] Invalid integer for --move-bias-current.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--tol-turn") {
      if (!parse_double_value(argc, argv, i, "--tol-turn", cfg.tol_turn)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--tol-turn=", 0) == 0) {
      if (!parse_double_str(arg.substr(std::string("--tol-turn=").size()), cfg.tol_turn)) {
        std::cerr << "[ERROR] Invalid number for --tol-turn.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--timeout") {
      if (!parse_double_value(argc, argv, i, "--timeout", cfg.timeout_s)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--timeout=", 0) == 0) {
      if (!parse_double_str(arg.substr(std::string("--timeout=").size()), cfg.timeout_s)) {
        std::cerr << "[ERROR] Invalid number for --timeout.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--settle-ms") {
      if (!parse_int_value(argc, argv, i, "--settle-ms", cfg.settle_ms)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--settle-ms=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--settle-ms=").size()), cfg.settle_ms)) {
        std::cerr << "[ERROR] Invalid integer for --settle-ms.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--hold-bias-current") {
      if (!parse_int_value(argc, argv, i, "--hold-bias-current", cfg.hold_bias_current)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--hold-bias-current=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--hold-bias-current=").size()),
                         cfg.hold_bias_current)) {
        std::cerr << "[ERROR] Invalid integer for --hold-bias-current.\n";
        return ParseResult::kError;
      }
      continue;
    }
    if (arg == "--hold-print-ms") {
      if (!parse_int_value(argc, argv, i, "--hold-print-ms", cfg.hold_print_ms)) {
        return ParseResult::kError;
      }
      continue;
    }
    if (arg.rfind("--hold-print-ms=", 0) == 0) {
      if (!parse_int_str(arg.substr(std::string("--hold-print-ms=").size()),
                         cfg.hold_print_ms)) {
        std::cerr << "[ERROR] Invalid integer for --hold-print-ms.\n";
        return ParseResult::kError;
      }
      continue;
    }

    std::cerr << "[ERROR] Unknown argument: " << arg << "\n";
    return ParseResult::kError;
  }

  if (cfg.can_if.empty()) {
    std::cerr << "[ERROR] --if cannot be empty.\n";
    return ParseResult::kError;
  }
  if (cfg.motor_id < 1 || cfg.motor_id > 8) {
    std::cerr << "[ERROR] --id out of range, expected 1..8.\n";
    return ParseResult::kError;
  }

  if (cfg.mode == RunMode::kCurrent) {
    if (cfg.target_current < -dji::kCurrentAbsMax || cfg.target_current > dji::kCurrentAbsMax) {
      std::cerr << "[ERROR] --current out of range, expected [" << -dji::kCurrentAbsMax
                << ", " << dji::kCurrentAbsMax << "].\n";
      return ParseResult::kError;
    }
    if (!(cfg.duration_s > 0.0)) {
      std::cerr << "[ERROR] --duration must be > 0.\n";
      return ParseResult::kError;
    }
    if (cfg.ramp_ms < 0) {
      std::cerr << "[ERROR] --ramp-ms must be >= 0.\n";
      return ParseResult::kError;
    }
  } else {
    if (!cfg.has_target_turn) {
      std::cerr << "[ERROR] --target-turn is required in position mode.\n";
      return ParseResult::kError;
    }
    if (!std::isfinite(cfg.kp) || !std::isfinite(cfg.kd)) {
      std::cerr << "[ERROR] --kp/--kd must be finite numbers.\n";
      return ParseResult::kError;
    }
    if (cfg.max_current < 1 || cfg.max_current > dji::kCurrentAbsMax) {
      std::cerr << "[ERROR] --max-current out of range, expected [1, " << dji::kCurrentAbsMax
                << "].\n";
      return ParseResult::kError;
    }
    if (std::abs(cfg.move_bias_current) > dji::kCurrentAbsMax) {
      std::cerr << "[ERROR] --move-bias-current out of range, expected [" << -dji::kCurrentAbsMax
                << ", " << dji::kCurrentAbsMax << "].\n";
      return ParseResult::kError;
    }
    if (!(cfg.tol_turn > 0.0)) {
      std::cerr << "[ERROR] --tol-turn must be > 0.\n";
      return ParseResult::kError;
    }
    if (!(cfg.timeout_s > 0.0)) {
      std::cerr << "[ERROR] --timeout must be > 0.\n";
      return ParseResult::kError;
    }
    if (cfg.settle_ms < 0) {
      std::cerr << "[ERROR] --settle-ms must be >= 0.\n";
      return ParseResult::kError;
    }
    if (std::abs(cfg.hold_bias_current) > dji::kCurrentAbsMax) {
      std::cerr << "[ERROR] --hold-bias-current out of range, expected [" << -dji::kCurrentAbsMax
                << ", " << dji::kCurrentAbsMax << "].\n";
      return ParseResult::kError;
    }
    if (cfg.hold_print_ms <= 0) {
      std::cerr << "[ERROR] --hold-print-ms must be > 0.\n";
      return ParseResult::kError;
    }
  }

  return ParseResult::kOk;
}

bool confirm_or_abort(bool skip_confirm) {
  std::cout << "============================================\n"
            << "DJI 3508 single-motor test (C620)\n"
            << "Safety checklist:\n"
            << "  1) Lift wheel/leg off ground and remove external load.\n"
            << "  2) Ensure other motor controllers are NOT sending CAN commands.\n"
            << "  3) Keep emergency stop / power kill switch ready.\n"
            << "============================================\n";

  if (skip_confirm) {
    std::cout << "[WARN] --yes enabled: skip interactive confirmation.\n";
    return true;
  }

  std::cout << "Type single letter y to continue: y\n> ";
  std::string input;
  std::getline(std::cin, input);
  if (input != "y") {
    std::cerr << "[ABORT] Confirmation mismatch (expected: y). No drive command sent.\n";
    return false;
  }
  return true;
}

int run_current_mode(dji::Dji3508Controller &controller, const Config &cfg) {
  std::cout << "[INFO] mode=current if=" << cfg.can_if << " id=" << cfg.motor_id
            << " target_current=" << cfg.target_current << " duration=" << cfg.duration_s
            << "s ramp=" << cfg.ramp_ms << "ms\n";

  const auto start = std::chrono::steady_clock::now();
  auto next_tick = start;
  auto next_print = start;
  auto last_feedback_tp = start;
  dji::Feedback last_fb;
  bool got_feedback_once = false;
  const int period_us = 1000000 / kDefaultLoopHz;

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - start).count();
    if (elapsed_s >= cfg.duration_s) {
      break;
    }

    const double elapsed_ms = elapsed_s * 1000.0;
    const double ratio =
        (cfg.ramp_ms == 0) ? 1.0 : std::min(1.0, elapsed_ms / static_cast<double>(cfg.ramp_ms));
    const int cmd_int = static_cast<int>(std::lround(static_cast<double>(cfg.target_current) * ratio));
    controller.send_current(static_cast<int16_t>(cmd_int));

    dji::Feedback fb;
    if (controller.poll_feedback(fb)) {
      last_fb = fb;
      last_feedback_tp = now;
      got_feedback_once = true;
    }

    if (now >= next_print) {
      std::cout << "[STAT] t=" << elapsed_s << "s cmd=" << cmd_int;
      if (got_feedback_once) {
        const double stale_ms =
            std::chrono::duration<double, std::milli>(now - last_feedback_tp).count();
        std::cout << " fb: ecd=" << last_fb.encoder << " rpm=" << last_fb.rpm
                  << " iq=" << last_fb.given_current
                  << " temp=" << static_cast<int>(last_fb.temperature) << "C"
                  << " turn=" << controller.current_turn();
        if (stale_ms > 500.0) {
          std::cout << " [WARN feedback stale " << stale_ms << "ms]";
        }
      } else {
        std::cout << " [WARN no feedback yet]";
      }
      std::cout << "\n";
      next_print += std::chrono::milliseconds(kStatusPrintMs);
    }

    next_tick += std::chrono::microseconds(period_us);
    std::this_thread::sleep_until(next_tick);
    const auto after_sleep = std::chrono::steady_clock::now();
    if (after_sleep > next_tick + std::chrono::microseconds(period_us)) {
      next_tick = after_sleep;
    }
  }

  std::cout << "[DONE] Current-mode test complete.\n";
  return 0;
}

int run_position_mode(dji::Dji3508Controller &controller, const Config &cfg) {
  dji::PositionMoveOptions opts;
  opts.kp = cfg.kp;
  opts.kd = cfg.kd;
  opts.max_current = cfg.max_current;
  opts.move_bias_current = cfg.move_bias_current;
  opts.tolerance_turn = cfg.tol_turn;
  opts.timeout_s = cfg.timeout_s;
  opts.settle_hold_ms = cfg.settle_ms;
  opts.loop_hz = kDefaultLoopHz;

  std::cout << "[INFO] mode=position if=" << cfg.can_if << " id=" << cfg.motor_id
            << " target_turn=" << cfg.target_turn << " kp=" << cfg.kp << " kd=" << cfg.kd
            << " max_current=" << cfg.max_current
            << " move_bias_current=" << cfg.move_bias_current
            << " tol_turn=" << cfg.tol_turn
            << " timeout=" << cfg.timeout_s << "s settle_ms=" << cfg.settle_ms
            << " hold_after_reach=" << (cfg.hold_after_reach ? "true" : "false")
            << " hold_bias_current=" << cfg.hold_bias_current << "\n";

  const dji::MoveResult result = controller.move_relative_turns(cfg.target_turn, opts);
  if (result.reached) {
    std::cout << "[DONE] Position reached. final_turn=" << result.final_turn
              << " final_cmd_current=" << result.final_cmd_current << "\n";
    if (!cfg.hold_after_reach) {
      return 0;
    }

    const double hold_target_turn = result.final_turn;
    std::cout << "[HOLD] Holding target_turn=" << hold_target_turn
              << " (Ctrl+C to stop).\n";

    auto next_tick = std::chrono::steady_clock::now();
    auto next_print = next_tick;
    auto last_feedback_tp = next_tick;
    dji::Feedback last_fb{};
    bool got_feedback = false;
    const int period_us = 1000000 / kDefaultLoopHz;

    while (!g_stop_requested.load()) {
      const auto now = std::chrono::steady_clock::now();

      dji::Feedback fb;
      if (controller.poll_feedback(fb)) {
        last_fb = fb;
        last_feedback_tp = now;
        got_feedback = true;
      }

      if (!got_feedback) {
        controller.send_current(0);
      } else {
        const double stale_s = std::chrono::duration<double>(now - last_feedback_tp).count();
        if (stale_s > 0.5) {
          std::cerr << "[ERROR] Hold feedback stale " << stale_s
                    << "s. Leaving hold loop.\n";
          return 6;
        }

        const double err = hold_target_turn - controller.current_turn();
        double cmd = cfg.kp * err - cfg.kd * static_cast<double>(last_fb.rpm);

        if (cfg.hold_bias_current != 0 && std::fabs(err) > (cfg.tol_turn * 0.3)) {
          const int bias_mag = std::abs(cfg.hold_bias_current);
          if (cmd > 0.0) {
            cmd += static_cast<double>(bias_mag);
          } else if (cmd < 0.0) {
            cmd -= static_cast<double>(bias_mag);
          } else {
            cmd = (err >= 0.0) ? static_cast<double>(bias_mag) : -static_cast<double>(bias_mag);
          }
        }

        const int cmd_int =
            static_cast<int>(std::lround(std::max(-static_cast<double>(cfg.max_current),
                                                  std::min(static_cast<double>(cfg.max_current),
                                                           cmd))));
        controller.send_current(static_cast<int16_t>(cmd_int));

        if (now >= next_print) {
          std::cout << "[HOLD] turn=" << controller.current_turn() << " err=" << err
                    << " rpm=" << last_fb.rpm << " cmd=" << cmd_int
                    << " temp=" << static_cast<int>(last_fb.temperature) << "C\n";
          next_print += std::chrono::milliseconds(cfg.hold_print_ms);
        }
      }

      next_tick += std::chrono::microseconds(period_us);
      std::this_thread::sleep_until(next_tick);
      const auto after_sleep = std::chrono::steady_clock::now();
      if (after_sleep > next_tick + std::chrono::microseconds(period_us)) {
        next_tick = after_sleep;
      }
    }
    std::cout << "[HOLD] Stop requested.\n";
    return 0;
  }
  if (result.timeout) {
    std::cerr << "[ERROR] Position move timeout. final_turn=" << result.final_turn << "\n";
    return 4;
  }

  std::cerr << "[ERROR] Position move failed (feedback lost or no feedback). final_turn="
            << result.final_turn << "\n";
  return 5;
}

}  // namespace

int main(int argc, char **argv) {
  install_signal_handlers();
  g_stop_requested.store(false);

  Config cfg;
  const ParseResult parse_result = parse_args(argc, argv, cfg);
  if (parse_result == ParseResult::kHelp) {
    print_usage(argv[0]);
    return 0;
  }
  if (parse_result == ParseResult::kError) {
    print_usage(argv[0]);
    return 1;
  }

  if (!confirm_or_abort(cfg.skip_confirm)) {
    return 2;
  }

  try {
    dji::Dji3508Controller controller(cfg.can_if, cfg.motor_id);
    SafeStopGuard safe_stop(controller, kSafeStopMs);

    if (cfg.mode == RunMode::kCurrent) {
      return run_current_mode(controller, cfg);
    }
    return run_position_mode(controller, cfg);
  } catch (const std::exception &e) {
    std::cerr << "[ERROR] " << e.what()
              << "\n[SAFE-STOP] Sending zero current for " << kSafeStopMs
              << "ms before exit.\n";
    return 3;
  }
}
