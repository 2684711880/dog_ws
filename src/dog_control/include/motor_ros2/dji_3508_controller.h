#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace motor_ros2 {
namespace dji_3508 {

constexpr int kCurrentAbsMax = 16384;
constexpr int kEncoderTicksPerTurn = 8192;

struct Feedback {
  uint16_t encoder = 0;
  int16_t rpm = 0;
  int16_t given_current = 0;
  uint8_t temperature = 0;
};

struct PositionMoveOptions {
  double kp = 2600.0; 
  double kd = 6.0;
  int max_current = 3000;
  int move_bias_current = 0;
  double tolerance_turn = 0.01;
  double timeout_s = 4.0;
  int settle_hold_ms = 300;
  int loop_hz = 500;
};

struct MoveResult {
  bool reached = false;
  bool timeout = false;
  double final_turn = 0.0;
  int16_t final_cmd_current = 0;
};

class Dji3508Controller {
 public:
  Dji3508Controller(const std::string &can_if, int motor_id);
  ~Dji3508Controller();

  Dji3508Controller(const Dji3508Controller &) = delete;
  Dji3508Controller &operator=(const Dji3508Controller &) = delete;

  void send_current(int16_t current_cmd);
  bool poll_feedback(Feedback &latest_feedback);

  MoveResult move_relative_turns(double delta_turn, const PositionMoveOptions &options);
  void stop_and_relax(int safe_stop_ms = 200);

  void reset_turn_reference();
  bool has_feedback() const;
  double current_turn() const;
  int16_t last_command_current() const;

 private:
  void init_socket();
  bool ensure_feedback(double timeout_s);
  void update_tracking(const Feedback &fb);
  static Feedback parse_feedback_frame(const uint8_t *data, std::size_t data_len);
  static uint16_t command_frame_id_for_motor(int motor_id);
  static int command_slot_for_motor(int motor_id);

 private:
  std::string can_if_;
  int motor_id_ = 1;
  int socket_fd_ = -1;

  bool tracking_initialized_ = false;
  bool has_feedback_ = false;
  uint16_t last_encoder_ = 0;
  int32_t total_ticks_ = 0;
  int32_t ref_ticks_ = 0;
  double current_turn_ = 0.0;
  int16_t last_cmd_current_ = 0;
  Feedback last_feedback_{};
  std::chrono::steady_clock::time_point last_feedback_tp_{};
};

}  // namespace dji_3508
}  // namespace motor_ros2
