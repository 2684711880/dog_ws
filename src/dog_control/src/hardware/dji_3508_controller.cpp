


#include "motor_ros2/dji_3508_controller.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace motor_ros2 {
namespace dji_3508 {

namespace {
constexpr int kUnwrapThreshold = kEncoderTicksPerTurn / 2;
constexpr double kFeedbackLostTimeoutS = 0.30;
constexpr double kInitialFeedbackTimeoutS = 1.0;
}

Dji3508Controller::Dji3508Controller(const std::string &can_if, int motor_id)
    : can_if_(can_if), motor_id_(motor_id) {
  if (can_if_.empty()) {
    throw std::invalid_argument("CAN interface cannot be empty");
  }
  if (motor_id_ < 1 || motor_id_ > 8) {
    throw std::invalid_argument("motor_id out of range, expected 1..8");
  }
  init_socket();
}

Dji3508Controller::~Dji3508Controller() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void Dji3508Controller::init_socket() {
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw std::runtime_error("socket failed: " + std::string(std::strerror(errno)));
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_if_.c_str(), IFNAMSIZ);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    throw std::runtime_error("ioctl(SIOCGIFINDEX) failed for " + can_if_ + ": " +
                             std::string(std::strerror(errno)));
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    throw std::runtime_error("bind failed on " + can_if_ + ": " +
                             std::string(std::strerror(errno)));
  }

  const uint16_t feedback_id = static_cast<uint16_t>(0x200 + motor_id_);
  struct can_filter filter {};
  filter.can_id = feedback_id;
  filter.can_mask = CAN_SFF_MASK;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    throw std::runtime_error("setsockopt(CAN_RAW_FILTER) failed: " +
                             std::string(std::strerror(errno)));
  }

  const int old_flags = fcntl(socket_fd_, F_GETFL, 0);
  if (old_flags < 0) {
    throw std::runtime_error("fcntl(F_GETFL) failed: " + std::string(std::strerror(errno)));
  }
  if (fcntl(socket_fd_, F_SETFL, old_flags | O_NONBLOCK) < 0) {
    throw std::runtime_error("fcntl(F_SETFL O_NONBLOCK) failed: " +
                             std::string(std::strerror(errno)));
  }
}

uint16_t Dji3508Controller::command_frame_id_for_motor(int motor_id) {
  return (motor_id <= 4) ? static_cast<uint16_t>(0x200) : static_cast<uint16_t>(0x1FF);
}

int Dji3508Controller::command_slot_for_motor(int motor_id) { return (motor_id - 1) % 4; }

Feedback Dji3508Controller::parse_feedback_frame(const uint8_t *data, std::size_t data_len) {
  Feedback fb;
  if (data == nullptr || data_len < 7) {
    return fb;
  }
  fb.encoder = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
  fb.rpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
  fb.given_current = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
  fb.temperature = data[6];
  return fb;
}

void Dji3508Controller::update_tracking(const Feedback &fb) {
  if (!tracking_initialized_) {
    tracking_initialized_ = true;
    has_feedback_ = true;
    last_encoder_ = fb.encoder;
    total_ticks_ = 0;
    ref_ticks_ = 0;
    current_turn_ = 0.0;
  } else {
    int delta = static_cast<int>(fb.encoder) - static_cast<int>(last_encoder_);
    if (delta > kUnwrapThreshold) {
      delta -= kEncoderTicksPerTurn;
    } else if (delta < -kUnwrapThreshold) {
      delta += kEncoderTicksPerTurn;
    }
    total_ticks_ += delta;
    last_encoder_ = fb.encoder;
    current_turn_ =
        static_cast<double>(total_ticks_ - ref_ticks_) / static_cast<double>(kEncoderTicksPerTurn);
  }

  has_feedback_ = true;
  last_feedback_ = fb;
  last_feedback_tp_ = std::chrono::steady_clock::now();
}

void Dji3508Controller::send_current(int16_t current_cmd) {
  current_cmd = static_cast<int16_t>(
      std::max(-kCurrentAbsMax, std::min(kCurrentAbsMax, static_cast<int>(current_cmd))));

  struct can_frame frame {};
  frame.can_id = command_frame_id_for_motor(motor_id_);
  frame.can_dlc = 8;
  std::memset(frame.data, 0, sizeof(frame.data));

  const int slot = command_slot_for_motor(motor_id_);
  frame.data[slot * 2] = static_cast<uint8_t>((current_cmd >> 8) & 0xFF);
  frame.data[slot * 2 + 1] = static_cast<uint8_t>(current_cmd & 0xFF);

  const int n = write(socket_fd_, &frame, sizeof(frame));
  if (n != static_cast<int>(sizeof(frame))) {
    throw std::runtime_error("write CAN frame failed: " + std::string(std::strerror(errno)));
  }
  last_cmd_current_ = current_cmd;
}

bool Dji3508Controller::poll_feedback(Feedback &latest_feedback) {
  bool found = false;
  const uint16_t feedback_id = static_cast<uint16_t>(0x200 + motor_id_);

  while (true) {
    struct can_frame frame {};
    const int n = read(socket_fd_, &frame, sizeof(frame));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      throw std::runtime_error("read CAN frame failed: " + std::string(std::strerror(errno)));
    }
    if (n != static_cast<int>(sizeof(frame))) {
      continue;
    }
    if (frame.can_id & CAN_EFF_FLAG) {
      continue;
    }
    if (frame.can_id & CAN_RTR_FLAG) {
      continue;
    }

    const uint16_t rx_id = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
    if (rx_id != feedback_id) {
      continue;
    }

    latest_feedback = parse_feedback_frame(frame.data, frame.can_dlc);
    update_tracking(latest_feedback);
    found = true;
  }

  return found;
}

bool Dji3508Controller::ensure_feedback(double timeout_s) {
  const auto start = std::chrono::steady_clock::now();
  const auto poll_sleep = std::chrono::milliseconds(2);
  Feedback fb;

  while (true) {
    if (poll_feedback(fb)) {
      return true;
    }

    try {
      send_current(0);
    } catch (...) {
      // Keep trying until timeout.
    }
    std::this_thread::sleep_for(poll_sleep);

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - start).count() >= timeout_s) {
      return false;
    }
  }
}

void Dji3508Controller::reset_turn_reference() {
  ref_ticks_ = total_ticks_;
  current_turn_ = 0.0;
}

bool Dji3508Controller::has_feedback() const { return has_feedback_; }

double Dji3508Controller::current_turn() const { return current_turn_; }

int16_t Dji3508Controller::last_command_current() const { return last_cmd_current_; }

MoveResult Dji3508Controller::move_relative_turns(double delta_turn,
                                                  const PositionMoveOptions &options) {
  MoveResult result;

  PositionMoveOptions opt = options;
  if (opt.loop_hz <= 0) {
    opt.loop_hz = 500;
  }
  if (opt.max_current < 1) {
    opt.max_current = 1;
  }
  opt.max_current = std::min(opt.max_current, kCurrentAbsMax);
  if (std::abs(opt.move_bias_current) > kCurrentAbsMax) {
    opt.move_bias_current =
        (opt.move_bias_current > 0) ? kCurrentAbsMax : -kCurrentAbsMax;
  }
  if (opt.tolerance_turn <= 0.0) {
    opt.tolerance_turn = 0.01;
  }
  if (opt.timeout_s <= 0.0) {
    opt.timeout_s = 4.0;
  }
  if (opt.settle_hold_ms < 0) {
    opt.settle_hold_ms = 0;
  }

  if (!ensure_feedback(kInitialFeedbackTimeoutS)) {
    stop_and_relax(200);
    result.final_turn = current_turn_;
    result.final_cmd_current = 0;
    return result;
  }

  reset_turn_reference();
  const double target_turn = delta_turn;
  const auto start = std::chrono::steady_clock::now();
  auto next_tick = start;
  bool in_settle = false;
  auto settle_begin = start;
  const int period_us = std::max(1, 1000000 / opt.loop_hz);

  try {
    while (true) {
      const auto now = std::chrono::steady_clock::now();

      Feedback fb;
      poll_feedback(fb);
      if (has_feedback_) {
        const double stale_s = std::chrono::duration<double>(now - last_feedback_tp_).count();
        if (stale_s > kFeedbackLostTimeoutS) {
          stop_and_relax(200);
          result.final_turn = current_turn_;
          result.final_cmd_current = 0;
          return result;
        }
      }

      const double err = target_turn - current_turn_;
      double cmd = opt.kp * err - opt.kd * static_cast<double>(last_feedback_.rpm);
      if (opt.move_bias_current != 0 && std::fabs(err) > (opt.tolerance_turn * 0.5)) {
        const int bias_mag = std::abs(opt.move_bias_current);
        if (cmd > 0.0) {
          cmd += static_cast<double>(bias_mag);
        } else if (cmd < 0.0) {
          cmd -= static_cast<double>(bias_mag);
        } else {
          cmd = (err >= 0.0) ? static_cast<double>(bias_mag) : -static_cast<double>(bias_mag);
        }
      }
      const int clipped =
          static_cast<int>(std::lround(std::max(-static_cast<double>(opt.max_current),
                                                std::min(static_cast<double>(opt.max_current),
                                                         cmd))));
      send_current(static_cast<int16_t>(clipped));

      result.final_turn = current_turn_;
      result.final_cmd_current = static_cast<int16_t>(clipped);

      if (std::fabs(err) <= opt.tolerance_turn) {
        if (!in_settle) {
          in_settle = true;
          settle_begin = now;
        } else if (std::chrono::duration_cast<std::chrono::milliseconds>(now - settle_begin)
                       .count() >= opt.settle_hold_ms) {
          result.reached = true;
          result.timeout = false;
          return result;
        }
      } else {
        in_settle = false;
      }

      if (std::chrono::duration<double>(now - start).count() >= opt.timeout_s) {
        result.reached = false;
        result.timeout = true;
        stop_and_relax(200);
        result.final_turn = current_turn_;
        result.final_cmd_current = 0;
        return result;
      }

      next_tick += std::chrono::microseconds(period_us);
      std::this_thread::sleep_until(next_tick);
      const auto after_sleep = std::chrono::steady_clock::now();
      if (after_sleep > next_tick + std::chrono::microseconds(period_us)) {
        next_tick = after_sleep;
      }
    }
  } catch (...) {
    stop_and_relax(200);
    throw;
  }
}

void Dji3508Controller::stop_and_relax(int safe_stop_ms) {
  if (safe_stop_ms <= 0) {
    try {
      send_current(0);
    } catch (...) {
      // best effort
    }
    return;
  }

  const auto until =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(safe_stop_ms);
  while (std::chrono::steady_clock::now() < until) {
    try {
      send_current(0);
    } catch (...) {
      // best effort
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

}  // namespace dji_3508
}  // namespace motor_ros2
