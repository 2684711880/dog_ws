#pragma once

#include "motor_ros2/motor_cfg.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

namespace motor_ros2::controller_helpers {

static constexpr float TWO_PI = 6.283185307179586f;

inline float wrap_to_pi(float a)
{
    a = std::fmod(a + static_cast<float>(M_PI), TWO_PI);
    if (a < 0) a += TWO_PI;
    return a - static_cast<float>(M_PI);
}

// Map target to the nearest equivalent angle around current (target + k*2pi).
inline float nearest_equivalent(float target, float current)
{
    const float t = wrap_to_pi(target);
    const float k = std::round((current - t) / TWO_PI);
    return t + k * TWO_PI;
}

inline float clampf(float x, float lo, float hi)
{
    return std::min(std::max(x, lo), hi);
}

inline void ramp_motor_to_target(
    std::array<RobStrideMotor, 12> &motors,
    const std::array<bool, 12> &enable_enabled,
    std::array<float, 12> &last_pos_fb,
    size_t idx,
    float target_rad,
    float torque,
    float cmd_vel,
    float kp,
    float kd,
    int steps,
    int step_ms)
{
    if (!enable_enabled[idx]) return;

    const int n = std::max(steps, 1);
    const int sleep_ms = std::max(step_ms, 0);

    const float cur = last_pos_fb[idx];
    const float tgt = nearest_equivalent(target_rad, cur);
    const float err = tgt - cur;
    if (std::fabs(err) < 1e-4f) return;

    const float delta = err / static_cast<float>(n);

    for (int k = 1; k <= n; ++k) {
        const float inter = cur + delta * static_cast<float>(k);
        auto [pos_fb, vel_fb, tq, temp] =
            motors[idx].send_motion_command(torque, inter, cmd_vel, kp, kd);

        (void)vel_fb;
        (void)tq;
        (void)temp;
        last_pos_fb[idx] = pos_fb;

        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    // Ensure final command lands exactly on the wrapped target.
    auto [pos_fb, vel_fb, tq, temp] =
        motors[idx].send_motion_command(torque, tgt, cmd_vel, kp, kd);
    (void)vel_fb;
    (void)tq;
    (void)temp;
    last_pos_fb[idx] = pos_fb;
}

inline void ramp_group_parallel(
    std::array<RobStrideMotor, 12> &motors,
    const std::array<bool, 12> &enable_enabled,
    std::array<float, 12> &last_pos_fb,
    const std::vector<size_t> &idxs,
    float target_rad,
    float torque,
    float cmd_vel,
    float kp,
    float kd,
    int steps,
    int step_ms)
{
    std::vector<std::thread> ths;
    ths.reserve(idxs.size());

    for (const size_t idx : idxs) {
        if (!enable_enabled[idx]) continue;
        ths.emplace_back([&, idx]() {
            ramp_motor_to_target(
                motors, enable_enabled, last_pos_fb, idx,
                target_rad, torque, cmd_vel, kp, kd, steps, step_ms);
        });
    }

    for (auto &t : ths) {
        if (t.joinable()) t.join();
    }
}

inline void ramp_group_parallel_targets(
    std::array<RobStrideMotor, 12> &motors,
    const std::array<bool, 12> &enable_enabled,
    std::array<float, 12> &last_pos_fb,
    const std::vector<size_t> &idxs,
    const std::array<float, 12> &targets_rad,
    float torque,
    float cmd_vel,
    float kp,
    float kd,
    int steps,
    int step_ms)
{
    std::vector<std::thread> ths;
    ths.reserve(idxs.size());

    for (const size_t idx : idxs) {
        if (!enable_enabled[idx]) continue;
        ths.emplace_back([&, idx]() {
            ramp_motor_to_target(
                motors, enable_enabled, last_pos_fb, idx,
                targets_rad[idx], torque, cmd_vel, kp, kd, steps, step_ms);
        });
    }

    for (auto &t : ths) {
        if (t.joinable()) t.join();
    }
}

inline void ramp_all_to_targets_grouped(
    std::array<RobStrideMotor, 12> &motors,
    const std::array<bool, 12> &enable_enabled,
    std::array<float, 12> &last_pos_fb,
    const std::array<float, 12> &targets_rad)
{
    const float torque = 0.0f;
    const float cmd_vel = 0.0f;
    const float kp = 40.0f;
    const float kd = 2.5f;
    const int steps = 120;
    const int step_ms = 10;

    // index mapping:
    // g1: 0(ID1), 3(ID4), 9(ID7), 6(ID10)
    // g2: 1(ID2), 4(ID5), 10(ID8), 7(ID11)
    // g3: 2(ID3), 5(ID6), 11(ID9), 8(ID12)
    const std::vector<size_t> g1 = {0, 3, 9, 6};
    const std::vector<size_t> g2 = {1, 4, 10, 7};
    const std::vector<size_t> g3 = {2, 5, 11, 8};

    ramp_group_parallel_targets(motors, enable_enabled, last_pos_fb, g1, targets_rad, torque, cmd_vel, kp, kd, steps, step_ms);
    ramp_group_parallel_targets(motors, enable_enabled, last_pos_fb, g2, targets_rad, torque, cmd_vel, kp, kd, steps, step_ms);
    ramp_group_parallel_targets(motors, enable_enabled, last_pos_fb, g3, targets_rad, torque, cmd_vel, kp, kd, steps, step_ms);
}

inline void ramp_all_to_zero_grouped(
    std::array<RobStrideMotor, 12> &motors,
    const std::array<bool, 12> &enable_enabled,
    std::array<float, 12> &last_pos_fb)
{
    std::array<float, 12> zero_targets{};
    zero_targets.fill(0.0f);
    ramp_all_to_targets_grouped(motors, enable_enabled, last_pos_fb, zero_targets);
}

}  // namespace motor_ros2::controller_helpers
