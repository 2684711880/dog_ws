#include "motor_ros2/motor_cfg.h"
#include "motor_ros2/controller_helpers.h"
#include "motor_ros2/leg_model.h"

#include "dog_msgs/msg/motor_feedback.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <unistd.h>
#include <atomic>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include <array>
#include <string>
#include <vector>
#include <cmath>

namespace lm = motor_ros2::leg_model;

namespace {
// Set to true to enable fixed joint command offsets.
constexpr bool kEnableJointOffsets = true;

// Fixed offsets in joint-command space (rad), order:
// LF_J1, LF_J2, LF_J3, RF_J1, RF_J2, RF_J3, LB_J1, LB_J2, LB_J3, RB_J1, RB_J2, RB_J3.
// Edit these 12 values directly when tuning stand pose offsets.
constexpr std::array<float, lm::kJointCount> kFixedJointCmdOffsets = {
    0.0f, -0.8f, -2.3f,
    0.0f, -0.8f, -2.3f,
    0.0f, -1.0f, -2.3f,
    0.0f, -1.0f, -2.3f
};
}  // namespace
// J3,负值往下，J2正值往下。电机符号位不用管。
//      -0.1f, -0.8f, -2.6f,
//      0.1f, -0.8f, -2.6f,
//      0.1f, -1.0f, -2.6f,
//      -0.1f, -0.85f, -2.6f

class MotorControlSample : public rclcpp::Node
{
public:
    MotorControlSample()
    : rclcpp::Node("real_motor_controller"),
      motors_{{
          RobStrideMotor("can0", 0xFF, 0x01, 0),
          RobStrideMotor("can0", 0xFF, 0x02, 0),
          RobStrideMotor("can0", 0xFF, 0x03, 0),

          RobStrideMotor("can1", 0xFF, 0x04, 0),
          RobStrideMotor("can1", 0xFF, 0x05, 0),
          RobStrideMotor("can1", 0xFF, 0x06, 0),

          RobStrideMotor("can3", 0xFF, 0x0A, 0),
          RobStrideMotor("can3", 0xFF, 0x0B, 0),
          RobStrideMotor("can3", 0xFF, 0x0C, 0),

          RobStrideMotor("can2", 0xFF, 0x07, 0),
          RobStrideMotor("can2", 0xFF, 0x08, 0),
          RobStrideMotor("can2", 0xFF, 0x09, 0)
      }}
    {
        static_assert(lm::kJointCount == 12, "real_motor_controller expects 12 joints");

        joint_names_ = lm::joint_names_vector();
        for (std::size_t i = 0; i < lm::kJointCount; ++i) {
            name_to_index_.emplace(lm::kJointNames[i], i);
        }
        load_fixed_offset_config();
        build_hold_target_motor_from_offsets();
        log_offset_config();

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MotorControlSample::joint_cb, this, std::placeholders::_1)
        );

        gait_params_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/gait_params", 10,
            std::bind(&MotorControlSample::gait_params_cb, this, std::placeholders::_1)
        );

        motor_feedback_pub_ = this->create_publisher<dog_msgs::msg::MotorFeedback>(
            "/motor_feedback", 10);

        enable_enabled_.fill(false);
        hold_zero_only_.fill(false);
        joint_cmd_seen_.fill(false);

        // ========== 12个电机独立开关 ==========
        enable_enabled_[0]  = true;   // LF_J1 (左前-髋)
        enable_enabled_[1]  = true;   // LF_J2 (左前-大腿)
        enable_enabled_[2]  = true;   // LF_J3 (左前-小腿)
        enable_enabled_[3]  = true;   // RF_J1 (右前-髋)
        enable_enabled_[4]  = true;   // RF_J2 (右前-大腿)
        enable_enabled_[5]  = true;   // RF_J3 (右前-小腿)
        enable_enabled_[6]  = true;   // LB_J1 (左后-髋)
        enable_enabled_[7]  = true;   // LB_J2 (左后-大腿)
        enable_enabled_[8]  = true;   // LB_J3 (左后-小腿)
        enable_enabled_[9]  = true;   // RB_J1 (右后-髋)
        enable_enabled_[10] = true;   // RB_J2 (右后-大腿)
        enable_enabled_[11] = true;   // RB_J3 (右后-小腿)
        // =======================================

        for (std::size_t i = 0; i < motors_.size(); ++i)
        {
            if (!enable_enabled_[i]) continue;

            auto [p, v, tq, temp] = motors_[i].enable_motor();
            last_pos_fb_[i] = p;
            feedback_pos_[i] = p;
            feedback_vel_[i] = v;
            feedback_torque_[i] = tq;
            feedback_temp_[i] = temp;

            RCLCPP_INFO(this->get_logger(),
                        "Enable %-10s hold_zero_only=%d fb_raw=%.3f fb_wrap=%.3f",
                        joint_names_[i].c_str(),
                        static_cast<int>(hold_zero_only_[i]),
                        static_cast<double>(p),
                        static_cast<double>(motor_ros2::controller_helpers::wrap_to_pi(p)));

            usleep(1000);
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Soft-start: GROUPED parallel ramp to hold target (use_joint_offsets=%d)...",
            static_cast<int>(use_joint_offsets_));
        motor_ros2::controller_helpers::ramp_all_to_targets_grouped(
            motors_, enable_enabled_, last_pos_fb_, hold_target_motor_);
        RCLCPP_WARN(this->get_logger(), "Soft-start done.");

        running_ = true;
        worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);

        RCLCPP_INFO(this->get_logger(), "Controller started.");
    }

    ~MotorControlSample()
    {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        for (std::size_t i = 0; i < motors_.size(); ++i)
        {
            if (enable_enabled_[i]) {
                motors_[i].Disenable_Motor(0);
            }
        }
    }

private:
    enum class Mode { HOLD_TARGET, ACTIVE_FOLLOW };

    struct JointPd {
        float kp;
        float kd;
    };

    struct GaitParams {
        std::array<JointPd, lm::kJointsPerLeg> front_pd;  // J1, J2, J3
        std::array<JointPd, lm::kJointsPerLeg> rear_pd;   // J1, J2, J3
        float torque;
        float vel;
    };

    static constexpr GaitParams kStandPd{{
            JointPd{60.0f, 3.0f},
            JointPd{60.0f, 3.0f},
            JointPd{15.0f, 0.75f},
        }, {
            JointPd{60.0f, 3.0f},
            JointPd{60.0f, 3.0f},
            JointPd{15.0f, 0.75f},
        },
        0.0f, 0.0f};

    static constexpr GaitParams kSteppingPd{{
            JointPd{80.0f, 2.5f},
            JointPd{80.0f, 2.5f},
            JointPd{20.0f, 0.625f},
        }, {
            JointPd{80.0f, 2.5f},
            JointPd{80.0f, 2.5f},
            JointPd{20.0f, 0.625f},
        },
        0.0f, 0.0f};

    static constexpr GaitParams kWalkPd{{
            JointPd{80.0f, 2.5f},
            JointPd{80.0f, 2.5f},
            JointPd{20.0f, 0.625f},
        }, {
            JointPd{80.0f, 2.5f},
            JointPd{80.0f, 2.5f},
            JointPd{20.0f, 0.625f},
        },
        0.0f, 0.0f};

    static constexpr GaitParams kTrotPd{{
            JointPd{80.0f, 2.0f},
            JointPd{80.0f, 2.0f},
            JointPd{20.0f, 0.5f},
        }, {
            JointPd{80.0f, 2.0f},
            JointPd{80.0f, 2.0f},
            JointPd{80.0f, 2.0f},
        },
        0.0f, 0.0f};

    static constexpr GaitParams kFastTrotPd{{
            JointPd{60.0f, 1.5f},
            JointPd{60.0f, 1.5f},
            JointPd{15.0f, 0.375f},
        }, {
            JointPd{60.0f, 1.5f},
            JointPd{60.0f, 1.5f},
            JointPd{15.0f, 0.375f},
        },
        0.0f, 0.0f};

    static bool is_front_leg(lm::LegId leg)
    {
        return leg == lm::LegId::LF || leg == lm::LegId::RF;
    }

    static JointPd select_joint_pd(const GaitParams &params, lm::LegId leg, lm::JointId joint)
    {
        const auto joint_idx = static_cast<std::size_t>(joint);
        return is_front_leg(leg) ? params.front_pd[joint_idx] : params.rear_pd[joint_idx];
    }

    void load_fixed_offset_config()
    {
        use_joint_offsets_ = kEnableJointOffsets;
        joint_cmd_offsets_ = kFixedJointCmdOffsets;
    }

    float compute_motor_cmd_target(std::size_t idx, float joint_cmd) const
    {
        // Clamp the command (from /joint_states) BEFORE adding offset.
        // kJointLimits is in joint-command space, same as what manual_joint_pub sends.
        // Offset is a calibrated stand-pose value and must not be clamped.
        const float lo = lm::kJointLimits[idx].min_rad;
        const float hi = lm::kJointLimits[idx].max_rad;
        const float clamped_cmd = motor_ros2::controller_helpers::clampf(joint_cmd, lo, hi);
        const float joint_offset = use_joint_offsets_ ? joint_cmd_offsets_[idx] : 0.0f;
        return (clamped_cmd + joint_offset) * lm::kJointSigns[idx];
    }

    void build_hold_target_motor_from_offsets()
    {
        // Startup hold target uses the same mapping path as runtime follow path
        // to avoid HOLD_TARGET -> ACTIVE_FOLLOW target mismatch.
        hold_target_motor_.fill(0.0f);
        for (std::size_t i = 0; i < lm::kJointCount; ++i) {
            hold_target_motor_[i] = compute_motor_cmd_target(i, 0.0f);
        }
    }

    void log_offset_config() const
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Startup hold target mode: use_joint_offsets=%d",
            static_cast<int>(use_joint_offsets_));
        for (std::size_t i = 0; i < lm::kJointCount; ++i) {
            const float cfg_cmd = joint_cmd_offsets_[i];
            const float cfg_motor = cfg_cmd * lm::kJointSigns[i];
            RCLCPP_INFO(
                this->get_logger(),
                "  %-10s offset_cmd=%+.4f offset_motor=%+.4f applied_target_motor=%+.4f",
                lm::kJointNames[i],
                static_cast<double>(cfg_cmd),
                static_cast<double>(cfg_motor),
                static_cast<double>(hold_target_motor_[i]));
        }
    }

    void gait_params_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(gait_params_mutex_);
        const std::string gait_name = msg ? msg->data : std::string();

        if (gait_name == "STAND") {
            current_gait_params_ = kStandPd;
        } else if (gait_name == "STEPPING") {
            current_gait_params_ = kSteppingPd;
        } else if (gait_name == "WALK") {
            current_gait_params_ = kWalkPd;
        } else if (gait_name == "TROT") {
            current_gait_params_ = kTrotPd;
        } else if (gait_name == "FAST_TROT") {
            current_gait_params_ = kFastTrotPd;
        }
    }

    void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!msg) return;
        if (msg->name.size() != msg->position.size()) return;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                auto it = name_to_index_.find(msg->name[i]);
                if (it == name_to_index_.end()) continue;

                const std::size_t idx = it->second;
                joint_cmd_flat_[idx] = static_cast<float>(msg->position[i]);
                joint_cmd_seen_[idx] = true;
            }
        }

        got_first_js_.store(true);
        last_js_ns_.store(this->now().nanoseconds());
    }

    void publish_motor_feedback()
    {
        dog_msgs::msg::MotorFeedback msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.position.resize(feedback_pos_.size());
        for (std::size_t i = 0; i < feedback_pos_.size(); ++i) {
            // Publish position in joint-command space and subtract configured stand offset.
            // This keeps "stand pose" near 0 when /joint_states command is 0.
            const float motor_wrap = motor_ros2::controller_helpers::wrap_to_pi(feedback_pos_[i]);
            const float joint_cmd_space = motor_wrap * lm::kJointSigns[i];
            const float joint_offset = use_joint_offsets_ ? joint_cmd_offsets_[i] : 0.0f;
            msg.position[i] = motor_ros2::controller_helpers::wrap_to_pi(joint_cmd_space - joint_offset);
        }
        msg.velocity.assign(feedback_vel_.begin(), feedback_vel_.end());
        msg.torque.assign(feedback_torque_.begin(), feedback_torque_.end());
        msg.temperature.assign(feedback_temp_.begin(), feedback_temp_.end());
        motor_feedback_pub_->publish(msg);
    }

    void excute_loop()
    {
        const double timeout_s = 0.20;
        const auto loop_period = std::chrono::milliseconds(5);
        auto next_tick = std::chrono::steady_clock::now();
        Mode mode = Mode::HOLD_TARGET;

        // RCLCPP_WARN(this->get_logger(), "控制循环已启动，开始运行...");
        // std::cout << "[DEBUG] 控制循环启动" << std::endl;

        while (running_)
        {
            auto sleep_to_next_tick = [&]() {
                next_tick += loop_period;
                std::this_thread::sleep_until(next_tick);
                const auto now = std::chrono::steady_clock::now();
                if (now > next_tick + loop_period) {
                    next_tick = now;
                }
            };

            const bool got = got_first_js_.load();
            const int64_t now_ns  = this->now().nanoseconds();
            const int64_t last_ns = last_js_ns_.load();
            const double dt = (now_ns - last_ns) / 1e9;
            const bool js_ok = got && (dt <= timeout_s);

            if (js_ok && mode != Mode::ACTIVE_FOLLOW)
            {
                mode = Mode::ACTIVE_FOLLOW;
                // RCLCPP_WARN(this->get_logger(), "joint_states OK -> ACTIVE_FOLLOW");
            }
            if (!js_ok && mode != Mode::HOLD_TARGET)
            {
                mode = Mode::HOLD_TARGET;
                // RCLCPP_WARN(this->get_logger(), "joint_states timeout -> HOLD_TARGET (grouped ramp to configured hold target)");
                motor_ros2::controller_helpers::ramp_all_to_targets_grouped(
                    motors_, enable_enabled_, last_pos_fb_, hold_target_motor_);
            }

            if (mode == Mode::HOLD_TARGET)
            {
                publish_motor_feedback();
                sleep_to_next_tick();
                continue;
            }

            std::array<float, lm::kJointCount> cmd_flat_snapshot{};
            std::array<bool, lm::kJointCount> cmd_seen_snapshot{};
            {
                std::lock_guard<std::mutex> lk(mtx_);
                cmd_flat_snapshot = joint_cmd_flat_;
                cmd_seen_snapshot = joint_cmd_seen_;
            }
            const lm::QuadJoints<float> cmd_quad = lm::unflatten(cmd_flat_snapshot);

            for (std::size_t li = 0; li < lm::kLegCount; ++li)
            {
                const lm::LegId leg = static_cast<lm::LegId>(li);
                for (std::size_t ji = 0; ji < lm::kJointsPerLeg; ++ji)
                {
                    const lm::JointId joint = static_cast<lm::JointId>(ji);
                    const std::size_t idx = lm::flat_index(leg, joint);

                    if (!enable_enabled_[idx]) continue;
                    if (hold_zero_only_[idx]) continue;

                    if (!cmd_seen_snapshot[idx]) {
                        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        //     "未找到关节 %s 的缓存数据", lm::kJointNames[idx]);
                        continue;
                    }

                    const float cmd_joint = cmd_quad.at(leg).at(joint);
                    const float cmd = compute_motor_cmd_target(idx, cmd_joint);

                    const float cmd_send = motor_ros2::controller_helpers::nearest_equivalent(cmd, last_pos_fb_[idx]);

                    const JointPd pd = select_joint_pd(current_gait_params_, leg, joint);
                    const float kp = pd.kp;
                    const float kd = pd.kd;
                    const float torque = current_gait_params_.torque;
                    const float vel = current_gait_params_.vel;

                    auto [pos_fb, vel_fb, tq, temp] =
                        motors_[idx].send_motion_command(torque, cmd_send, vel, kp, kd);

                    last_pos_fb_[idx] = pos_fb;
                    feedback_pos_[idx] = pos_fb;
                    feedback_vel_[idx] = vel_fb;
                    feedback_torque_[idx] = tq;
                    feedback_temp_[idx] = temp;
                }
            }

            publish_motor_feedback();
            sleep_to_next_tick();
        }
    }

private:
    std::thread worker_thread_;
    std::atomic<bool> running_{false};

    std::array<RobStrideMotor, lm::kJointCount> motors_;
    std::vector<std::string> joint_names_;

    std::array<float, lm::kJointCount> last_pos_fb_{};
    std::array<float, lm::kJointCount> feedback_pos_{};
    std::array<float, lm::kJointCount> feedback_vel_{};
    std::array<float, lm::kJointCount> feedback_torque_{};
    std::array<float, lm::kJointCount> feedback_temp_{};

    std::array<bool, lm::kJointCount> enable_enabled_{};
    std::array<bool, lm::kJointCount> hold_zero_only_{};
    bool use_joint_offsets_{false};
    std::array<float, lm::kJointCount> joint_cmd_offsets_{};
    std::array<float, lm::kJointCount> hold_target_motor_{};

    std::array<float, lm::kJointCount> joint_cmd_flat_{};
    std::array<bool, lm::kJointCount> joint_cmd_seen_{};
    std::unordered_map<std::string, std::size_t> name_to_index_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_params_sub_;
    rclcpp::Publisher<dog_msgs::msg::MotorFeedback>::SharedPtr motor_feedback_pub_;

    GaitParams current_gait_params_{kStandPd};
    std::mutex gait_params_mutex_;

    std::mutex mtx_;
    std::atomic<bool> got_first_js_{false};
    std::atomic<int64_t> last_js_ns_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlSample>());
    rclcpp::shutdown();
    return 0;
}
