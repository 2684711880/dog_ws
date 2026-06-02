#include <rclcpp/rclcpp.hpp>

#include <dog_msgs/msg/locomotion_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "quad/locomotion_controller.h"
#include "quad/leg_kinematics.h"
#include "quad/types.h"

using namespace std::chrono_literals;
namespace lm = quad::lm;

class QuadLocomotionNode : public rclcpp::Node
{
public:
  QuadLocomotionNode()
  : Node("quad_locomotion_node")
  {
    // --- 通用参数 ---
    declare_parameter<std::string>("input_source", "xbox");
    declare_parameter<double>("cmd_timeout_s", 0.25);
    declare_parameter<double>("command_filter_alpha", 0.40);
    declare_parameter<double>("vx_slew_rate", 0.80);
    declare_parameter<double>("vy_slew_rate", 0.60);
    declare_parameter<double>("yaw_slew_rate", 2.5);
    declare_parameter<double>("height_slew_rate", 0.05);
    declare_parameter<double>("height_min", 0.22);
    declare_parameter<double>("height_max", 0.32);
    declare_parameter<double>("height_default", 0.295);
    declare_parameter<bool>("debug_locomotion", false);
    declare_parameter<int>("debug_locomotion_period_ms", 500);
    declare_parameter<double>("safety_joint_margin", 0.01);
    declare_parameter<int>("safety_ik_fail_threshold", 10);

    // --- STEPPING 参数 ---
    declare_parameter<double>("stepping_step_height", 0.025);
    declare_parameter<double>("stepping_frequency", 2.0);
    declare_parameter<double>("stepping_duty_factor", 0.65);

    // --- WALK 参数 ---
    declare_parameter<double>("walk_step_height", 0.045);
    declare_parameter<double>("walk_frequency", 0.833);
    declare_parameter<double>("walk_duty_factor", 0.80);
    declare_parameter<double>("walk_max_step_length", 0.06);
    declare_parameter<double>("walk_swing_scale_front", 1.0);
    declare_parameter<double>("walk_swing_scale_rear", 1.0);
    declare_parameter<bool>("walk_j1_lock_forward", false);

    // --- TROT 参数 ---
    declare_parameter<double>("trot_step_height", 0.060);
    declare_parameter<double>("trot_frequency", 1.6);
    declare_parameter<double>("trot_duty_factor", 0.55);
    declare_parameter<double>("trot_max_step_length", 0.080);
    declare_parameter<double>("trot_swing_scale_front", 1.0);
    declare_parameter<double>("trot_swing_scale_rear", 1.0);
    declare_parameter<double>("trot_yaw_lateral_gain", 1.0);
    declare_parameter<bool>("trot_j1_lock_forward", false);

    // --- 共用 ---
    declare_parameter<double>("min_step_height", 0.050);
    declare_parameter<double>("yaw_step_gain", 1.0);
    declare_parameter<double>("yaw_step_gain_in_place", 1.2);
    declare_parameter<double>("in_place_turn_vx_threshold", 0.03);
    declare_parameter<double>("in_place_turn_vy_threshold", 0.02);

    input_source_ = get_parameter("input_source").as_string();

    // --- 构建控制器参数 ---
    quad::LocomotionControllerParams cp;

    // 滤波
    cp.filter.alpha = get_parameter("command_filter_alpha").as_double();
    cp.filter.vx_slew_rate = get_parameter("vx_slew_rate").as_double();
    cp.filter.vy_slew_rate = get_parameter("vy_slew_rate").as_double();
    cp.filter.yaw_slew_rate = get_parameter("yaw_slew_rate").as_double();
    cp.filter.height_slew_rate = get_parameter("height_slew_rate").as_double();
    cp.filter.cmd_timeout_s = get_parameter("cmd_timeout_s").as_double();
    cp.filter.height_min = get_parameter("height_min").as_double();
    cp.filter.height_max = get_parameter("height_max").as_double();
    height_default_ = get_parameter("height_default").as_double();

    // 安全
    cp.safety.joint_margin = get_parameter("safety_joint_margin").as_double();
    cp.safety.ik_fail_threshold = get_parameter("safety_ik_fail_threshold").as_int();

    // 共用 foot 参数
    quad::FootPlannerParams foot_common;
    foot_common.min_step_height = get_parameter("min_step_height").as_double();
    foot_common.yaw_step_gain = get_parameter("yaw_step_gain").as_double();
    foot_common.yaw_step_gain_in_place = get_parameter("yaw_step_gain_in_place").as_double();
    foot_common.yaw_lateral_gain = 1.0;
    foot_common.in_place_turn_vx_threshold = get_parameter("in_place_turn_vx_threshold").as_double();
    foot_common.in_place_turn_vy_threshold = get_parameter("in_place_turn_vy_threshold").as_double();

    // STAND
    cp.gait_stand.type = quad::GaitType::STAND;
    cp.gait_stand.frequency = 0.0;
    cp.gait_stand.duty_factor = 1.0;
    cp.gait_stand.phase_offsets = quad::default_phase_offsets(quad::GaitType::STAND);

    // STEPPING
    cp.gait_stepping.type = quad::GaitType::STEPPING;
    cp.gait_stepping.frequency = get_parameter("stepping_frequency").as_double();
    cp.gait_stepping.duty_factor = get_parameter("stepping_duty_factor").as_double();
    cp.gait_stepping.step_height = get_parameter("stepping_step_height").as_double();
    cp.gait_stepping.phase_offsets = quad::default_phase_offsets(quad::GaitType::STEPPING);

    // WALK
    cp.gait_walk.type = quad::GaitType::WALK;
    cp.gait_walk.frequency = get_parameter("walk_frequency").as_double();
    cp.gait_walk.duty_factor = get_parameter("walk_duty_factor").as_double();
    cp.gait_walk.step_height = get_parameter("walk_step_height").as_double();
    cp.gait_walk.phase_offsets = quad::default_phase_offsets(quad::GaitType::WALK);

    cp.foot_walk = foot_common;
    cp.foot_walk.step_height = get_parameter("walk_step_height").as_double();
    cp.foot_walk.max_step_length = get_parameter("walk_max_step_length").as_double();
    cp.foot_walk.swing_scale_front = get_parameter("walk_swing_scale_front").as_double();
    cp.foot_walk.swing_scale_rear = get_parameter("walk_swing_scale_rear").as_double();
    cp.foot_walk.j1_lock_forward = get_parameter("walk_j1_lock_forward").as_bool();
    cp.foot_walk.gait_frequency = cp.gait_walk.frequency;

    // TROT
    cp.gait_trot.type = quad::GaitType::TROT;
    cp.gait_trot.frequency = get_parameter("trot_frequency").as_double();
    cp.gait_trot.duty_factor = get_parameter("trot_duty_factor").as_double();
    cp.gait_trot.step_height = get_parameter("trot_step_height").as_double();
    cp.gait_trot.phase_offsets = quad::default_phase_offsets(quad::GaitType::TROT);

    cp.foot_trot = foot_common;
    cp.foot_trot.step_height = get_parameter("trot_step_height").as_double();
    cp.foot_trot.max_step_length = get_parameter("trot_max_step_length").as_double();
    cp.foot_trot.swing_scale_front = get_parameter("trot_swing_scale_front").as_double();
    cp.foot_trot.swing_scale_rear = get_parameter("trot_swing_scale_rear").as_double();
    cp.foot_trot.yaw_lateral_gain = get_parameter("trot_yaw_lateral_gain").as_double();
    cp.foot_trot.j1_lock_forward = get_parameter("trot_j1_lock_forward").as_bool();
    cp.foot_trot.gait_frequency = cp.gait_trot.frequency;

    // 默认 gait (构造用)
    cp.gait = cp.gait_stand;
    cp.foot = foot_common;

    // 运动学
    kin_ = std::make_unique<quad::LegKinematics>(quad::make_default_leg_configs());
    cp.kin = kin_.get();

    // 控制器
    ctrl_ = std::make_unique<quad::LocomotionController>(cp);

    joint_names_ = lm::joint_names_vector();

    // ROS 接口
    cmd_sub_ = create_subscription<dog_msgs::msg::LocomotionCommand>(
      "/locomotion_cmd", 50,
      std::bind(&QuadLocomotionNode::cmd_callback, this, std::placeholders::_1));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);
    gait_pub_ = create_publisher<std_msgs::msg::String>("/gait_params", 20);
    debug_pub_ = create_publisher<std_msgs::msg::String>("/locomotion_debug", 10);
    debug_locomotion_ = get_parameter("debug_locomotion").as_bool();
    debug_locomotion_period_ms_ = get_parameter("debug_locomotion_period_ms").as_int();

    timer_ = create_wall_timer(5ms, std::bind(&QuadLocomotionNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "quad_locomotion_node started. input_source=%s cmd_timeout=%.2f",
      input_source_.c_str(), get_parameter("cmd_timeout_s").as_double());
  }

private:
  void cmd_callback(const dog_msgs::msg::LocomotionCommand::SharedPtr msg)
  {
    if (!msg) return;
    if (msg->source != input_source_) return;

    quad::MotionCommand cmd;
    cmd.vx = static_cast<double>(msg->vx);
    cmd.vy = static_cast<double>(msg->vy);
    cmd.yaw_rate = static_cast<double>(msg->yaw_rate);
    cmd.body_height = height_default_ + static_cast<double>(msg->height);

    auto gait = quad::to_gait_type(msg->gait_mode);
    ctrl_->set_command(cmd, gait, now().seconds());
  }

  void timer_callback()
  {
    auto jc = ctrl_->step(0.005);

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.header.frame_id = "base_link";
    js.name = joint_names_;
    js.position = lm::flatten_to_vector(jc.positions);
    joint_pub_->publish(js);

    std_msgs::msg::String gait_msg;
    gait_msg.data = quad::gait_type_name(ctrl_->current_gait());
    gait_pub_->publish(gait_msg);

    if (debug_locomotion_) {
      publish_debug(gait_msg.data);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "gait=%s timeout=%d force_stand=%d",
      gait_msg.data.c_str(),
      ctrl_->is_timed_out() ? 1 : 0,
      ctrl_->should_force_stand() ? 1 : 0);
  }

  static const char *leg_name(std::size_t i)
  {
    switch (static_cast<lm::LegId>(i)) {
      case lm::LegId::LF: return "LF";
      case lm::LegId::RF: return "RF";
      case lm::LegId::LB: return "LB";
      case lm::LegId::RB: return "RB";
      default: return "??";
    }
  }

  void publish_debug(const std::string &gait_name)
  {
    const auto &dbg = ctrl_->last_debug();
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "gait=" << gait_name
       << " vx=" << dbg.command.vx
       << " vy=" << dbg.command.vy
       << " yaw=" << dbg.command.yaw_rate
       << " h=" << dbg.command.body_height;

    for (std::size_t i = 0; i < lm::kLegCount; ++i) {
      const auto &leg = dbg.legs[i];
      ss << " | " << leg_name(i)
         << (leg.in_stance ? ":ST" : ":SW")
         << " sp=" << leg.stance_progress
         << " wp=" << leg.swing_progress
         << " foot=(" << leg.foot_pos_j1.x << "," << leg.foot_pos_j1.y << "," << leg.foot_pos_j1.z << ")"
         << " q=(" << leg.joints.j1 << "," << leg.joints.j2 << "," << leg.joints.j3 << ")"
         << " ik=" << (leg.ik_success ? 1 : 0);
    }

    std_msgs::msg::String msg;
    msg.data = ss.str();
    debug_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_locomotion_period_ms_,
      "%s", msg.data.c_str());
  }

private:
  std::string input_source_;
  std::unique_ptr<quad::LegKinematics> kin_;
  std::unique_ptr<quad::LocomotionController> ctrl_;

  rclcpp::Subscription<dog_msgs::msg::LocomotionCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
  double height_default_{0.295};
  bool debug_locomotion_{false};
  int debug_locomotion_period_ms_{500};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadLocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
