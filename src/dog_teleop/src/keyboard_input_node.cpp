#include <rclcpp/rclcpp.hpp>

#include <dog_msgs/msg/locomotion_command.hpp>

#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <functional>
#include <string>
#include <termios.h>
#include <unistd.h>

class KeyboardReader
{
public:
  KeyboardReader() { setup(); }
  ~KeyboardReader() { restore(); }

  bool read_char(char & ch)
  {
    unsigned char c = 0;
    const int n = ::read(STDIN_FILENO, &c, 1);
    if (n == 1) {
      ch = static_cast<char>(c);
      return true;
    }
    return false;
  }

private:
  void setup()
  {
    if (configured_) return;
    if (tcgetattr(STDIN_FILENO, &orig_) != 0) return;

    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_iflag &= ~(IXON | ICRNL);
    raw.c_oflag &= ~(OPOST);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;

    orig_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (orig_flags_ == -1) orig_flags_ = 0;
    (void)fcntl(STDIN_FILENO, F_SETFL, orig_flags_ | O_NONBLOCK);
    configured_ = true;
  }

  void restore()
  {
    if (!configured_) return;
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    (void)fcntl(STDIN_FILENO, F_SETFL, orig_flags_);
    configured_ = false;
  }

  termios orig_{};
  int orig_flags_{0};
  bool configured_{false};
};

class KeyboardInputNode : public rclcpp::Node
{
public:
  KeyboardInputNode()
  : Node("keyboard_input_node")
  {
    declare_and_load_params();

    cmd_pub_ = create_publisher<dog_msgs::msg::LocomotionCommand>("/locomotion_cmd", 20);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&KeyboardInputNode::timer_callback, this));

    height_offset_cmd_ = 0.0;
    mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STAND;

    RCLCPP_INFO(
      get_logger(),
      "keyboard_input_node started: WASD move, QE yaw, 1/2/3/4 gait, H/L height, Space stop, X exit. source=%s",
      source_.c_str());
  }

private:
  template<typename T>
  static T clamp_value(T v, T lo, T hi)
  {
    return std::min(std::max(v, lo), hi);
  }

  static double scaled_signed_axis(double axis, double positive_max, double negative_max)
  {
    return axis * ((axis >= 0.0) ? positive_max : negative_max);
  }

  void declare_and_load_params()
  {
    declare_parameter<double>("vx_max_walk_forward", 0.18);
    declare_parameter<double>("vx_max_walk_backward", 0.18);
    declare_parameter<double>("vx_max_trot_forward", 0.63);
    declare_parameter<double>("vx_max_trot_backward", 0.42);
    declare_parameter<double>("vy_max_walk", 0.03);
    declare_parameter<double>("vy_max_trot", 0.05);
    declare_parameter<double>("yaw_rate_max", 0.35);

    declare_parameter<double>("height_min", 0.24);
    declare_parameter<double>("height_max", 0.32);
    declare_parameter<double>("height_default", 0.31);
    declare_parameter<double>("height_step_per_input", 0.005);

    declare_parameter<double>("keyboard_key_timeout_s", 0.12);
    declare_parameter<double>("keyboard_initial_key_timeout_s", 0.40);
    declare_parameter<double>("keyboard_repeat_window_s", 0.08);
    declare_parameter<std::string>("source", "keyboard");

    vx_max_walk_forward_ = get_parameter("vx_max_walk_forward").as_double();
    vx_max_walk_backward_ = get_parameter("vx_max_walk_backward").as_double();
    vx_max_trot_forward_ = get_parameter("vx_max_trot_forward").as_double();
    vx_max_trot_backward_ = get_parameter("vx_max_trot_backward").as_double();
    vy_max_walk_ = get_parameter("vy_max_walk").as_double();
    vy_max_trot_ = get_parameter("vy_max_trot").as_double();
    yaw_rate_max_ = get_parameter("yaw_rate_max").as_double();

    height_min_ = get_parameter("height_min").as_double();
    height_max_ = get_parameter("height_max").as_double();
    height_default_ = get_parameter("height_default").as_double();
    height_step_per_input_ = get_parameter("height_step_per_input").as_double();
    height_offset_min_ = height_min_ - height_default_;
    height_offset_max_ = height_max_ - height_default_;

    key_timeout_s_ = get_parameter("keyboard_key_timeout_s").as_double();
    initial_key_timeout_s_ = get_parameter("keyboard_initial_key_timeout_s").as_double();
    repeat_window_s_ = get_parameter("keyboard_repeat_window_s").as_double();
    source_ = get_parameter("source").as_string();
  }

  // 将按键归一化为小写，用于 repeat 判断（W 和 w 视为同一个键）
  static char normalize_motion_key(char ch)
  {
    if (ch >= 'A' && ch <= 'Z') return static_cast<char>(ch + 32);
    return ch;
  }

  void mark_motion_key(char ch)
  {
    const auto t = now();
    const char norm = normalize_motion_key(ch);

    if (norm == last_motion_char_) {
      // 同一个键：在 repeat_window_s_ 内再次收到，确认为 OS key repeat
      if (motion_active_ && (t - last_motion_key_time_).seconds() < repeat_window_s_) {
        motion_repeat_seen_ = true;
      }
    } else {
      // 换了方向键：重置 repeat 状态，重新走长 timeout
      motion_repeat_seen_ = false;
    }

    last_motion_char_ = norm;
    motion_active_ = true;
    last_motion_key_time_ = t;
  }

  void handle_key(char ch)
  {
    switch (ch) {
      case 'w':
      case 'W':
        axis_vx_ = 1.0;
        mark_motion_key(ch);
        break;
      case 's':
      case 'S':
        axis_vx_ = -1.0;
        mark_motion_key(ch);
        break;
      case 'a':
      case 'A':
        axis_vy_ = 1.0;
        mark_motion_key(ch);
        break;
      case 'd':
      case 'D':
        axis_vy_ = -1.0;
        mark_motion_key(ch);
        break;
      case 'q':
      case 'Q':
        axis_yaw_ = 1.0;
        mark_motion_key(ch);
        break;
      case 'e':
      case 'E':
        axis_yaw_ = -1.0;
        mark_motion_key(ch);
        break;
      case '1':
        mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STAND;
        break;
      case '2':
        mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STEPPING;
        break;
      case '3':
        mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_WALK;
        break;
      case '4':
        mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_TROT;
        break;
      case 'h':
      case 'H':
        height_offset_cmd_ -= height_step_per_input_;
        height_offset_cmd_ = clamp_value(height_offset_cmd_, height_offset_min_, height_offset_max_);
        break;
      case 'l':
      case 'L':
        height_offset_cmd_ += height_step_per_input_;
        height_offset_cmd_ = clamp_value(height_offset_cmd_, height_offset_min_, height_offset_max_);
        break;
      case ' ':
        clear_motion_axes();
        break;
      case 'x':
      case 'X':
        RCLCPP_WARN(get_logger(), "Exit requested by keyboard.");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  void clear_motion_axes()
  {
    axis_vx_ = 0.0;
    axis_vy_ = 0.0;
    axis_yaw_ = 0.0;
    motion_active_ = false;
    motion_repeat_seen_ = false;
    last_motion_char_ = 0;
  }

  void timer_callback()
  {
    char ch = 0;
    while (reader_.read_char(ch)) {
      handle_key(ch);
    }

    // repeat 已确认用短 timeout，否则用长 timeout 等待首次 repeat 到来
    const double motion_timeout_s = motion_repeat_seen_ ? key_timeout_s_ : initial_key_timeout_s_;
    if (motion_active_ && (now() - last_motion_key_time_).seconds() > motion_timeout_s) {
      clear_motion_axes();
    }

    double vx = 0.0;
    double vy = 0.0;
    if (mode_cmd_ == dog_msgs::msg::LocomotionCommand::GAIT_WALK) {
      vx = scaled_signed_axis(axis_vx_, vx_max_walk_forward_, vx_max_walk_backward_);
      vy = axis_vy_ * vy_max_walk_;
    } else if (mode_cmd_ == dog_msgs::msg::LocomotionCommand::GAIT_TROT) {
      vx = scaled_signed_axis(axis_vx_, vx_max_trot_forward_, vx_max_trot_backward_);
      vy = axis_vy_ * vy_max_trot_;
    }

    const double yaw_rate =
      (mode_cmd_ == dog_msgs::msg::LocomotionCommand::GAIT_STAND) ? 0.0 : axis_yaw_ * yaw_rate_max_;

    dog_msgs::msg::LocomotionCommand cmd;
    cmd.header.stamp = now();
    cmd.source = source_;
    cmd.gait_mode = mode_cmd_;
    cmd.vx = static_cast<float>(vx);
    cmd.vy = static_cast<float>(vy);
    cmd.yaw_rate = static_cast<float>(yaw_rate);
    cmd.height = static_cast<float>(height_offset_cmd_);
    cmd_pub_->publish(cmd);
  }

private:
  KeyboardReader reader_;
  rclcpp::Publisher<dog_msgs::msg::LocomotionCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string source_{"keyboard"};

  double vx_max_walk_forward_{0.18};
  double vx_max_walk_backward_{0.18};
  double vx_max_trot_forward_{0.63};
  double vx_max_trot_backward_{0.42};
  double vy_max_walk_{0.03};
  double vy_max_trot_{0.05};
  double yaw_rate_max_{0.35};

  double height_min_{0.24};
  double height_max_{0.32};
  double height_default_{0.31};
  double height_offset_min_{-0.04};
  double height_offset_max_{0.04};
  double height_step_per_input_{0.005};

  // 消抖参数
  double key_timeout_s_{0.12};          // repeat 确认后的松键超时
  double initial_key_timeout_s_{0.40};  // 首次按下等待 repeat 的超时
  double repeat_window_s_{0.08};        // 同一键两次到达在此窗口内才算 repeat

  double height_offset_cmd_{0.0};
  double axis_vx_{0.0};
  double axis_vy_{0.0};
  double axis_yaw_{0.0};
  bool motion_active_{false};
  bool motion_repeat_seen_{false};
  char last_motion_char_{0};            // 上一次 motion 按键（归一化小写）
  rclcpp::Time last_motion_key_time_{0, 0, RCL_ROS_TIME};

  uint8_t mode_cmd_{dog_msgs::msg::LocomotionCommand::GAIT_STAND};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardInputNode>());
  rclcpp::shutdown();
  return 0;
}