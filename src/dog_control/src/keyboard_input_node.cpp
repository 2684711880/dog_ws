#include <rclcpp/rclcpp.hpp>

#include <dog_msgs/msg/locomotion_command.hpp>

#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <functional>
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

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags == -1) flags = 0;
    (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    configured_ = true;
  }

  void restore()
  {
    if (!configured_) return;
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    configured_ = false;
  }

  termios orig_{};
  bool configured_{false};
};

class KeyboardInputNode : public rclcpp::Node
{
public:
  KeyboardInputNode()
  : Node("keyboard_input_node")
  {
    declare_and_load_params();

    height_offset_cmd_ = 0.0;
    mode_cmd_ = dog_msgs::msg::LocomotionCommand::GAIT_STAND;
    last_motion_key_time_ = now();

    cmd_pub_ = create_publisher<dog_msgs::msg::LocomotionCommand>("/locomotion_cmd", 20);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&KeyboardInputNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "keyboard_input_node started. WASD move (W:+forward A:+left), QE yaw (Q:+left-turn), 1/2/3/4 gait, H/L height_offset (H:raise), X exit.");
  }

private:
  template<typename T>
  static T clamp_value(T v, T lo, T hi)
  {
    return std::min(std::max(v, lo), hi);
  }

  void declare_and_load_params()
  {
    declare_parameter<double>("keyboard_vx_fixed", 0.10);
    declare_parameter<double>("keyboard_vy_fixed", 0.08);
    declare_parameter<double>("keyboard_yaw_fixed", 0.35);  // +0.35 rad/s = CCW (left turn) in right-handed frame
    declare_parameter<double>("keyboard_height_step", 0.005);
    declare_parameter<double>("keyboard_key_timeout_s", 0.15);
    declare_parameter<double>("height_min", 0.24);
    declare_parameter<double>("height_max", 0.32);
    declare_parameter<double>("height_default", 0.31);

    vx_fixed_ = get_parameter("keyboard_vx_fixed").as_double();
    vy_fixed_ = get_parameter("keyboard_vy_fixed").as_double();
    yaw_fixed_ = get_parameter("keyboard_yaw_fixed").as_double();  // Left turn (CCW) is positive in right-handed Z-up frame
    height_step_ = get_parameter("keyboard_height_step").as_double();
    key_timeout_s_ = get_parameter("keyboard_key_timeout_s").as_double();
    height_min_ = get_parameter("height_min").as_double();
    height_max_ = get_parameter("height_max").as_double();
    height_default_ = get_parameter("height_default").as_double();
    height_offset_min_ = height_min_ - height_default_;
    height_offset_max_ = height_max_ - height_default_;
  }

  void handle_key(char ch)
  {
    switch (ch) {
      case 'w':
      case 'W':
        vx_cmd_ = vx_fixed_;
        last_motion_key_time_ = now();
        break;
      case 's':
      case 'S':
        vx_cmd_ = -vx_fixed_;
        last_motion_key_time_ = now();
        break;
      case 'a':
      case 'A':
        vy_cmd_ = vy_fixed_;
        last_motion_key_time_ = now();
        break;
      case 'd':
      case 'D':
        vy_cmd_ = -vy_fixed_;
        last_motion_key_time_ = now();
        break;
      case 'q':
      case 'Q':
        yaw_cmd_ = yaw_fixed_;
        last_motion_key_time_ = now();
        break;
      case 'e':
      case 'E':
        yaw_cmd_ = -yaw_fixed_;
        last_motion_key_time_ = now();
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
        // H raises body by reducing clearance target (clearance down = body up in right-handed Z-up frame), so offset decreases.
        height_offset_cmd_ = clamp_value(
          height_offset_cmd_ - height_step_, height_offset_min_, height_offset_max_);
        break;
      case 'l':
      case 'L':
        // L lowers body by increasing clearance target (clearance up = body down in right-handed Z-up frame), so offset increases.
        height_offset_cmd_ = clamp_value(
          height_offset_cmd_ + height_step_, height_offset_min_, height_offset_max_);
        break;
      case ' ':
        vx_cmd_ = 0.0;
        vy_cmd_ = 0.0;
        yaw_cmd_ = 0.0;
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

  void timer_callback()
  {
    char ch = 0;
    while (reader_.read_char(ch)) {
      handle_key(ch);
    }

    if ((now() - last_motion_key_time_).seconds() > key_timeout_s_) {
      vx_cmd_ = 0.0;
      vy_cmd_ = 0.0;
      yaw_cmd_ = 0.0;
    }

    dog_msgs::msg::LocomotionCommand cmd;
    cmd.header.stamp = now();
    cmd.source = "keyboard";
    cmd.gait_mode = mode_cmd_;
    cmd.vx = static_cast<float>(vx_cmd_);
    cmd.vy = static_cast<float>(vy_cmd_);
    cmd.yaw_rate = static_cast<float>(yaw_cmd_);
    cmd.height = static_cast<float>(height_offset_cmd_);
    cmd_pub_->publish(cmd);
  }

private:
  KeyboardReader reader_;
  rclcpp::Publisher<dog_msgs::msg::LocomotionCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double vx_fixed_{0.10};
  double vy_fixed_{0.08};
  double yaw_fixed_{0.35};
  double height_step_{0.005};
  double key_timeout_s_{0.15};
  double height_min_{0.24};
  double height_max_{0.32};
  double height_default_{0.31};
  double height_offset_min_{-0.04};
  double height_offset_max_{0.04};

  uint8_t mode_cmd_{dog_msgs::msg::LocomotionCommand::GAIT_STAND};
  double vx_cmd_{0.0};
  double vy_cmd_{0.0};
  double yaw_cmd_{0.0};
  double height_offset_cmd_{0.0};
  rclcpp::Time last_motion_key_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardInputNode>());
  rclcpp::shutdown();
  return 0;
}
