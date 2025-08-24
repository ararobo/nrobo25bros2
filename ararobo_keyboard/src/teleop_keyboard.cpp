#include "ararobo_keyboard/teleop_keyboard.hpp"
#include "ararobo_keyboard/key_code_configure.hpp"

TeleopKeyboard::TeleopKeyboard() : Node("teleop_keyboard")
{
    keyboard = std::make_shared<KeyboardDriver>();
    key = std::make_shared<KeyState>();
    keyboard->init(event_path_.c_str());
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_arm_ = this->create_publisher<ararobo_msgs::msg::ArmData>("arm_target", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TeleopKeyboard::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "start");
}

TeleopKeyboard::~TeleopKeyboard()
{
    keyboard->release();
}

void TeleopKeyboard::timer_callback()
{
    geometry_msgs::msg::Twist msg;
    uint8_t key_code, key_state;
    keyboard->get_event(&key_code, &key_state);
    double x = 0.0, y = 0.0, omega = 0.0;
    double hand_width = 0.0, hand_depth = 0.0;
    key->update_state(key_code, key_state);
    if (key->w.is_pushed)
    {
        y += linear_speed_;
    }
    if (key->a.is_pushed)
    {
        x -= linear_speed_;
    }
    if (key->s.is_pushed)
    {
        y -= linear_speed_;
    }
    if (key->d.is_pushed)
    {
        x += linear_speed_;
    }
    if (key->t.is_pushed)
    {
        hand_depth += hand_depth_speed_;
    }
    if (key->f.is_pushed)
    {
        hand_width -= hand_width_speed_;
    }
    if (key->g.is_pushed)
    {
        hand_depth -= hand_depth_speed_;
    }
    if (key->h.is_pushed)
    {
        hand_width += hand_width_speed_;
    }
    geometry_msgs::msg::Twist twist_msg;
    ararobo_msgs::msg::ArmData arm_msg;
    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.angular.z = omega;
    arm_msg.depth = hand_depth;
    arm_msg.width = hand_width;
    pub_cmd_->publish(twist_msg);
    pub_arm_->publish(arm_msg);
}

void KeyState::update_state(uint8_t code, uint8_t state)
{
    if (state == 0 || state == 1)
    {
        switch (code)
        {
        case key_code::w:
            w.is_pushed = state;
            break;

        case key_code::a:
            a.is_pushed = state;
            break;

        case key_code::s:
            s.is_pushed = state;
            break;

        case key_code::d:
            d.is_pushed = state;
            break;

        case key_code::t:
            t.is_pushed = state;
            break;

        case key_code::g:
            g.is_pushed = state;
            break;

        default:
            break;
        }
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopKeyboard>());
    rclcpp::shutdown();
    return 0;
}
