#include "ararobo_keyboard/teleop_keyboard.hpp"
#include "ararobo_keyboard/key_code_configure.hpp"

TeleopKeyboard::TeleopKeyboard() : Node("teleop_keyboard")
{
    keyboard = std::make_shared<KeyboardDriver>();
    keyboard->init(event_path_.c_str());
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&TeleopKeyboard::timer_callback, this));
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
    double x, y, omega = 0;
}