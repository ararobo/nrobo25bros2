#include "ararobo_keyboard/teleop_keyboard.hpp"
#include "ararobo_keyboard/key_code_configure.hpp"

TeleopKeyboard::TeleopKeyboard() : Node("teleop_keyboard")
{
    keyboard = std::make_shared<KeyboardDriver>();
    key = std::make_shared<KeyState>();
    keyboard->init(event_path_.c_str());
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_arm_ = this->create_publisher<ararobo_msgs::msg::ArmData>("arm_target", 10);
    pub_lift_ = this->create_publisher<std_msgs::msg::Float32>("lift_vel", 10);
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
    double lift_vel = 0.0;
    double right_slide = 0.0, right_raise = 0.0, left_slide = 0.0, left_raise = 0.0;
    key->update_state(key_code, key_state);
    // mecanum
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
    if (key->right.is_pushed)
    {
        omega -= angular_speed_;
    }
    if (key->left.is_pushed)
    {
        omega += angular_speed_;
    }
    // upper_hand
    if (key->h.is_pushed)
    {
        hand_depth += hand_depth_speed_;
    }
    if (key->t.is_pushed)
    {
        hand_width -= hand_width_speed_;
    }
    if (key->f.is_pushed)
    {
        hand_depth -= hand_depth_speed_;
        if (key->x.is_pushed)
        {
            hold_flag_ = true;
        }
    }
    else
    {
        if (key->x.is_pushed)
        {
            hold_flag_ = false;
        }
    }
    if (key->g.is_pushed)
    {
        hand_width += hand_width_speed_;
    }
    // under_hand
    if (key->y.is_pushed)
    {
        left_raise -= raise_speed_;
    }
    if (key->u.is_pushed)
    {
        left_raise += raise_speed_;
    }
    if (key->i.is_pushed)
    {
        right_raise += raise_speed_;
    }
    if (key->o.is_pushed)
    {
        right_raise -= raise_speed_;
    }
    if (key->j.is_pushed)
    {
        left_slide += slide_speed_;
    }
    if (key->n.is_pushed)
    {
        left_slide -= slide_speed_;
    }
    if (key->k.is_pushed)
    {
        right_slide += slide_speed_;
    }
    if (key->m.is_pushed)
    {
        right_slide -= slide_speed_;
    }
    // lift
    if (key->up.is_pushed)
    {
        lift_vel += lift_speed_;
    }
    if (key->down.is_pushed)
    {
        lift_vel -= lift_speed_;
    }
    if (key->shift.is_pushed)
    {
        x *= shift_rate_;
        y *= shift_rate_;
        omega *= shift_rate_;
        hand_depth *= shift_rate_;
        hand_width *= shift_rate_;
        lift_vel *= shift_rate_;
        left_raise *= shift_rate_;
        right_raise *= shift_rate_;
        left_slide *= shift_rate_;
        right_slide *= shift_rate_;
    }

    if (hold_flag_)
    {
        hand_depth = -hand_depth_speed_;
    }

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.angular.z = omega;
    pub_cmd_->publish(twist_msg);
    ararobo_msgs::msg::ArmData arm_msg;
    arm_msg.upper_hand_depth = hand_depth;
    arm_msg.upper_hand_width = hand_width;
    arm_msg.left_raise = left_raise;
    arm_msg.left_slide = left_slide;
    arm_msg.right_raise = right_raise;
    arm_msg.right_slide = right_slide;
    pub_arm_->publish(arm_msg);
    std_msgs::msg::Float32 lift_msg;
    lift_msg.data = lift_vel;
    pub_lift_->publish(lift_msg);
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
        case key_code::f:
            f.is_pushed = state;
            break;
        case key_code::g:
            g.is_pushed = state;
            break;
        case key_code::h:
            h.is_pushed = state;
            break;
        case key_code::y:
            y.is_pushed = state;
            break;
        case key_code::u:
            u.is_pushed = state;
            break;
        case key_code::j:
            j.is_pushed = state;
            break;
        case key_code::i:
            i.is_pushed = state;
            break;
        case key_code::k:
            k.is_pushed = state;
            break;
        case key_code::o:
            o.is_pushed = state;
            break;
        case key_code::p:
            p.is_pushed = state;
            break;
        case key_code::n:
            n.is_pushed = state;
            break;
        case key_code::m:
            m.is_pushed = state;
            break;
        case key_code::up:
            up.is_pushed = state;
            break;
        case key_code::down:
            down.is_pushed = state;
            break;
        case key_code::left:
            left.is_pushed = state;
            break;
        case key_code::right:
            right.is_pushed = state;
            break;
        case key_code::shift_l:
            shift.is_pushed = state;
            break;
        case key_code::x:
            x.is_pushed = state;
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
