#include "ararobo_can/pc_data_slave.hpp"
#include <cstring>

PCDataSlave::PCDataSlave(uint8_t board_id_) : board_id(board_id_)
{
    // 受信フラグの初期化
    this->init_flag = false;
    this->target_flag = false;
    velocity_scale = 1000; // 送信時のベクトルの係数
}

void PCDataSlave::send_init(uint8_t config)
{
    uint8_t data[1] = {config};
    this->send(can_config::encode_id(can_config::direction::master,
                                     can_config::board_type::pc, this->board_id,
                                     can_config::data_type::pc::init),
               data, sizeof(data));
}

bool PCDataSlave::get_init(uint8_t *config)
{
    if (this->init_flag)
    {
        std::memcpy(config, this->init_buffer, sizeof(this->init_buffer));
        this->init_flag = false;
        return true;
    }
    return false;
}

bool PCDataSlave::get_odometry(float *x, float *y, float *theta)
{
    if (this->cmd_vel_flag)
    {
        int16_t x_int = (this->cmd_vel_buffer[0] | (this->cmd_vel_buffer[1] << 8));
        int16_t y_int = (this->cmd_vel_buffer[2] | (this->cmd_vel_buffer[3] << 8));
        int16_t theta_int = (this->cmd_vel_buffer[4] | (this->cmd_vel_buffer[5] << 8));
        *x = static_cast<float>(x_int) / velocity_scale;
        *y = static_cast<float>(y_int) / velocity_scale;
        *theta = static_cast<float>(theta_int) / velocity_scale;
        this->cmd_vel_flag = false;
        return true;
    }
    return false;
}

void PCDataSlave::send_cmd_vel(float vx, float vy, float omega)
{
    uint8_t data[6];
    int16_t vx_int = vx * velocity_scale;
    int16_t vy_int = vy * velocity_scale;
    int16_t omega_int = omega * velocity_scale;
    data[0] = static_cast<uint8_t>(vx_int & 0xFF);
    data[1] = static_cast<uint8_t>((vx_int >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vy_int & 0xFF);
    data[3] = static_cast<uint8_t>((vy_int >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(omega_int & 0xFF);
    data[5] = static_cast<uint8_t>((omega_int >> 8) & 0xFF);
    this->send(can_config::encode_id(can_config::direction::master,
                                     can_config::board_type::pc, this->board_id,
                                     can_config::data_type::pc::cmd_vel),
               data, sizeof(data));
}

void PCDataSlave::receive(uint16_t id, uint8_t *data, uint8_t len)
{
    can_config::decode_id(id, this->packet_direction, this->packet_board_type,
                          this->packet_board_id, this->packet_data_type);
    if (this->packet_direction == can_config::direction::master &&
        this->packet_board_type == can_config::board_type::pc &&
        this->packet_board_id == this->board_id)
    {
        switch (this->packet_data_type)
        {
        case can_config::data_type::pc::init:
            if (len == sizeof(this->init_buffer))
            {
                std::memcpy(this->init_buffer, data, sizeof(this->init_buffer));
                this->init_flag = true;
            }
            break;
        case can_config::data_type::pc::target:
            if (len == sizeof(this->target_buffer))
            {
                std::memcpy(this->target_buffer, data, sizeof(this->target_buffer));
                this->target_flag = true;
            }
            break;
        case can_config::data_type::pc::cmd_vel:
            if (len == sizeof(this->cmd_vel_buffer))
            {
                std::memcpy(this->cmd_vel_buffer, data, sizeof(this->cmd_vel_buffer));
                this->cmd_vel_flag = true;
            }
            break;
        default:
            break;
        }
    }
}

PCDataSlave::~PCDataSlave()
{
    CANDriver::~CANDriver();
}