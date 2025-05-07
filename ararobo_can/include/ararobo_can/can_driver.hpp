/**
 * @file can_driver.hpp
 * @author Gento Aiba
 * @brief CAN通信をLinuxで行うクラス
 * @version 2.0
 * @date 2024-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CANDriver
{
private:
    int can_socket;
    int rx_numbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame can_rx_frame;
    struct can_frame can_tx_frame;
    uint8_t RxData[8];

    /**
     * @brief Packetが受信可能かどうかを確認する
     *
     * @return true 受信可能
     * @return false 受信不可能
     */
    bool is_packet_available();

protected:
    /**
     * @brief CANの受信処理(オーバーライドしてください)
     *
     * @param id CANのID
     * @param data 受信データ
     * @param len データの長さ
     */
    virtual void receive(uint16_t id, uint8_t *data, uint8_t len);

public:
    /**
     * @brief デストラクタ
     *
     */
    ~CANDriver();

    /**
     * @brief CANの初期化
     *
     */
    void init();

    /**
     * @brief CANの送信処理
     *
     * @param id CANのID
     * @param data 送信するデータ
     * @param len データの長さ
     */
    void send(uint16_t id, uint8_t *data, uint8_t len);

    /**
     * @brief CANの受信処理
     * @note 受信処理を行う周期に合わせて継続的に呼び出す必要があります。
     *
     */
    void can_receive_process();
};