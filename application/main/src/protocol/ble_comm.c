/*
Copyright (C) 2019 Jim Jiang <jim@lotlab.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "../ble/ble_hid_service.h"
#include "../ble/ble_services.h"
#include "keyboard_host_driver.h"
#include "passkey.h"
#include "hid_configuration.h"

static struct host_driver ble_driver;

static uint8_t ble_get_keyboard_leds()
{
    return keyboard_led_val_ble;
}

static bool ble_host_working()
{
    return m_conn_handle != BLE_CONN_HANDLE_INVALID;
}

static void ble_send_packet(enum packet_type type, uint8_t len, uint8_t* data)
{
    if (type == PACKET_CONF)
    {
        ble_send_conf(len, data);
        return;
    }
    // handle passkey input
    if (type == PACKET_KEYBOARD)
        passkey_input_handler(KEYBOARD_REPORT_SIZE - 2, &data[2]);

    if (type == PACKET_NKRO){ // 将包类型0x80转换成0x04
        type = 0x04;
    }
    keys_send(type, len, data);
}

/**
 * @brief 处理蓝牙HID设备接收到的数据
 *
 * 当蓝牙HID设备接收到数据时，调用此函数转发hid_on_recv进行处理。
 *
 * @param command 命令码
 * @param len 数据长度
 * @param data 数据指针
 */
void ble_hid_on_recv(uint8_t command, uint8_t len, uint8_t* data) {
    hid_on_recv(&ble_driver, command, len, data);
}

/**
 * @brief 蓝牙通信驱动
 * 
 */
static struct host_driver ble_driver = {
    .keyboard_leds = &ble_get_keyboard_leds,
    .queue_empty = &hid_queue_empty,
    .send_packet = &ble_send_packet,
    .driver_working = &ble_host_working,
    .mtu = 28,
};

// 以一个较低优先级注册蓝牙通信
KEYBOARD_HOST_DRIVER(8, ble_driver);
