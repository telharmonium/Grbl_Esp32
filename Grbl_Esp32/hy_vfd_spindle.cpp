/*
    hy_vfd_spindle.cpp
    Part of Grbl_ESP32

    Support for basic control of the Hungyang VFD spindle

    2020    - Bart Dring

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.


    Manual: http://www.hy-electrical.com/bf/inverter.pdf
    Reference: https://github.com/Smoothieware/Smoothieware/blob/edge/src/modules/tools/spindle/HuanyangSpindleControl.cpp
    Refernece: https://gist.github.com/Bouni/803492ed0aab3f944066

*/


#include "grbl.h"
#include <HardwareSerial.h>
#include "driver/uart.h"

#ifdef USE_HY_VFD_SPINDLE

const uart_port_t hy_vfd_spindle_uart_num = HY_VFD_SPINDLE_UART_PORT;
uint8_t hy_vfd_spindle_rx_message[50]; // received from vfd

void hy_vfd_spindle_init() {
    uart_config_t uart_config = {
        .baud_rate = HY_VFD_SPINDLE_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    uart_param_config(hy_vfd_spindle_uart_num, &uart_config);

    uart_set_pin(hy_vfd_spindle_uart_num,
                 HY_VFD_SPINDLE_TXD,
                 HY_VFD_SPINDLE_RXD,
                 HY_VFD_SPINDLE_RTS,
                 UART_PIN_NO_CHANGE);

    uart_driver_install(hy_vfd_spindle_uart_num,
                        HY_VFD_SPINDLE_BUF_SIZE * 2,
                        0,
                        0,
                        NULL,
                        0);

    uart_set_mode(hy_vfd_spindle_uart_num, UART_MODE_RS485_HALF_DUPLEX);
    // Allocate buffers for UART
    uint8_t* data = (uint8_t*) malloc(HY_VFD_SPINDLE_BUF_SIZE);
}

// wait for and get the servo response
uint16_t hy_vfd_spindle_get_response(uint16_t length) {
    length = uart_read_bytes(hy_vfd_spindle_uart_num, hy_vfd_spindle_rx_message, length, HY_VFD_SPINDLE_RESPONSE_WAIT_TICKS);
    return length;
}


/*
    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x01    0x31 0x88                   Start spindle clockwise
    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x08    0xF1 0x8E                   Stop spindle
    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x11    0x30 0x44                   Start spindle counter-clockwise
*/
void hy_vfd_spindle_set_state(uint8_t state, float rpm) {
    byte msg[6];
    uint16_t crc;

    msg[MODBUS_ADDR_BYTE] = HY_VFD_SPINDLE_ADDR;
    msg[MODBUS_CMD_BYTE] = 0x03;
    msg[MODBUS_LEN_BYTE] = 0x01;

    if (state == SPINDLE_DISABLE)   // Halt or set spindle direction and rpm.
        msg[3] = 0x08;
    else if (state == SPINDLE_ENABLE_CW)
        msg[3] = 0x01;
    else if (state == SPINDLE_ENABLE_CCW)
        msg[3] = 0x11;

    crc = ModRTU_CRC(msg, 4);
    msg[4] = (crc & 0xFF);
    msg[5] = (crc & 0xFF00) >> 8;

    grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "VFS Cmd 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",  msg[0],  msg[1],  msg[2],  msg[3],  msg[4],  msg[5]);

    //uart_write_bytes(hy_vfd_spindle_uart_num, msg, 6);
    //hy_vfd_spindle_get_response(6);


    if (state == SPINDLE_DISABLE)
        return;

    hy_vfd_spindle_set_speed(rpm);
    sys.report_ovr_counter = 0; // Set to report change immediately
}

/*
    ADDR    CMD     LEN     DATA        CRC
    0x01    0x05    0x02    0x09 0xC4   0xBF 0x0F               Write Frequency (0x9C4 = 2500 = 25.00HZ)
*/
void hy_vfd_spindle_set_speed(float rpm) {
    //float rpm = (float)pwm_value / (float)(1 << SPINDLE_PWM_BIT_PRECISION) * settings.rpm_max;
    byte msg[7];
    uint16_t crc;

    // fill in header bytes
    msg[MODBUS_ADDR_BYTE] = HY_VFD_SPINDLE_ADDR;
    msg[MODBUS_CMD_BYTE] = 0x05;
    msg[MODBUS_LEN_BYTE] = 0x02;

    // add data bytes
    uint16_t data = uint16_t(rpm / 60.0 * 100.0); // Hz * 10  (1500 RPM = 25Hz .... Send 2500)
    msg[3] = (data & 0xFF00) >> 8;
    msg[4] = (data & 0xFF);

    //calculate CRC
    crc = ModRTU_CRC(msg, 5);
    msg[5] = (crc & 0xFF);
    msg[6] = (crc & 0xFF00) >> 8;

    // TO DO send message rather than print it
    grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "VFS Cmd 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",  msg[0],  msg[1],  msg[2],  msg[3],  msg[4],  msg[5],  msg[6]);

    //uart_write_bytes(hy_vfd_spindle_uart_num, msg, 7);
    //hy_vfd_spindle_get_response(6);

}



// Compute the MODBUS RTU CRC
// https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
uint16_t ModRTU_CRC(byte* buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--) {    // Loop over each bit
            if ((crc & 0x0001) != 0) {      // If the LSB is set
                crc >>= 1;                    // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                          // Else LSB is not set
                crc >>= 1;                    // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}



#endif