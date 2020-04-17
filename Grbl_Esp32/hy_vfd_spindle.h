/*
    hy_vfd_spindle.h
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
*/


#define HY_VFD_SPINDLE_UART_PORT   UART_NUM_2
#define ECHO_TEST_CTS         UART_PIN_NO_CHANGE
#define HY_VFD_SPINDLE_ADDR         1

#define MODBUS_ADDR_BYTE    0
#define MODBUS_CMD_BYTE     1
#define MODBUS_LEN_BYTE     2

#define HY_VFD_SPINDLE_RESPONSE_WAIT_TICKS 80 // how long to wait for a response

#define HY_VFD_SPINDLE_BUF_SIZE    127

// PD164 setting
#define HY_VFD_SPINDLE_BAUD_RATE   9600

#ifndef hy_vfd_spindle_h
#define hy_vfd_spindle_h

void hy_vfd_spindle_init();
void hy_vfd_spindle_set_speed(float rpm);
void hy_vfd_spindle_set_state(uint8_t state, float rpm);
uint16_t ModRTU_CRC(char* buf, int len);
bool hy_vfd_spindle_get_response(uint16_t length);
uint32_t hy_vfd_spindle_report_speed();

#endif
