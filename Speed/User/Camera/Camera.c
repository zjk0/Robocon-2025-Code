#include "Camera.h"

uint8_t rx_bytes[RX_BTYES_LENGTH];
Camera camera;
int mid_value = -1;

uint16_t crc16_ccitt(uint8_t *data, uint16_t length) {
    uint8_t i;
    uint16_t crc = 0;        // Initial value
    while (length--) {
        crc ^= *data++;        // crc ^= *data; data++;
        for (i = 0; i < 8; i++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0x8408;        // 0x8408 = reverse 0x1021
            }
            else {
                crc = (crc >> 1);
            }
        }
    }
    return crc;
}

int check_rx_bytes (uint8_t* rx_bytes) {
    // uint8_t rx_data[RX_BTYES_LENGTH - 6];
    // memcpy(rx_data, &rx_bytes[2], (RX_BTYES_LENGTH - 6) * sizeof(uint8_t));

    // uint16_t crc_calc = crc16_ccitt(rx_data, RX_BTYES_LENGTH - 2);
    // uint16_t crc_real = rx_bytes[RX_BTYES_LENGTH - 4] | (rx_bytes[RX_BTYES_LENGTH - 3] << 8);

    // if (/*crc_calc == crc_real && */
    //     rx_bytes[0] == CAMERA_RX_HEADER_1 && rx_bytes[1] == CAMERA_RX_HEADER_2 && 
    //     rx_bytes[RX_BTYES_LENGTH - 2] == CAMERA_RX_FOOTER_1 && rx_bytes[RX_BTYES_LENGTH - 1] == CAMERA_RX_FOOTER_2) {
        
    //     return RX_NORMAL;
    // }
    // else {
    //     return RX_ERROR;
    // }

    if (rx_bytes[0] == CAMERA_RX_HEADER && rx_bytes[2] == CAMERA_RX_FOOTER) {
        return RX_NORMAL;
    }
    else {
        return RX_ERROR;
    }
}

void parse_camera_bytes (uint8_t* rx_bytes) {
    // RxFloat rx_float;
    // memcpy(rx_float.bytes, &rx_bytes[2], 16 * sizeof(uint8_t));
    // camera.roll = rx_float.data[0];
    // camera.pitch = rx_float.data[1];
    // camera.yaw = rx_float.data[2];
    // camera.yaw_angle = rx_float.data[3];

    // RxInt rx_int;
    // memcpy(rx_int.bytes, &rx_bytes[18], 4 * sizeof(uint8_t));
    // camera.stop_flag = rx_int.data;

    // camera.yaw = rx_bytes[2];
    // camera.stop_flag = rx_bytes[3];
    mid_value = rx_bytes[1];
}