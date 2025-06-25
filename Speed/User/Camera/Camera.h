#pragma once

#include "stdint.h"
#include "string.h"
#include "main.h"

#define CAMERA_RX_HEADER_1 0x0D
#define CAMERA_RX_HEADER_2 0x0A
#define CAMERA_RX_FOOTER_1 0x0A
#define CAMERA_RX_FOOTER_2 0x0D

#define CAMERA_RX_HEADER 0x0A
#define CAMERA_RX_FOOTER 0x0D

#define RX_BTYES_LENGTH 3

#define RX_NORMAL 1
#define RX_ERROR 0

typedef union {
    uint8_t bytes[16];
    float data[4];
} RxFloat;

typedef union {
    uint8_t bytes[4];
    int data;
} RxInt;

typedef struct {
    float yaw;
    int stop_flag;
} Camera;

extern uint8_t rx_bytes[RX_BTYES_LENGTH];
extern Camera camera;

extern int mid_value;

int check_rx_bytes (uint8_t* rx_bytes);
void parse_camera_bytes (uint8_t* rx_bytes);
