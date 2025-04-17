#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "main.h"
#include "stdint.h"
#include "string.h"

/**
 * ----------------------------------- Marcos -----------------------------------
 */
#define DMA_BUFFER_SIZE 128
#define HANDLE_DATA_SIZE 12
#define CMD_SIZE 6

#define CMD_FLAG_1 0xFB
#define CMD_FLAG_2 0xFC
#define CMD_FLAG_3 0xFD
#define CMD_FLAG_4 0xFE

#define CMD_ERROR 0
#define CMD_NORMAL 1

#define TROT_FORWARD_CMD 0x01
#define TROT_BACK_CMD 0x04
#define ROTATE_LEFT_CMD 0x02
#define ROTATE_RIGHT_CMD 0x08
#define TURN_LEFT_CMD 0x20
#define TURN_RIGHT_CMD 0x80
#define JUMP_UP_CMD 0x10
#define JUMP_FORWARD_CMD 0x40
#define SLOPE_CMD 0x01
#define SLOPE_LR_CMD 0x02
#define BECOME_HIGHER 0x10
#define BECOME_LOWWER 0x20
#define STOP_CMD 0x00
#define NO_CMD 0x00

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern uint8_t dma_buffer[DMA_BUFFER_SIZE];
extern uint8_t dma_buffer_copy[DMA_BUFFER_SIZE];
extern uint8_t handle_command[CMD_SIZE];

extern uint16_t parse_start_pos;
extern uint16_t dma_pointer_pos;

/**
 * ----------------------------------- Functions -----------------------------------
 */
int CheckRxData (uint8_t* rx_data);
void ParseHandle (uint8_t* rx_data, uint8_t* cmd);
int CompareCommand (uint8_t* last_cmd, uint8_t* cmd);