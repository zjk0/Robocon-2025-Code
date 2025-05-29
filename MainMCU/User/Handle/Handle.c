#include "Handle.h"

uint8_t dma_buffer[DMA_BUFFER_SIZE] = {0};
uint8_t dma_buffer_copy[DMA_BUFFER_SIZE] = {0};
uint8_t handle_command[CMD_SIZE] = {0};

uint16_t parse_start_pos = 0;
uint16_t dma_pointer_pos = 0;

int CheckRxData (uint8_t* rx_data) {
    if (rx_data[0] != CMD_FLAG_1 || rx_data[1] != CMD_FLAG_1) {
        return CMD_ERROR;
    }
    else if (rx_data[3] != CMD_FLAG_2 || rx_data[4] != CMD_FLAG_2) {
        return CMD_ERROR;
    }
    else if (rx_data[6] != CMD_FLAG_3) {
        return CMD_ERROR;
    }
    else if (rx_data[9] != CMD_FLAG_4) {
        return CMD_ERROR;
    }

    if (rx_data[2] != 0x01 && rx_data[2] != 0x02 && rx_data[2] != 0x04 && rx_data[2] != 0x08 &&
        rx_data[2] != 0x10 && rx_data[2] != 0x20 && rx_data[2] != 0x40 && rx_data[2] != 0x80 &&
        rx_data[2] != 0x00) {
        
        return CMD_ERROR;
    }

    return CMD_NORMAL;
}

void ParseHandle (uint8_t* rx_data, uint8_t* cmd) {
    cmd[0] = rx_data[2];
    cmd[1] = rx_data[5];
    cmd[2] = rx_data[7];
    cmd[3] = rx_data[8];
    cmd[4] = rx_data[10];
    cmd[5] = rx_data[11];
}

int CompareCommand (uint8_t* last_cmd, uint8_t* cmd) {
    if (last_cmd[0] == cmd[0] && last_cmd[1] == cmd[1]) {
        return 0;
    }
    return 1;
    // return memcmp(last_cmd, cmd, sizeof(cmd));
}