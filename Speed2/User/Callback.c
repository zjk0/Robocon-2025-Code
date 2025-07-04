#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "usart.h"
#include "tim.h"
#include "GaitController.h"
#include "Handle.h"
#include "imu.h"
#include "main.h"
#include "string.h"
#include "Camera.h"

#define SLOW_DELTA_T 10
#define JUMP_LEGUP_DELTA_T 50
#define JUMP_SQUAT_STANDUP_DELTA_T 20

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        Can_Receive_new(&Can, &hcan1);
        AnalyseJ60MotorReceiveData_new(J60Motor_CAN1);
    }
    else if (hcan->Instance == CAN2) {
        Can_Receive_new(&Can, &hcan2);
        AnalyseJ60MotorReceiveData_new(J60Motor_CAN2);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    uint8_t rx_cmd[HANDLE_DATA_SIZE] = {0};
    BaseType_t higher_priority_task_wake = pdFALSE;

    if (huart->Instance == USART3) {
        memcpy(rx_cmd, dma_buffer, DMA_BUFFER_SIZE);
        if (cmd_queue != NULL) {
            xQueueSendFromISR(cmd_queue, rx_cmd, &higher_priority_task_wake);
        }
        portYIELD_FROM_ISR(higher_priority_task_wake);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, (uint8_t*)rx_bytes, RX_BTYES_LENGTH);
        
    }
}
