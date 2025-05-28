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

#define NORMAL_DELTA_T 50
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
    dma_pointer_pos = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
    uint8_t rx_cmd[HANDLE_DATA_SIZE] = {0};
    BaseType_t higher_priority_task_wake = pdFALSE;

    if (huart->Instance == USART6) {
        memcpy(dma_buffer_copy, dma_buffer, DMA_BUFFER_SIZE);

        while ((dma_pointer_pos > parse_start_pos && dma_pointer_pos - parse_start_pos >= HANDLE_DATA_SIZE) || 
               (dma_pointer_pos < parse_start_pos && DMA_BUFFER_SIZE + dma_pointer_pos - parse_start_pos >= HANDLE_DATA_SIZE)) {
                
            if (dma_buffer_copy[parse_start_pos] == CMD_FLAG_1 && dma_buffer_copy[(parse_start_pos + 1) % DMA_BUFFER_SIZE] == CMD_FLAG_1) {
                if (parse_start_pos + HANDLE_DATA_SIZE < DMA_BUFFER_SIZE) {
                    memcpy(rx_cmd, &dma_buffer_copy[parse_start_pos], HANDLE_DATA_SIZE);
                }
                else {
                    uint16_t first_length = DMA_BUFFER_SIZE - parse_start_pos;
                    uint16_t second_length = HANDLE_DATA_SIZE - first_length;
                    memcpy(rx_cmd, &dma_buffer_copy[parse_start_pos], first_length);
                    memcpy(&rx_cmd[first_length], &dma_buffer_copy[0], second_length);
                }
                
                if (cmd_queue != NULL) {
                    xQueueSendFromISR(cmd_queue, rx_cmd, &higher_priority_task_wake);
                }
                portYIELD_FROM_ISR(higher_priority_task_wake);
                
                parse_start_pos = (parse_start_pos + HANDLE_DATA_SIZE) % DMA_BUFFER_SIZE;
            }
            else {
                parse_start_pos = (parse_start_pos + 1) % DMA_BUFFER_SIZE;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint8_t rx_imu[IMU_DATA_SIZE] = {0};
    BaseType_t higher_priority_task_wake = pdFALSE;

    if (huart->Instance == USART2) {
        memcpy(rx_imu, imu_rx_data, IMU_DATA_SIZE);
		if (imu_queue != NULL) {
            xQueueSendFromISR(imu_queue, rx_imu, &higher_priority_task_wake);
        }
        portYIELD_FROM_ISR(higher_priority_task_wake);

        HAL_UART_Receive_IT(&huart2, (uint8_t*)imu_rx_data, IMU_DATA_SIZE);
    }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
    BaseType_t higher_priority_task_wake = pdFALSE;

    if (htim->Instance == TIM2) {
        vTaskNotifyGiveFromISR((TaskHandle_t)NotifyActionHandle, &higher_priority_task_wake);
        portYIELD_FROM_ISR(higher_priority_task_wake);
    }

    HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
}
