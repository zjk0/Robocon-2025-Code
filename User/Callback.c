#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "usart.h"
#include "leg_control.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        Can_Receive_new(&Can, &hcan1);
        AnalyseJ60MotorReceiveData_new(J60Motor_CAN1);
    }
    else if (hcan->Instance == CAN2) {
        Can_Receive_new(&Can, &hcan2);
        AnalyseJ60MotorReceiveData_new(J60Motor_CAN2);
    }
	// else {
	// 	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxInformation1, Can.ReceiveCanData.data);
	// }
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//     if (GPIO_Pin == GPIO_PIN_1) {
//         TrotEnable = 1;
//     }
// }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART8) {
        if (controller_signal[0] == 1 && controller_signal[1] != 1 && controller_signal[2] != 1 && controller_signal[3] != 1) {
            TrotDirection = Advance;
            Trotbegin = 1;
        }
        else if (controller_signal[0] != 1 && controller_signal[1] == 1 && controller_signal[2] != 1 && controller_signal[3] != 1) {
            TrotDirection = Retreat;
            Trotbegin = 1;
        }
        else if (controller_signal[0] != 1 && controller_signal[1] != 1 && controller_signal[2] == 1 && controller_signal[3] != 1) {
            TrotDirection = Turn_left;
        }
        else if (controller_signal[0] != 1 && controller_signal[1] != 1 && controller_signal[2] != 1 && controller_signal[3] == 1) {
            TrotDirection = Turn_right;
        }
        else {
            TrotDirection = 0;
            Trotbegin = 1;
        }
        HAL_UART_Receive_IT(&huart8, (uint8_t*)controller_signal, 4);
    }
}
