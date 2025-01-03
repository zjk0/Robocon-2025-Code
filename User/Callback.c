#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {
        MoveEnable = 1;
    }
}
