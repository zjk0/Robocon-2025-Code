#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "tim.h"
#include "main.h"
#include "leg_control.h"
#include "spi.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    if (hcan->Instance == CAN1) 
    {
        Can_Receive_new(&Can, &hcan1);
        AnalyseJ60MotorReceiveData_new(J60Motor_CAN1);
    }
    else if (hcan->Instance == CAN2) 
    {
        Can_Receive_new(&Can, &hcan2);
        AnalyseJ60MotorReceiveData_new(J60Motor_CAN2);
    }
	// else {
	// 	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxInformation1, Can.ReceiveCanData.data);
	// }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        // transfer_spi_to_can();
      spi_rx_flag = NORMAL_COMMAND;
        if (HAL_SPI_Receive_DMA(&hspi1, spi_motor_data.rx_motor_data, 60) != HAL_OK)
        {
            Error_Handler();
        }
    }
}