#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "leg_control.h"
#include "GaitController.h"

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

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//     if (GPIO_Pin == GPIO_PIN_1) {
//         TrotEnable = 1;
//     }
// }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART8) {
        if (controller_signal[0] == 1 && controller_signal[1] != 1 && controller_signal[2] != 1 && controller_signal[3] != 1) {
            trot_controller.trot_state = PreTrot;
            trot_controller.trot_direction = Forward;
            TrotEnable = 1;
            RotateEnable = 0;
			t = 0;
			last_t = -1;
        }
        else if (controller_signal[0] != 1 && controller_signal[1] == 1 && controller_signal[2] != 1 && controller_signal[3] != 1) {
            trot_controller.trot_state = PreTrot;
            trot_controller.trot_direction = Back;
            TrotEnable = 1;
            RotateEnable = 0;
			t = 0;
			last_t = -1;
        }
        else if (controller_signal[0] != 1 && controller_signal[1] != 1 && controller_signal[2] == 1 && controller_signal[3] != 1) {
            if (trot_controller.trot_state == Trotting) {
                TrotStateChange = 1;
            }
            rotate_controller.rotate_state = PreRotate;
            rotate_controller.rotate_direction = Left;
            TrotEnable = 0;
            RotateEnable = 1;
			t = 0;
			last_t = -1;
        }
        else if (controller_signal[0] != 1 && controller_signal[1] != 1 && controller_signal[2] != 1 && controller_signal[3] == 1) {
            if (trot_controller.trot_state == Trotting) {
                TrotStateChange = 1;
            }
            rotate_controller.rotate_state = PreRotate;
            rotate_controller.rotate_direction = Right;
            TrotEnable = 0;
            RotateEnable = 1;
			t = 0;
			last_t = -1;
        }
        else {
            if (trot_controller.trot_state == Trotting) {
                TrotStateChange = 1;
            }
            if (rotate_controller.rotate_state == Rotating) {
                RotateStateChange = 1;
            }
        }
        HAL_UART_Receive_IT(&huart8, (uint8_t*)controller_signal, 4);
    }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (trot_controller.trot_state == PreTrot || trot_controller.trot_state == PreEndTrot) {
            if (t < 1200) {
                t += 40;
                if (t == 1200) {
                    TrotStateChange = 1;
                }
            }
            else {
                t = 0;
                last_t = -1;
            }
        }
        else if (rotate_controller.rotate_state == PreRotate || rotate_controller.rotate_state == PreEndRotate) {
            if (t < 1200) {
                t += 40;
                if (t == 1200) {
                    RotateStateChange = 1;
                }
            }
            else {
                t = 0;
                last_t = -1;
            }
        }
        else {
            if (t < 2000) {
                t += 40;
            }
            else {
                t = 0;
                last_t = -1;
            }
        }


        // if (t < 2000) {
        //     t += 5;
        //     if (t == 2000) {
        //         if (trot_controller.trot_state == PreTrot || trot_controller.trot_state == PreEndTrot) {
        //             TrotStateChange = 1;
        //         }
        //         if (rotate_controller.rotate_state == PreRotate || rotate_controller.rotate_state == PreEndTrot) {
        //             RotateStateChange = 1;
        //         }
        //     }
        //     if (trot_controller.trot_state == PreTrot || trot_controller.trot_state == PreEndTrot) {
        //         if (t == 1000) {
        //             t = 0;
        //             last_t = -1;
        //             TrotStateChange = 1;
        //         }
        //     }
        // }
        // else {
        //     t = 0;
        //     last_t = -1;        
        // }

        HAL_TIM_Base_Stop_IT(&htim2);
    }
}
