#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "leg_control.h"
#include "GaitController.h"

#define FORWARD_SIGNAL 0x65
#define FORWARD_END_SIGNAL 0x45
#define BACK_SIGNAL 0x67
#define BACK_END_SIGNAL    0x47
#define LEFT_SIGNAL 0x66
#define LEFT_END_SIGNAL    0x46
#define RIGHT_SIGNAL 0x68
#define RIGHT_END_SIGNAL   0x48
#define IMMEDIATELY_STOP_SIGNAL 0x6E
#define JUMP_SIGNAL 0x64

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
        if(controller_signal[0] == 0xFF && controller_signal[1] == 0xFF){
            switch(controller_signal[2]){
                case FORWARD_SIGNAL:
                    trot_controller.trot_state = PreTrot;
                    trot_controller.trot_direction = Forward;
                    trot_controller.trot_enable = 1;
                    rotate_controller.rotate_enable = 0;
                    t = 0;
                    last_t = -1;
                    break;
                case BACK_SIGNAL:
                    trot_controller.trot_state = PreTrot;
                    trot_controller.trot_direction = Back;
                    trot_controller.trot_enable = 1;
                    rotate_controller.rotate_enable = 0;
                    jump_controller.jump_enable = 0;
                    t = 0;
                    last_t = -1;
                    break;
                case LEFT_SIGNAL:
                    rotate_controller.rotate_state = PreRotate;
                    rotate_controller.rotate_direction = Left;
                    trot_controller.trot_enable = 0;
                    rotate_controller.rotate_enable = 1;
                    jump_controller.jump_enable = 0;
                    t = 0;
                    last_t = -1;
                    break;
                case RIGHT_SIGNAL:
                    rotate_controller.rotate_state = PreRotate;
                    rotate_controller.rotate_direction = Right;
                    trot_controller.trot_enable = 0;
                    rotate_controller.rotate_enable = 1;
                    jump_controller.jump_enable = 0;
                    t = 0;
                    last_t = -1;
                    break;
                case JUMP_SIGNAL:
                    jump_controller.jump_state = Squat;
                    jump_controller.jump_enable = 1;
                    rotate_controller.rotate_enable = 0;
                    trot_controller.trot_enable = 0;
                    t = 0;
                    last_t = -1;
                    break;
                case FORWARD_END_SIGNAL:
                    trot_controller.trot_state_change = 1;
                    break;
                case BACK_END_SIGNAL:
                    trot_controller.trot_state_change = 1;
                    break;
                case LEFT_END_SIGNAL:
                    rotate_controller.rotate_state_change = 1;
                    break;
                case RIGHT_END_SIGNAL:
                    rotate_controller.rotate_state_change = 1;
                    break;
                case IMMEDIATELY_STOP_SIGNAL:
                    trot_controller.trot_state_change = 1;
                    rotate_controller.rotate_state_change = 1;
                    break;
                default:
                    break;
            }
        }
        HAL_UART_Receive_IT(&huart8, (uint8_t*)controller_signal, 3);
    }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (trot_controller.trot_state == PreTrot || trot_controller.trot_state == PreEndTrot) {
            if (t < 1000) {
                t += 40;
                if (t >= 1000) {
                    trot_controller.trot_state_change = 1;
                }
            }
        }
        else if (rotate_controller.rotate_state == PreRotate || rotate_controller.rotate_state == PreEndRotate) {
            if (t < 1000) {
                t += 40;
                if (t >= 1000) {
                    rotate_controller.rotate_state_change = 1;
                }
            }
        }
        else if (jump_controller.jump_state == Squat || jump_controller.jump_state == StandUp || jump_controller.jump_state == LegUp) {
            if (t < 1000) {
                if (jump_controller.jump_state == LegUp) {
                    t += 50;
                }
                else {
                    t += 20;
                }
                if (t >= 1000) {
                    jump_controller.jump_state_change = 1;
                }
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

        HAL_TIM_Base_Stop_IT(&htim2);
    }
}
