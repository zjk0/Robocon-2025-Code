#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "KinematicSolution.h"
#include "GaitController.h"

#define NORMAL_DELTA_T 50
#define SLOW_DELTA_T 10
#define JUMP_LEGUP_DELTA_T 50
#define JUMP_SQUAT_STANDUP_DELTA_T 20

//#define FORWARD_SIGNAL 0x65
//#define FORWARD_END_SIGNAL 0x45
//#define BACK_SIGNAL 0x67
//#define BACK_END_SIGNAL    0x47
//#define LEFT_SIGNAL 0x66
//#define LEFT_END_SIGNAL    0x46
//#define RIGHT_SIGNAL 0x68
//#define RIGHT_END_SIGNAL   0x48
//#define IMMEDIATELY_STOP_SIGNAL 0
//#define JUMP_SIGNAL 0x64
//#define INCREASE_ROBOT_HEIGHT 0x6D
//#define DECREASE_ROBOT_HEIGHT 0x6E
//#define FLATLAND_TO_SLOPE 0x6A
//#define SLOPE_TO_FLATLAND 0x6B
//#define FLATLAND_TO_RL_SLOPE 0x6C

#define FORWARD_SIGNAL 0x61
#define FORWARD_END_SIGNAL 0x41
#define BACK_SIGNAL 0x63
#define BACK_END_SIGNAL    0x43
#define LEFT_SIGNAL_0 0x62
#define LEFT_END_SIGNAL_0    0x42
#define RIGHT_SIGNAL_0 0x64
#define RIGHT_END_SIGNAL_0   0x44
#define IMMEDIATELY_STOP_SIGNAL 0
#define JUMP_SIGNAL_0 0x65
#define JUMP_SIGNAL_1 0x67
#define LEFT_SIGNAL_1 0x66
#define LEFT_END_SIGNAL_1    0x46
#define RIGHT_SIGNAL_1 0x68
#define RIGHT_END_SIGNAL_1   0x48
#define INCREASE_ROBOT_HEIGHT 0x6D
#define DECREASE_ROBOT_HEIGHT 0x6E
#define FLATLAND_TO_SLOPE 0x6A
#define SLOPE_TO_FLATLAND_0 0x4A
#define FLATLAND_TO_RL_SLOPE 0x6B
#define SLOPE_TO_FLATLAND_1 0x4B

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

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if(huart->Instance == USART3)
//  {
//        if(rx_data[0] == 0xFF && rx_data[1] == 0xFF) 
//        {
//            switch (rx_data[2]) 
//            {
//                case FORWARD_SIGNAL:
//                    if (isSlope == 0) 
//                    {
//                        trot_controller.trot_state = PreTrot;
//                        trot_controller.trot_direction = Forward;
//                        trot_controller.trot_enable = 1;
//                    }
//                    else if (isSlope == 1) 
//                    {
//                        walk_slope_controller.trot_state = PreTrot;
//                        walk_slope_controller.trot_direction = Forward;
//                        walk_slope_controller.trot_enable = 1;
//                    }
//                    else if(isSlope == 2)
//                    {
//                        walk_LR_slope_controller.trot_state = PreTrot;
//                        walk_LR_slope_controller.trot_direction = Forward;
//                        walk_LR_slope_controller.trot_enable = 1;
//                    }
//                    t = 0;
//                    last_t = -1;
//                    break;
//                case BACK_SIGNAL:
//                    trot_controller.trot_state = PreTrot;
//                    trot_controller.trot_direction = Back;
//                    trot_controller.trot_enable = 1;
//                    t = 0;
//                    last_t = -1;
//                    break;
//                case LEFT_SIGNAL_0:
//                     rotate_controller.rotate_state = PreRotate;
//                     rotate_controller.rotate_direction = Left;
//                     rotate_controller.rotate_enable = 1;
//                     t = 0;
//                     last_t = -1;
//                case LEFT_SIGNAL_1:
//                    turn_controller.turn_state = PreTurn;
//                    turn_controller.turn_angular_direction = TurnLeft;
//                    turn_controller.turn_enable = 1;
//                    t = 0;
//                    last_t = -1;
//                    break;
//                case RIGHT_SIGNAL_0:
//                     rotate_controller.rotate_state = PreRotate;
//                     rotate_controller.rotate_direction = Right;
//                     rotate_controller.rotate_enable = 1;
//                     t = 0;
//                     last_t = -1;
//                     break;
//                case RIGHT_SIGNAL_1:
//                    turn_controller.turn_state = PreTurn;
//                    turn_controller.turn_angular_direction = TurnRight;
//                    turn_controller.turn_enable = 1;
//                    t = 0;
//                    last_t = -1;
//                    break;
//                case JUMP_SIGNAL_0:
//                    jump_up_controller.jump_state = Squat;
//                    jump_up_controller.jump_enable = 1;
//                    t = 0;
//                    last_t = -1;
//                    break;
//                case JUMP_SIGNAL_1:
//                    jump_forward_controller.jump_state = Squat;
//                    jump_forward_controller.jump_enable = 1;
//                    t = 0;
//                    last_t = -1;
//                    break;
//                case FORWARD_END_SIGNAL:
//                    if (isSlope == 0) 
//                    {
//                        trot_controller.trot_state_change = 1;
//                    }
//                    else if (isSlope == 1) 
//                    {
//                        walk_slope_controller.trot_state_change = 1;
//                    }
//                    else if(isSlope == 2)
//                    {
//                        walk_LR_slope_controller.trot_state_change = 1;
//                    }
//                    break;
//                case BACK_END_SIGNAL:
//                    trot_controller.trot_state_change = 1;
//                    break;
//                case LEFT_END_SIGNAL_0:
//                    rotate_controller.rotate_state_change = 1;
//                    break;
//                case LEFT_END_SIGNAL_1:
//	             turn_controller.turn_state_change = 1;
//                    break;
//                case RIGHT_END_SIGNAL_0:
//                     rotate_controller.rotate_state_change = 1;
//                     break;
//                case RIGHT_END_SIGNAL_1:
//                    turn_controller.turn_state_change = 1;
//                    break;
//                case IMMEDIATELY_STOP_SIGNAL:
//                    trot_controller.trot_state_change = 1;
//                    rotate_controller.rotate_state_change = 1;
//                    break;
//                case INCREASE_ROBOT_HEIGHT:
//                    robot_height += 0.01;
//                    break;
//                case DECREASE_ROBOT_HEIGHT:
//                    robot_height -= 0.01;
//                    break;
//                case FLATLAND_TO_SLOPE:
//                    isSlope = 1;
//                    // Stand_on_slope((1 / 3));
//                    break;
//                case SLOPE_TO_FLATLAND_0:
//                    isSlope = 0;
//                    break;
//                case SLOPE_TO_FLATLAND_1:
//                    isSlope = 0;
//                    // Stand();
//                    break;
//                case FLATLAND_TO_RL_SLOPE:
//                    isSlope = 2;
//                    // Stand_on_LR_slope(pi / 12);
//                    break;    
//                default:
//                    break;
//            }
//        }
////        HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_data, 3);
//        HAL_UART_Receive_DMA(&huart3, (uint8_t*)rx_data, 3);
//  }
//
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) 
    {
        if(controller_signal[0] == 0xFF && controller_signal[1] == 0xFF) 
        {
            switch (controller_signal[2]) 
            {
              case FORWARD_SIGNAL:
                    if (isSlope == 0) 
                    {
                        trot_controller.trot_state = PreTrot;
                        trot_controller.trot_direction = Forward;
                        trot_controller.trot_enable = 1;
                    }
                    else if (isSlope == 1) 
                    {
                        walk_slope_controller.trot_state = PreTrot;
                        walk_slope_controller.trot_direction = Forward;
                        walk_slope_controller.trot_enable = 1;
                    }
                    else if(isSlope == 2)
                    {
                        walk_LR_slope_controller.trot_state = PreTrot;
                        walk_LR_slope_controller.trot_direction = Forward;
                        walk_LR_slope_controller.trot_enable = 1;
                    }
                    t = 0;
                    last_t = -1;
                    break;
                case BACK_SIGNAL:
                    trot_controller.trot_state = PreTrot;
                    trot_controller.trot_direction = Back;
                    trot_controller.trot_enable = 1;
                    t = 0;
                    last_t = -1;
                    break;
                case LEFT_SIGNAL_0:
                     rotate_controller.rotate_state = PreRotate;
                     rotate_controller.rotate_direction = Left;
                     rotate_controller.rotate_enable = 1;
                     t = 0;
                     last_t = -1;
                     break;
                case LEFT_SIGNAL_1:
                    turn_controller.turn_state = PreTurn;
                    turn_controller.turn_angular_direction = TurnLeft;
                    turn_controller.turn_enable = 1;
                    t = 0;
                    last_t = -1;
                    break;
                case RIGHT_SIGNAL_0:
                     rotate_controller.rotate_state = PreRotate;
                     rotate_controller.rotate_direction = Right;
                     rotate_controller.rotate_enable = 1;
                     t = 0;
                     last_t = -1;
                     break;
                case RIGHT_SIGNAL_1:
                    turn_controller.turn_state = PreTurn;
                    turn_controller.turn_angular_direction = TurnRight;
                    turn_controller.turn_enable = 1;
                    t = 0;
                    last_t = -1;
                    break;
                case JUMP_SIGNAL_0:
                    jump_up_controller.jump_state = Squat;
                    jump_up_controller.jump_enable = 1;
                    t = 0;
                    last_t = -1;
                    break;
                case JUMP_SIGNAL_1:
                    jump_forward_controller.jump_state = Squat;
                    jump_forward_controller.jump_enable = 1;
                    t = 0;
                    last_t = -1;
                    break;
                case FORWARD_END_SIGNAL:
                    if (isSlope == 0) 
                    {
                        trot_controller.trot_state_change = 1;
                    }
                    else if (isSlope == 1) 
                    {
                        walk_slope_controller.trot_state_change = 1;
                    }
                    else if(isSlope == 2)
                    {
                        walk_LR_slope_controller.trot_state_change = 1;
                    }
                    break;
                case BACK_END_SIGNAL:
                    trot_controller.trot_state_change = 1;
                    break;
                case LEFT_END_SIGNAL_0:
                    rotate_controller.rotate_state_change = 1;
                    break;
                case LEFT_END_SIGNAL_1:
	             turn_controller.turn_state_change = 1;
                    break;
                case RIGHT_END_SIGNAL_0:
                     rotate_controller.rotate_state_change = 1;
                     break;
                case RIGHT_END_SIGNAL_1:
                    turn_controller.turn_state_change = 1;
                    break;
                case IMMEDIATELY_STOP_SIGNAL:
                    trot_controller.trot_state_change = 1;
                    rotate_controller.rotate_state_change = 1;
                    break;
                case INCREASE_ROBOT_HEIGHT:
                    robot_height += 0.01;
                    break;
                case DECREASE_ROBOT_HEIGHT:
                    robot_height -= 0.01;
                    break;
                case FLATLAND_TO_SLOPE:
                    isSlope = 1;
                    // Stand_on_slope((1 / 3));
                    break;
                case SLOPE_TO_FLATLAND_0:
                    isSlope = 0;
                    break;
                case SLOPE_TO_FLATLAND_1:
                    isSlope = 0;
                    // Stand();
                    break;
                case FLATLAND_TO_RL_SLOPE:
                    isSlope = 2;
                    // Stand_on_LR_slope(pi / 12);
                    break;    
                default:
                    break;
            }
        }
        HAL_UART_Receive_IT(&huart6, (uint8_t*)controller_signal, 3);
    }
    
    
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (trot_controller.trot_state == PreTrot || trot_controller.trot_state == PreEndTrot) {
            if (t < 1000) {
                t += NORMAL_DELTA_T;
                if (t >= 1000) {
                    trot_controller.trot_state_change = 1;
                }
            }
        }
        else if (rotate_controller.rotate_state == PreRotate || rotate_controller.rotate_state == PreEndRotate) {
            if (t < 1000) 
            {
                t += NORMAL_DELTA_T;
                if (t >= 1000) 
                {
                    rotate_controller.rotate_state_change = 1;
                }
            }
        }
        else if (jump_up_controller.jump_state == Squat || jump_up_controller.jump_state == StandUp || jump_up_controller.jump_state == LegUp) {
            if (t < 1000) 
            {
                if (jump_up_controller.jump_state == LegUp) 
                {
                    t += JUMP_LEGUP_DELTA_T;
                }
                else {
                    t += JUMP_SQUAT_STANDUP_DELTA_T;
                }
                if (t >= 1000) 
                {
                    jump_up_controller.jump_state_change = 1;
                }
            }
        }
        else if (jump_forward_controller.jump_state == Recline || jump_forward_controller.jump_state == Squat || jump_forward_controller.jump_state == StandUp || jump_forward_controller.jump_state == LegUp) {
            if (t < 1000) {
                if (jump_forward_controller.jump_state == LegUp) {
                    t += JUMP_LEGUP_DELTA_T;
                }
                else if(jump_forward_controller.jump_state == Recline)
                {
                      t += 20;
                }
                else {
                    t += JUMP_SQUAT_STANDUP_DELTA_T;
                }
                if (t >= 1000) {                
                    jump_forward_controller.jump_state_change = 1;
                }
            }
        }
        else if (turn_controller.turn_state == PreTurn || turn_controller.turn_state == PreEndTurn) {
            if (t < 1000) {
                t += NORMAL_DELTA_T;
                if (t >= 1000) {
                    turn_controller.turn_state_change = 1;
                }
            }
        }
        else if (walk_slope_controller.trot_state == PreTrot || walk_slope_controller.trot_state == PreEndTrot) {
            if (t < 1000) {
                t += NORMAL_DELTA_T;
                if (t >= 1000) {
                    walk_slope_controller.trot_state_change = 1;
                }
            }
        }
       else if (walk_LR_slope_controller.trot_state == PreTrot || walk_LR_slope_controller.trot_state == PreEndTrot) 
       {
            if (t < 1000) 
            {
                t += SLOW_DELTA_T;
                if (t >= 1000) 
                {
                    walk_LR_slope_controller.trot_state_change = 1;
                }
            }
        }
       else if (walk_LR_slope_controller.trot_state == Trotting) 
       {
            if (t < 2000) 
            {
                t += SLOW_DELTA_T;
            }
            else
            {
                t = 0;
                last_t = -1;
            }
        }
        else {
            if (t < 2000) 
            {
                t += NORMAL_DELTA_T;
            }
            else {
                t = 0;
                last_t = -1;
            }
        }

        HAL_TIM_Base_Stop_IT(&htim2);
    }
}
