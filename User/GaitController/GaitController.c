/**
 * ----------------------------------- Include -----------------------------------
 */
#include "GaitController.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */
TrotController trot_controller = {EndTrot, Forward, ThreeOrderBezier};
uint8_t TrotStateChange = 0;
uint8_t rotate_direction = 0;
// int Debug1 = 0;
// int Debug2 = 0;
// int Debug3 = 0;
// int Debug4 = 0;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Trot_FSM (TrotController* trot_controller) {
    float t_real = t / 1000;
    float start_x = 0;
    float end_x = 0;
    float max_z = 0.03;
    if (trot_controller->trot_state == PreTrot) {
        start_x = 0;
        if (trot_controller->trot_direction == Forward) {
            end_x = 0.04;
        }
        else if (trot_controller->trot_direction == Back) {
            end_x = -0.04;
        }
        else {
            return;
        }
        Walk_straight_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

        // Debug
        // Debug1++;
        // if (Debug1 == 2000) {
        //     Debug1 = 0;
        // }

        usart_motor_data.real_motor_data[0] = angle[1][0];
        usart_motor_data.real_motor_data[1] = angle[1][1];
        usart_motor_data.real_motor_data[2] = angle[3][0];
        usart_motor_data.real_motor_data[3] = angle[3][1];

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        // Only send the command of lf and rb
        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);

        // Change state
        if (t == 2000 && TrotStateChange == 1) {
            trot_controller->trot_state = Trotting;
            TrotStateChange = 0;
        }
    }
    else if (trot_controller->trot_state == Trotting) {
        if (trot_controller->trot_direction == Forward) {
            start_x = -0.04;
            end_x = 0.04;
        }
        else if (trot_controller->trot_direction == Back) {
            start_x = 0.04;
            end_x = -0.04;
        }
        else {
            return;
        }
        
        Walk_straight_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

        // Debug
        // Debug2++;
        // if (Debug2 == 2000) {
        //     Debug2 = 0;
        // }

        usart_motor_data.real_motor_data[0] = angle[1][0];
        usart_motor_data.real_motor_data[1] = angle[1][1];
        usart_motor_data.real_motor_data[2] = angle[3][0];
        usart_motor_data.real_motor_data[3] = angle[3][1];

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        // Only send the command of lf and rb
        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        
        // Change state
        if (t == 2000 && TrotStateChange == 1) {
            trot_controller->trot_state = PreEndTrot;
            TrotStateChange = 0;
        }
    }
    else if (trot_controller->trot_state == PreEndTrot) {
        end_x = 0;
        if (trot_controller->trot_direction == Forward) {
            start_x = -0.04;
        }
        else if (trot_controller->trot_direction == Back) {
            start_x = 0.04;
        }
        else {
            return;
        }
        Walk_straight_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

        // Debug
        // Debug3++;
        // if (Debug3 == 2000) {
        //     Debug3 = 0;
        // }

        usart_motor_data.real_motor_data[0] = angle[1][0];
        usart_motor_data.real_motor_data[1] = angle[1][1];
        usart_motor_data.real_motor_data[2] = angle[3][0];
        usart_motor_data.real_motor_data[3] = angle[3][1];
    
        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
    
        // Only send the command of lf and rb
        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);

        // Change state
        if (t == 2000 && TrotStateChange == 1) {
            trot_controller->trot_state = EndTrot;
            TrotStateChange = 0;
        }
    }
    else if (trot_controller->trot_state == EndTrot) {
        // Debug
        // Debug4++;
        // if (Debug4 == 2000) {
        //     Debug4 = 0;
        // }

        for (int i = 0; i < 4; i++) {
            usart_motor_data.real_motor_data[i] = 0;
        }

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 20, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 20, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 20, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 20, 5, PositionMode);
        HAL_Delay(1);
    }
    else {
        return;
    }
}