/**
 * ----------------------------------- Include -----------------------------------
 */
#include "GaitController.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */
TrotController trot_controller = {EndTrot, Forward, ThreeOrderBezier, 0.4, 0, 0};
RotateController rotate_controller = {EndRotate, Left, ThreeOrderBezier, 0.4, 0, 0};

int Debug1 = 0;
int Debug2 = 0;
int Debug3 = 0;
int Debug4 = 0;

/**
 * ----------------------------------- Functions -----------------------------------
 */
/**
 * @brief The finite state machine of trotting
 * 
 * @param trot_controller: The pointer to the struct which store the information of trot-gait
 * 
 * @return none
 */
void Trot_FSM (TrotController* trot_controller) {
    float t_real = t / 1000;
    float start_x = 0;
    float end_x = 0;
    float max_z = 0.03;
    if (trot_controller->trot_state == PreTrot) {
        start_x = 0;
        end_x = 0.06;
        
        Walk_straight_Bezier(&t_real, angle, 0.5, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

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

        if (t <= 800) {
            // Only send the command of lf and rb
            RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], 0, 0, 100, 5, PositionMode);
            HAL_Delay(1);
            RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], 0, 0, 100, 5, PositionMode);
            HAL_Delay(1);
            RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], 0, 0, 100, 5, PositionMode);
            HAL_Delay(1);
            RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], 0, 0, 100, 5, PositionMode);
            HAL_Delay(1);
        }

        // Change state
        if (t == 1000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = Trotting;
            trot_controller->trot_state_change = 0;
        }
    }
    else if (trot_controller->trot_state == Trotting) {
        start_x = -0.06;
        end_x = 0.06;
        
        Walk_straight_Bezier(&t_real, angle, 0.5, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

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
        if (t == 2000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = PreEndTrot;
            trot_controller->trot_state_change = 0;
        }
    }
    else if (trot_controller->trot_state == PreEndTrot) {
        start_x = -0.06;
        end_x = 0;

        Walk_straight_Bezier(&t_real, angle, 0.5, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

        // Debug
        // Debug3++;
        // if (Debug3 == 2000) {
        //     Debug3 = 0;
        // }

        if (t <= 800) {
            usart_motor_data.real_motor_data[0] = angle[1][0];
            usart_motor_data.real_motor_data[1] = angle[1][1];
            usart_motor_data.real_motor_data[2] = angle[3][0];
            usart_motor_data.real_motor_data[3] = angle[3][1];

            HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
            while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
        }
    
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
        if (t == 1000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = EndTrot;
            trot_controller->trot_state_change = 0;
        }
    }
    else if (trot_controller->trot_state == EndTrot) {
        // Debug
        // Debug4++;
        // if (Debug4 == 2000) {
        //     Debug4 = 0;
        // }
        trot_controller->trot_enable = 0;

        for (int i = 0; i < 4; i++) {
            usart_motor_data.real_motor_data[i] = 0;
        }

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        // Only send the command of lf and rb
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

/**
 * @brief The finite state machine of rotating
 * 
 * @param rotate_controller: The pointer to the struct which store the information of rotate-gait
 * 
 * @return none
 */
void Rotate_FSM (RotateController* rotate_controller) {
    float t_real = t / 1000;
    float start_x = 0;
    float end_x = 0;
    float max_z = 0.03;
    if (rotate_controller->rotate_state == PreRotate) {
        start_x = 0;
        end_x = 0.04;
        
        Walk_turn_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, rotate_controller->rotate_direction);

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
        if (t == 800 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = Rotating;
            rotate_controller->rotate_state_change = 0;
        }
    }
    else if (rotate_controller->rotate_state == Rotating) {
        start_x = -0.04;
        end_x = 0.04;
        
        Walk_turn_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, rotate_controller->rotate_direction);

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
        if (t == 2000 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = PreEndRotate;
            rotate_controller->rotate_state_change = 0;
        }
    }
    else if (rotate_controller->rotate_state == PreEndRotate) {
        end_x = 0;
        start_x = -0.04;

        Walk_turn_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, rotate_controller->rotate_direction);

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
        if (t == 800 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = EndRotate;
            rotate_controller->rotate_state_change = 0;
        }
    }
    else if (rotate_controller->rotate_state == EndRotate) {
        rotate_controller->rotate_enable = 0;

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