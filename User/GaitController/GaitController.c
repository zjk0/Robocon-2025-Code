/**
 * ----------------------------------- Include -----------------------------------
 */
#include "GaitController.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */
TrotController trot_controller = {0};

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Trot_FSM (TrotController* trot_controller) {
    if (trot_controller->trot_state == PreTrot) {
        trot_controller->trot_state = Trotting;
    }
    else if (trot_controller->trot_state == Trotting) {
        if (trot_controller->trot_direction == Forward) {

        }
        else if (trot_controller->trot_direction == Back) {

        }
        else {
            return;
        }
    }
    else if (trot_controller->trot_state == PreEndTrot) {
        trot_controller->trot_state = EndTrot;
    }
    else if (trot_controller->trot_state == EndTrot) {
        for (int i = 0; i < 4; i++) {
            usart_motor_data.real_motor_data[i] = 0;
        }

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        Stop();
    }
    else {
        return;
    }
}