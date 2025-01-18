/**
 * ----------------------------------- Include -----------------------------------
 */
#include "GaitController.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */
TrotController trot_controller = {EndTrot, Forward, ThreeOrderBezier, 0.5, 0, 0};
RotateController rotate_controller = {EndRotate, Left, ThreeOrderBezier, 0.5, 0, 0};
SuddenSituation sudden_situation = {0, 0};
JumpController jump_controller = {EndJump, 0, 0};

float ReInit_can1[2];
float ReInit_can2[2];

uint8_t reinit_signal[1] = {0x01};

int Debug1 = 0;
int Debug2 = 0;
int Debug3 = 0;
int Debug4 = 0;

/**
 * ----------------------------------- Functions -----------------------------------
 */

void SetMotor (void) {
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
}

/**
 * @brief The finite state machine of trotting
 * 
 * @param trot_controller: The pointer to the struct which store the information of trot-gait
 * 
 * @return none
 */
void Trot_FSM (TrotController* trot_controller) {
    float t_real = t / 1000;
    float t_real_2 = 0;
    //RF¡¢RF¡¢RD¡¢LD
    float start_x[4] = {0};
    float end_x[4] = {0};
    float max_z = 0.03;
    float fai_swing = 1;
    float fai_support = 1;
    if (trot_controller->trot_state == PreTrot) {
        start_x[0] =0;
        start_x[1] =0;
        start_x[2] =0;
        start_x[3] =0;
        end_x[0] = 0.06;
        end_x[1] = -0.06;
        end_x[2] = 0.06;
        end_x[3] = -0.06;

        if(trot_controller->trot_direction==Back){
            end_x[0] = -0.06;
            end_x[1] = 0.06;
            end_x[2] = -0.06;
            end_x[3] = 0.06;
        }
        //Walk_straight_Bezier(&t_real, angle, 0.5, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);
        //Pre_Walk_straight_Bezier
        if(t_real>=0&&t_real<=1)
        {
            //LF_leg_ID1×óÇ°ÍÈ
            Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], fai_swing, start_x[0], 0, end_x[0], 0, max_z, 0);
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], fai_support, start_x[1], 0, end_x[1], 0, 0, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], fai_swing, start_x[2], 0, end_x[2], 0, max_z, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], fai_support, start_x[3], 0, end_x[3], 0, 0, 0);
        }


        // Debug
        Debug1++;
        if (Debug1 == 2000) {
            Debug1 = 0;
        }

        SetMotor();
        
        // Change state
        if (t >= 1000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = Trotting;
            trot_controller->trot_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (trot_controller->trot_state == Trotting) {
        //Walk_straight_Bezier(&t_real, angle, 0.5f, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);

        if(t_real>=0&&t_real<fai_support){
            start_x[0] = 0.06f;
            start_x[2] = 0.06f;
            end_x[0] = -0.06f;
            end_x[2] = -0.06f;
            if(trot_controller->trot_direction==Back){
                start_x[0] = -0.06;
                start_x[2] = -0.06;
                end_x[0] = 0.06f;
                end_x[2] = 0.06f;
            }
            //LF_leg_ID1×óÇ°ÍÈ
            t_real_2=t_real/fai_support;
            Cubic_Bezier(&t_real_2, &angle[0][0], &angle[0][1], fai_support, start_x[0], 0, end_x[0], 0, 0, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[2][0], &angle[2][1], fai_support, start_x[2], 0, end_x[2], 0, 0, 0);
        }
        else if(t_real>=fai_support&&t_real<=2.0f){
            start_x[0] = -0.06f;
            start_x[2] = -0.06f;
            end_x[0] = 0.06f;
            end_x[2] = 0.06f;
            if(trot_controller->trot_direction==Back){
                start_x[0] = 0.06;
                start_x[2] = 0.06;
                end_x[0] = -0.06f;
                end_x[2] = -0.06f;
            }
            t_real_2=(t_real-fai_support)/(2.0f-fai_support);
            //LF_leg_ID1×óÇ°ÍÈ
            Cubic_Bezier(&t_real_2, &angle[0][0], &angle[0][1], fai_swing, start_x[0], 0, end_x[0], 0, max_z, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[2][0], &angle[2][1], fai_swing, start_x[2], 0, end_x[2], 0, max_z, 0);
        }

        if(t_real>=0&&t_real<(2.0f-fai_support)){
            start_x[1] = -0.06f;
            start_x[3] = -0.06f;
            end_x[1] = 0.06f;
            end_x[3] = 0.06f;
            if(trot_controller->trot_direction==Back){
                start_x[1] = 0.06;
                start_x[3] = 0.06;
                end_x[1] = -0.06f;
                end_x[3] = -0.06f;
            }
            t_real_2=t_real/(2.0f-fai_support);
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real_2, &angle[1][0], &angle[1][1], fai_swing, start_x[1], 0, end_x[1], 0, max_z, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[3][0], &angle[3][1], fai_swing, start_x[3], 0, end_x[3], 0, max_z, 0);
        }
        else if(t_real>=(2.0f-fai_support)&&t_real<=2.0f){
            start_x[1] = 0.06f;
            start_x[3] = 0.06f;
            end_x[1] = -0.06f;
            end_x[3] = -0.06f;
            if(trot_controller->trot_direction==Back){
                start_x[1] = -0.06;
                start_x[3] = -0.06;
                end_x[1] = 0.06f;
                end_x[3] = 0.06f;
            }
            t_real_2=(t_real+fai_support-2.0f)/fai_support;
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real_2, &angle[1][0], &angle[1][1], fai_support, start_x[1], 0, end_x[1], 0, 0, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[3][0], &angle[3][1], fai_support, start_x[3], 0, end_x[3], 0, 0, 0);
        }



        // Debug
        Debug2++;
        if (Debug2 == 2000) {
            Debug2 = 0;
        }

        SetMotor();
        
        // Change state
        if (t >= 2000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = PreEndTrot;
            trot_controller->trot_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (trot_controller->trot_state == PreEndTrot) {
        start_x[0] = 0.06;
        start_x[1] = -0.06;
        start_x[2] = 0.06;
        start_x[3] = -0.06;
        end_x[0] = 0;
        end_x[1] = 0;
        end_x[2] = 0;
        end_x[3] = 0;

        if(trot_controller->trot_direction==Back){
            start_x[0] = -0.06;
            start_x[1] = 0.06;
            start_x[2] = -0.06;
            start_x[3] = 0.06;
        }
        // Walk_straight_Bezier(&t_real, angle, 0.5, start_x, 0, end_x, 0, max_z, trot_controller->trot_direction);
        Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], fai_support, start_x[0], 0, end_x[0], 0, 0, 0);
        Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], fai_swing, start_x[1], 0, end_x[1], 0, max_z, 0);
        Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], fai_support, start_x[2], 0, end_x[2], 0, 0, 0);
        Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], fai_swing, start_x[3], 0, end_x[3], 0, max_z, 0);

//         Debug
        Debug3++;
        if (Debug3 == 2000) {
            Debug3 = 0;
        }

        
        SetMotor();

        // Change state
        if (t == 1000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = EndTrot;
            trot_controller->trot_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (trot_controller->trot_state == EndTrot) {
//         Debug
         Debug4++;
         if (Debug4 == 2000) {
             Debug4 = 0;
         }
        trot_controller->trot_enable = 0;

        for (int i = 0; i < 4; i++) {
            usart_motor_data.real_motor_data[i] = 0;
        }

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        // Only send the command of lf and rb
        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 100, 5, PositionMode);
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
    float t_real_2 =0;
    float start_x[4] = {0};
    float end_x[4] = {0};
    float max_z = 0.03;
    float fai_swing = 1;
    float fai_support = 1;
    if (rotate_controller->rotate_state == PreRotate) {
        start_x[0] = 0;
        start_x[1] = 0;
        start_x[2] = 0;
        start_x[3] = 0;
        end_x[0] = 0.04;
        end_x[1] = 0.04;
        end_x[2] = -0.04;
        end_x[3] = -0.04;
        if(rotate_controller->rotate_direction==Left){
            end_x[0] = -0.04;
            end_x[1] = -0.04;
            end_x[2] = 0.04;
            end_x[3] = 0.04;
        }
        
        //Walk_turn_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, rotate_controller->rotate_direction);
        if(t_real >=0 && t_real <= 1)
        {
            //LF_leg_ID1×óÇ°ÍÈ
            Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], fai_swing, start_x[0], 0, end_x[0], 0, max_z, 0);
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], fai_support, start_x[1], 0, end_x[1], 0, 0, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], fai_swing, start_x[2], 0, end_x[2], 0, max_z, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], fai_support, start_x[3], 0, end_x[3], 0, 0, 0);
        }
//         Debug
         Debug1++;
         if (Debug1 == 2000) {
             Debug1 = 0;
         }

        SetMotor();

        // Change state
        if (t >= 1000 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = Rotating;
            rotate_controller->rotate_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (rotate_controller->rotate_state == Rotating) {
        start_x[0] = 0.04;
        start_x[1] = 0.04;
        start_x[2] = -0.04;
        start_x[3] = -0.04;
        end_x[0] = -0.04;
        end_x[1] =-0.04;
        end_x[2] = 0.04;
        end_x[3] = 0.04;
        // fai=0.5;
        
        // Walk_turn_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, rotate_controller->rotate_direction);
        if(t_real>=0&&t_real<fai_support){
            start_x[0] = 0.04f;
            start_x[2] = -0.04f;
            end_x[0] = -0.04f;
            end_x[2] = 0.04f;
            if(rotate_controller->rotate_direction==Left){
                start_x[0] = -0.04;
                start_x[2] = 0.04;
                end_x[0] = 0.04f;
                end_x[2] = -0.04f;
            }
            //LF_leg_ID1×óÇ°ÍÈ
            t_real_2=t_real/fai_support;
            Cubic_Bezier(&t_real_2, &angle[0][0], &angle[0][1], fai_support, start_x[0], 0, end_x[0], 0, 0, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[2][0], &angle[2][1], fai_support, start_x[2], 0, end_x[2], 0, 0, 0);
        }
        else if(t_real>=fai_support&&t_real<=2.0f){
            start_x[0] = -0.04f;
            start_x[2] = 0.04f;
            end_x[0] = 0.04f;
            end_x[2] = -0.04f;
            if(rotate_controller->rotate_direction==Left){
                start_x[0] = 0.04;
                start_x[2] = -0.04;
                end_x[0] = -0.04f;
                end_x[2] = 0.04f;
            }
            t_real_2=(t_real-fai_support)/(2.0f-fai_support);
            //LF_leg_ID1×óÇ°ÍÈ
            Cubic_Bezier(&t_real_2, &angle[0][0], &angle[0][1], fai_swing, start_x[0], 0, end_x[0], 0, max_z, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[2][0], &angle[2][1], fai_swing, start_x[2], 0, end_x[2], 0, max_z, 0);
        }

        if(t_real>=0&&t_real<(2.0f-fai_support)){
            start_x[1] = 0.04f;
            start_x[3] = -0.04f;
            end_x[1] = -0.04f;
            end_x[3] = 0.04f;
            if(rotate_controller->rotate_direction==Left){
                start_x[1] = -0.04;
                start_x[3] = 0.04;
                end_x[1] = 0.04f;
                end_x[3] = -0.04f;
            }
            t_real_2=t_real/(2.0f-fai_support);
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real_2, &angle[1][0], &angle[1][1], fai_swing, start_x[1], 0, end_x[1], 0, max_z, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[3][0], &angle[3][1], fai_swing, start_x[3], 0, end_x[3], 0, max_z, 0);
        }
        else if(t_real>=(2.0f-fai_support)&&t_real<=2.0f){
            start_x[1] = -0.04f;
            start_x[3] = 0.04f;
            end_x[1] = 0.04f;
            end_x[3] = -0.04f;
            if(rotate_controller->rotate_direction==Left){
                start_x[1] = 0.04;
                start_x[3] = -0.04;
                end_x[1] = -0.04f;
                end_x[3] = 0.04f;
            }
            t_real_2=(t_real+fai_support-2.0f)/fai_support;
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real_2, &angle[1][0], &angle[1][1], fai_support, start_x[1], 0, end_x[1], 0, 0, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real_2, &angle[3][0], &angle[3][1], fai_support, start_x[3], 0, end_x[3], 0, 0, 0);
        }
        // Debug
        Debug2++;
        if (Debug2 == 2000) {
            Debug2 = 0;
        }

        SetMotor();
        
        // Change state
        if (t == 2000 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = PreEndRotate;
            rotate_controller->rotate_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (rotate_controller->rotate_state == PreEndRotate) {
        start_x[0] = 0.04;
        start_x[1] = 0.04;
        start_x[2] = -0.04;
        start_x[3] = -0.04;
        end_x[0] = 0;
        end_x[1] = 0;
        end_x[2] = 0;
        end_x[3] = 0;
        if(rotate_controller->rotate_direction==Left){
            start_x[0] = -0.04;
            start_x[1] = -0.04;
            start_x[2] = 0.04f;
            start_x[3] = 0.04f;
        }
        //Walk_turn_Bezier(&t_real, angle, 0.4, start_x, 0, end_x, 0, max_z, rotate_controller->rotate_direction);
        if(t_real>=0&&t_real<=1)
        {
            //LF_leg_ID1×óÇ°ÍÈ
            Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], fai_support, start_x[0], 0, end_x[0], 0, 0, 0);
            //RF_leg_ID2ÓÒÇ°ÍÈ
            Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], fai_swing, start_x[1], 0, end_x[1], 0, max_z, 0);
            //RD_leg_ID3ÓÒºóÍÈ
            Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], fai_support, start_x[2], 0, end_x[2], 0, 0, 0);
            //LD_leg_ID4×óºóÍÈ
            Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], fai_swing, start_x[3], 0, end_x[3], 0, max_z, 0);
        }
        // Debug
        Debug3++;
        if (Debug3 == 2000) {
            Debug3 = 0;
        }

        SetMotor();

        // Change state
        if (t >= 1000 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = EndRotate;
            rotate_controller->rotate_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (rotate_controller->rotate_state == EndRotate) {
        rotate_controller->rotate_enable = 0;

        for (int i = 0; i < 4; i++) {
            usart_motor_data.real_motor_data[i] = 0;
        }

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
    }
    else {
        return;
    }
}

// void ReInit (float t) {
//     Cubic_Bezier(&t, &angle[0][0], &angle[0][1], 1, sudden_situation.re_init_x[0], sudden_situation.re_init_y[0], 0, 0, 0, 0);
//     Cubic_Bezier(&t, &angle[1][0], &angle[1][1], 1, sudden_situation.re_init_x[1], sudden_situation.re_init_y[1], 0, 0, 0, 0);

//     RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], 0, 0, 100, 5, PositionMode);
//     HAL_Delay(1);
//     RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], 0, 0, 100, 5, PositionMode);
//     HAL_Delay(1);
//     RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], 0, 0, 100, 5, PositionMode);
//     HAL_Delay(1);
//     RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], 0, 0, 100, 5, PositionMode);
//     HAL_Delay(1);

// }

void Jump_FSM (JumpController* jump_controller) {
    float squat_length = 0.05;
    float jump_leg_length = squat_length + 0.02;
    float t_real = t / 1000;
    if (jump_controller->jump_state == Squat) {
        Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], 1, 0, 0, 0, squat_length, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], 1, 0, 0, 0, squat_length, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], 1, 0, 0, 0, squat_length, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], 1, 0, 0, 0, squat_length, squat_length, 0);

        SetMotor();

        if (t >= 1000 && jump_controller->jump_state_change == 1) {
            jump_controller->jump_state = JumpUp;
            jump_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (jump_controller->jump_state == JumpUp) {
        float original_position = 0.2457;
        for (int i = 0; i < 4; i++) {
            IK_leg(0, original_position + jump_leg_length, &angle[i][0], &angle[i][1]);
        }

        SetMotor();

        HAL_Delay(10);

        jump_controller->jump_state = LegUp;
    }
    else if (jump_controller->jump_state == LegUp) {
        Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], 1, 0, -jump_leg_length, 0, squat_length, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], 1, 0, -jump_leg_length, 0, squat_length, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], 1, 0, -jump_leg_length, 0, squat_length, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], 1, 0, -jump_leg_length, 0, squat_length, squat_length, 0);

        SetMotor();

        if (t >= 1000 && jump_controller->jump_state_change == 1) {
            jump_controller->jump_state = Land;
            jump_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }

    }
    else if (jump_controller->jump_state == Land) {
        Cubic_Bezier(&t_real, &angle[0][0], &angle[0][1], 1, 0, squat_length, 0, 0, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[1][0], &angle[1][1], 1, 0, squat_length, 0, 0, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[2][0], &angle[2][1], 1, 0, squat_length, 0, 0, squat_length, 0);
        Cubic_Bezier(&t_real, &angle[3][0], &angle[3][1], 1, 0, squat_length, 0, 0, squat_length, 0);

        SetMotor();

        if (t >= 1000 && jump_controller->jump_state_change == 1) {
            jump_controller->jump_state = EndJump;
            jump_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }

    }
    else if (jump_controller->jump_state == EndJump) {
        jump_controller->jump_enable = 0;

        for (int i = 0; i < 4; i++) {
            usart_motor_data.real_motor_data[i] = 0;
        }

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 100, 5, PositionMode);
        HAL_Delay(1);
    }
}