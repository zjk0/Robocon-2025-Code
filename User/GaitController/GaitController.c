/**
 * ----------------------------------- Include -----------------------------------
 */
#include "GaitController.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */
// Initialize tort controller
TrotController trot_controller = {
    .trot_state = EndTrot,
    .trot_direction = Forward,
    .trot_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .swing_duty_cycle = 0.5,
    .trot_enable = 0,
    .trot_state_change = 0
};

// Initialize rotate controller
RotateController rotate_controller = {
    .rotate_state = EndRotate,
    .rotate_direction = Left,
    .rotate_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .swing_duty_cycle = 0.5,
    .rotate_enable = 0,
    .rotate_state_change = 0
};

// Initialize jump up controller
JumpController jump_up_controller = {
    .jump_state = EndJump, 
    .jump_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .jump_enable = 0, 
    .jump_state_change = 0
};

// Initialize jump forward controller
JumpController jump_forward_controller = {
    .jump_state = EndJump, 
    .jump_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .jump_enable = 0, 
    .jump_state_change = 0
};

// Initialize turn controller
TurnController turn_controller = {
    .turn_state = EndTurn, 
    .turn_angular_direction = TurnLeft, 
    .turn_linear_direction = LinearForward, 
    .turn_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .turn_enable = 0,
    .turn_state_change = 0,
    .swing_duty_cycle = 0.5
};

usart_data usart_motor_data;

float angle[4][2] = {0};
float Velocity[4][2] = {0};
float Torque[4][2] = {0};

float t = 0;
float last_t = -1;

float J60Motor_StandUpData_CAN1[4] = {0.606880188, -2.1676292411, -0.308265686, 1.98978424};  // lf_out, lf_in, rf_out, rf_in
float J60Motor_StandUpData_CAN2[4] = {2.16770553, -0.684242248, -2.4397697444, 0.523414611};  // rb_out, rb_in, lb_out, lb_in

/**
 * ----------------------------------- Functions -----------------------------------
 */
/**
 * @brief Set the parameters of motors
 * 
 * @param angle: A two-dimensional array to store the positions of motors
 * @param Velocity: A two-dimensional array to store the velocities of motors
 * @param Torque: A two-dimensional array to store the torques of motors
 * @param Kp: The stiffness coefficient of motors
 * @param Kd: The damping coefficient of motors
 * @param motor_mode: The mode of motors when motors are working
 * 
 * @return none
 */
void SetMotor (float (*angle)[2], float (*Velocity)[2], float (*Torque)[2], float Kp, float Kd, enum MotorMode motor_mode) {
    usart_motor_data.real_motor_data[0] = angle[1][0];
    usart_motor_data.real_motor_data[1] = angle[1][1];
    usart_motor_data.real_motor_data[2] = angle[3][0];
    usart_motor_data.real_motor_data[3] = angle[3][1];
    usart_motor_data.real_motor_data[4] = Velocity[1][0];
    usart_motor_data.real_motor_data[5] = Velocity[1][1];
    usart_motor_data.real_motor_data[6] = Velocity[3][0];
    usart_motor_data.real_motor_data[7] = Velocity[3][1];
    usart_motor_data.real_motor_data[8] = Torque[1][0];
    usart_motor_data.real_motor_data[9] = Torque[1][1];
    usart_motor_data.real_motor_data[10] = Torque[3][0];
    usart_motor_data.real_motor_data[11] = Torque[3][1];
    usart_motor_data.real_motor_data[12] = Kp;
    usart_motor_data.real_motor_data[13] = Kd;
    usart_motor_data.real_motor_data[14] = motor_mode * 1.0;

    HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 60, 1000);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

    // Only send the command of lf and rb
    if (motor_mode == PositionMode || motor_mode == PositionTorqueMode) {
        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle[0][1], Velocity[0][1], Torque[0][1], Kp, Kd, motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle[0][0], Velocity[0][0], Torque[0][0], Kp, Kd, motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle[2][1], Velocity[2][1], Torque[2][1], Kp, Kd, motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle[2][0], Velocity[2][0], Torque[2][0], Kp, Kd, motor_mode);
        HAL_Delay(1);
    }
    else {
        RunJ60Motor(&J60Motor_CAN1[0], 0, Velocity[0][1], Torque[0][1], Kp, Kd, motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], 0, Velocity[0][0], Torque[0][0], Kp, Kd, motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], 0, Velocity[2][1], Torque[2][1], Kp, Kd, motor_mode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], 0, Velocity[2][0], Torque[2][0], Kp, Kd, motor_mode);
        HAL_Delay(1);
    }
}

/**
 * @brief The stand command to motors, which helps robot stand
 * 
 * @param none
 * 
 * @return none
 */
void Stand (void) {
    for (int i = 0; i < 12; i++) {
        usart_motor_data.real_motor_data[i] = 0;
    }
    usart_motor_data.real_motor_data[12] = 100;
    usart_motor_data.real_motor_data[13] = 5;
    usart_motor_data.real_motor_data[14] = PositionMode;

    HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 60, 1000);
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

/**
 * @brief Set the control points of bezier curve in trot gait
 * 
 * @param trot_controller: A struct to store trot information
 * @param bezier_height: The max height of bezier curve
 * @param bezier_length: The length of bezier curve, which is also the length in x-axis
 * @param leg: To specify which leg
 * @param trotting_state: To specify which state in trotting
 * 
 * @return none
 */
void SetTrotBezierControlPoints (TrotController* trot_controller, float bezier_height, float bezier_length, int leg, int trotting_state) {
    int symbol = trot_controller->trot_direction;
    float control_points_x_1[4] = {0, 0, bezier_length * symbol, bezier_length * symbol};
    float control_points_x_2[4] = {0, 0, -bezier_length * symbol, -bezier_length * symbol};
    float control_points_x_3[4] = {bezier_length * symbol, bezier_length * symbol, 0, 0};
    float control_points_x_4[4] = {-bezier_length * symbol, -bezier_length * symbol, 0, 0};
    float control_points_x_5[4] = {-bezier_length / 2 * symbol, -bezier_length / 2 * symbol, bezier_length / 2 * symbol, bezier_length / 2 * symbol};
    float control_points_x_6[4] = {bezier_length / 2 * symbol, bezier_length / 2 * symbol, -bezier_length / 2 * symbol, -bezier_length / 2 * symbol};
    float control_points_y_1[4] = {0, 0, 0, 0};
    float control_points_y_2[4] = {0, bezier_height * 0.09 / 0.0675, bezier_height * 0.09 / 0.0675, 0};

    if (trot_controller->trot_state == PreTrot) {
        if (leg == LEFT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[0]), control_points_x_1, control_points_y_2);
        }
        else if (leg == RIGHT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[1]), control_points_x_2, control_points_y_1);
        }
        else if (leg == RIGHT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[2]), control_points_x_1, control_points_y_2);
        }
        else if (leg == LEFT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[3]), control_points_x_2, control_points_y_1);
        }
    }
    else if (trot_controller->trot_state == Trotting) {
        if (trotting_state == TROTTING_LF_RB_SUPPORT_RF_LB_SWING) {
            if (leg == LEFT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[0]), control_points_x_6, control_points_y_1);
            }
            else if (leg == RIGHT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[1]), control_points_x_5, control_points_y_2);
            }
            else if (leg == RIGHT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[2]), control_points_x_6, control_points_y_1);
            }
            else if (leg == LEFT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[3]), control_points_x_5, control_points_y_2);
            }
        }
        else if (trotting_state == TROTTING_LF_RB_SWING_RF_LB_SUPPORT) {
            if (leg == LEFT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[0]), control_points_x_5, control_points_y_2);
            }
            else if (leg == RIGHT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[1]), control_points_x_6, control_points_y_1);
            }
            else if (leg == RIGHT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[2]), control_points_x_5, control_points_y_2);
            }
            else if (leg == LEFT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[3]), control_points_x_6, control_points_y_1);
            }
        }
        else {
            return;
        }
    }
    else if (trot_controller->trot_state == PreEndTrot) {
        if (leg == LEFT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[0]), control_points_x_3, control_points_y_1);
        }
        else if (leg == RIGHT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[1]), control_points_x_4, control_points_y_2);
        }
        else if (leg == RIGHT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[2]), control_points_x_3, control_points_y_1);
        }
        else if (leg == LEFT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(trot_controller->trot_bezier[3]), control_points_x_4, control_points_y_2);
        }
    }
    else {
        return;
    }
}

/**
 * @brief The finite state machine of trotting
 * 
 * @param trot_controller: A struct to store trot information
 * 
 * @return none
 */
void Trot_FSM (TrotController* trot_controller) {
    float bezier_height = 0.03;
    float bezier_length = 0.12;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&trot_controller->trot_bezier[i], 1.0);
    }
    
    float t_real = t / 1000;
    float t_real_2 = 0;
    float fai_swing = 1;
    float fai_support = 1;

    if (trot_controller->trot_state == PreTrot) {
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, LEFT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, RIGHT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, RIGHT_BACK_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, LEFT_BACK_LEG, NOT_TROTTING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(trot_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
        
        // Change state
        if (t >= 1000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = Trotting;
            trot_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (trot_controller->trot_state == Trotting) {
        if(t_real >= 0 && t_real < fai_support) {
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, LEFT_FRONT_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, RIGHT_BACK_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / fai_support;
        }
        else if(t_real >= fai_support && t_real <= 2.0f) {
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, LEFT_FRONT_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, RIGHT_BACK_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0f - fai_support);
        }
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if(t_real >= 0 && t_real < (2.0f - fai_support)) {
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, RIGHT_FRONT_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, LEFT_BACK_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0f - fai_support);
        }
        else if(t_real >= (2.0f - fai_support) && t_real <= 2.0f) {
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, RIGHT_FRONT_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length, LEFT_BACK_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0f) / fai_support;
        }
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
        
        // Change state
        if (t >= 2000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = PreEndTrot;
            trot_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (trot_controller->trot_state == PreEndTrot) {
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, LEFT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, RIGHT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, RIGHT_BACK_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, bezier_height, bezier_length / 2, LEFT_BACK_LEG, NOT_TROTTING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(trot_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }
        
        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        // Change state
        if (t == 1000 && trot_controller->trot_state_change == 1) {
            trot_controller->trot_state = EndTrot;
            trot_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (trot_controller->trot_state == EndTrot) {
        trot_controller->trot_enable = 0;
        Stand();
    }
    else {
        return;
    }
}

/**
 * @brief Set the control points of bezier curve in rotate gait
 * 
 * @param rotate_controller: A struct to store rotate information
 * @param bezier_height: The max height of bezier curve
 * @param bezier_length: The length of bezier curve, which is also the length in x-axis
 * @param leg: To specify which leg
 * @param rotating_state: To specify which state in rotating
 * 
 * @return none
 */
void SetRotateBezierControlPoints (RotateController* rotate_controller, float bezier_height, float bezier_length, int leg, int rotating_state) {
    float symbol = rotate_controller->rotate_direction;
    float control_points_x_1[4] = {0, 0, bezier_length * symbol, bezier_length * symbol};
    float control_points_x_2[4] = {0, 0, -bezier_length * symbol, -bezier_length * symbol};
    float control_points_x_3[4] = {bezier_length * symbol, bezier_length * symbol, 0, 0};
    float control_points_x_4[4] = {-bezier_length * symbol, -bezier_length * symbol, 0, 0};
    float control_points_x_5[4] = {-bezier_length / 2 * symbol, -bezier_length / 2 * symbol, bezier_length / 2 * symbol, bezier_length / 2 * symbol};
    float control_points_x_6[4] = {bezier_length / 2 * symbol, bezier_length / 2 * symbol, -bezier_length / 2 * symbol, -bezier_length / 2 * symbol};
    float control_points_y_1[4] = {0, 0, 0, 0};
    float control_points_y_2[4] = {0, bezier_height * 0.09 / 0.0675, bezier_height * 0.09 / 0.0675, 0};

    if (rotate_controller->rotate_state == PreRotate) {
        if (leg == LEFT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[0]), control_points_x_1, control_points_y_2);
        }
        else if (leg == RIGHT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[1]), control_points_x_1, control_points_y_1);
        }
        else if (leg == RIGHT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[2]), control_points_x_2, control_points_y_2);
        }
        else if (leg == LEFT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[3]), control_points_x_2, control_points_y_1);
        }
    }
    else if (rotate_controller->rotate_state == Rotating) {
        if (rotating_state == ROTATING_LF_RB_SUPPORT_RF_LB_SWING) {
            if (leg == LEFT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[0]), control_points_x_6, control_points_y_1);
            }
            else if (leg == RIGHT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[1]), control_points_x_6, control_points_y_2);
            }
            else if (leg == RIGHT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[2]), control_points_x_5, control_points_y_1);
            }
            else if (leg == LEFT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[3]), control_points_x_5, control_points_y_2);
            }
        }
        else if (rotating_state == ROTATING_LF_RB_SWING_RF_LB_SUPPORT) {
            if (leg == LEFT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[0]), control_points_x_5, control_points_y_2);
            }
            else if (leg == RIGHT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[1]), control_points_x_5, control_points_y_1);
            }
            else if (leg == RIGHT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[2]), control_points_x_6, control_points_y_2);
            }
            else if (leg == LEFT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[3]), control_points_x_6, control_points_y_1);
            }
        }
        else {
            return;
        }
    }
    else if (rotate_controller->rotate_state == PreEndRotate) {
        if (leg == LEFT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[0]), control_points_x_3, control_points_y_1);
        }
        else if (leg == RIGHT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[1]), control_points_x_3, control_points_y_2);
        }
        else if (leg == RIGHT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[2]), control_points_x_4, control_points_y_1);
        }
        else if (leg == LEFT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(rotate_controller->rotate_bezier[3]), control_points_x_4, control_points_y_2);
        }
    }
    else {
        return;
    }
}

/**
 * @brief The finite state machine of rotating
 * 
 * @param rotate_controller: A struct to store rotate information
 * 
 * @return none
 */
void Rotate_FSM (RotateController* rotate_controller) {
    float bezier_height = 0.03;
    float bezier_length = 0.12;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&rotate_controller->rotate_bezier[i], 1.0);
    }

    float t_real = t / 1000;
    float t_real_2 =0;
    float fai_swing = 1;
    float fai_support = 1;

    if (rotate_controller->rotate_state == PreRotate) {
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, LEFT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, RIGHT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, RIGHT_BACK_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, LEFT_BACK_LEG, NOT_ROTATING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        // Change state
        if (t >= 1000 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = Rotating;
            rotate_controller->rotate_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (rotate_controller->rotate_state == Rotating) {
        if(t_real >= 0 && t_real < fai_support) {
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, LEFT_FRONT_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, RIGHT_BACK_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real/fai_support;
        }
        else if(t_real >= fai_support && t_real <= 2.0f) {
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, LEFT_FRONT_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, RIGHT_BACK_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0f - fai_support);
        }
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if(t_real >= 0 && t_real < (2.0f - fai_support)) {
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, RIGHT_FRONT_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, LEFT_BACK_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0f - fai_support);
        }
        else if(t_real >= (2.0f - fai_support) && t_real <= 2.0f) {
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, RIGHT_FRONT_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length, LEFT_BACK_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0f) / fai_support;
        }
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
        
        // Change state
        if (t == 2000 && rotate_controller->rotate_state_change == 1) {
            rotate_controller->rotate_state = PreEndRotate;
            rotate_controller->rotate_state_change = 0;
            t=0;
            last_t=-1;
        }
    }
    else if (rotate_controller->rotate_state == PreEndRotate) {
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, LEFT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, RIGHT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, RIGHT_BACK_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, bezier_height, bezier_length / 2, LEFT_BACK_LEG, NOT_ROTATING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

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
        Stand();
    }
    else {
        return;
    }
}

/**
 * @brief Set the control points of bezier curve in jump gait
 * 
 * @param jump_up_controller: A struct to store jump-up information
 * @param squat_length: The length when robot squats
 * @param jump_length: The change value of length of leg when robot jumps up
 * 
 * @return none
 */
void SetJumpUpBezierControlPoints (JumpController* jump_up_controller, float squat_length, float jump_length) {
    float control_points_x[4] = {0, 0, 0, 0};
    float control_points_y_1[4] = {0, 0, squat_length, squat_length};
    float control_points_y_2[4] = {-jump_length, -jump_length, squat_length, squat_length};
    float control_points_y_3[4] = {squat_length, squat_length, 0, 0};

    if (jump_up_controller->jump_state == Squat) {
        for (int i = 0; i < 4; i++) {
            SetThreeOrderBezierControlPoints(&(jump_up_controller->jump_bezier[i]), control_points_x, control_points_y_1);
        }
    }
    else if (jump_up_controller->jump_state == LegUp) {
        for (int i = 0; i < 4; i++) {
            SetThreeOrderBezierControlPoints(&(jump_up_controller->jump_bezier[i]), control_points_x, control_points_y_2);
        }
    }
    else if (jump_up_controller->jump_state == StandUp) {
        for (int i = 0; i < 4; i++) {
            SetThreeOrderBezierControlPoints(&(jump_up_controller->jump_bezier[i]), control_points_x, control_points_y_3);
        }
    }
    else {
        return;
    }
}

/**
 * @brief The finite state machine of jumping up
 * 
 * @param jump_up_controller: A struct to store jump-up information
 * 
 * @return none
 */
void JumpUp_FSM (JumpController* jump_up_controller) {
    float squat_length = 0.08;
    float jump_length = 0.1;
    float jump_torque = 0;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&jump_up_controller->jump_bezier[i], 1.0);
    }

    float t_real = t / 1000;

    if (jump_up_controller->jump_state == Squat) {
        SetJumpUpBezierControlPoints(jump_up_controller, squat_length, jump_length);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(jump_up_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_up_controller->jump_state_change == 1) {
            jump_up_controller->jump_state = JumpUp;
            jump_up_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
            HAL_Delay(10);
        }
    }
    else if (jump_up_controller->jump_state == JumpUp) {
        for (int i = 0; i < 4; i++) {
            IK_leg(0, robot_height + jump_length, &angle[i][0], &angle[i][1]);
        }

        jump_torque = 10 * (fabs(J60Motor_CAN1[0].ReceiveMotorData.CurrentPosition - J60Motor_StandUpData_CAN1[0]) + fabs(angle[0][1]));
        if (jump_torque > TORQUE_MAX) {
            jump_torque = TORQUE_MAX;
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                Velocity[i][j] = 0;
                if (angle[i][j] > 0) {
                    if ( i == 0 || i == 3) {  // left foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = -jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = jump_torque;
                        }
                    }
                    else {  // right foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = -jump_torque;
                        }
                    }
                }
                else {
                    if (i == 0 || i == 3) {  // left foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = -jump_torque;
                        }
                    }
                    else {  // right foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = -jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = jump_torque;
                        }
                    }
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 400, 5, PositionTorqueMode);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                Velocity[i][j] = 0;
                Torque[i][j] = 0;
            }
        }

        HAL_Delay(200);
        jump_up_controller->jump_state = LegUp;
    }
    else if (jump_up_controller->jump_state == LegUp) {
        SetJumpUpBezierControlPoints(jump_up_controller, squat_length, jump_length);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(jump_up_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_up_controller->jump_state_change == 1) {
            jump_up_controller->jump_state = Land;
            jump_up_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (jump_up_controller->jump_state == Land) {
        SetMotor(angle, Velocity, Torque, 0, 5, KdMode);
        HAL_Delay(2000);

        jump_up_controller->jump_state = StandUp;
    }
    else if (jump_up_controller->jump_state == StandUp) {
        SetJumpUpBezierControlPoints(jump_up_controller, squat_length, jump_length);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(jump_up_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_up_controller->jump_state_change == 1) {
            jump_up_controller->jump_state = EndJump;
            jump_up_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (jump_up_controller->jump_state == EndJump) {
        jump_up_controller->jump_enable = 0;
        Stand();
    }
    else {
        return;
    }
}

void SetJumpForwardBezierControlPoints (JumpController* jump_forward_controller, float squat_length, float jump_length, float tilt_length) {
    float robot_height = 0.2295;
    float tilt_length_2 = tilt_length * ((robot_height + jump_length) / (robot_height - squat_length));
    float control_points_x_1[4] = {0, 0, -tilt_length, -tilt_length};
    float control_points_x_2[4] = {-tilt_length_2, -tilt_length, 0, tilt_length};
    float control_points_x_3[4] = {tilt_length, tilt_length, 0, 0};
    float control_points_y_1[4] = {0, 0, squat_length, squat_length};
    float control_points_y_2[4] = {-jump_length, squat_length, squat_length * 1.1, squat_length};
    float control_points_y_3[4] = {squat_length, squat_length, 0, 0};

    if (jump_forward_controller->jump_state == Squat) {
        for (int i = 0; i < 4; i++) {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_1, control_points_y_1);
        }
    }
    else if (jump_forward_controller->jump_state == LegUp) {
        for (int i = 0; i < 4; i++) {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_2, control_points_y_2);
        }
    }
    else if (jump_forward_controller->jump_state == StandUp) {
        for (int i = 0; i < 4; i++) {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_3, control_points_y_3);
        }
    }
    else {
        return;
    }
}

void JumpForward_FSM (JumpController* jump_forward_controller) {
    float squat_length = 0.08;
    float jump_length = 0.1;
    float tilt_length = 0.03;
    float jump_torque = 0;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&jump_forward_controller->jump_bezier[i], 1.0);
    }

    float t_real = t / 1000;

    if (jump_forward_controller->jump_state == Squat) {
        SetJumpForwardBezierControlPoints(jump_forward_controller, squat_length, jump_length, tilt_length);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
            jump_forward_controller->jump_state = JumpUp;
            jump_forward_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
            HAL_Delay(10);
        }
    }
    else if (jump_forward_controller->jump_state == JumpUp) {
        float tilt_length_2 = tilt_length * ((robot_height + jump_length) / (robot_height - squat_length));
        for (int i = 0; i < 4; i++) {
            IK_leg(-tilt_length_2, robot_height + jump_length, &angle[i][0], &angle[i][1]);
        }

        jump_torque = 10 * (fabs(J60Motor_CAN1[0].ReceiveMotorData.CurrentPosition - J60Motor_StandUpData_CAN1[0]) + fabs(angle[0][1]));
        if (jump_torque > TORQUE_MAX) {
            jump_torque = TORQUE_MAX;
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                Velocity[i][j] = 0;
                if (angle[i][j] > 0) {
                    if ( i == 0 || i == 3) {  // left foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = -jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = jump_torque;
                        }
                    }
                    else {  // right foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = -jump_torque;
                        }
                    }
                }
                else {
                    if (i == 0 || i == 3) {  // left foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = -jump_torque;
                        }
                    }
                    else {  // right foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = -jump_torque;
                        }
                        else {  // in leg
                            Torque[i][j] = jump_torque;
                        }
                    }
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 400, 5, PositionTorqueMode);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                Velocity[i][j] = 0;
                Torque[i][j] = 0;
            }
        }

        HAL_Delay(200);
        jump_forward_controller->jump_state = LegUp;
    }
    else if (jump_forward_controller->jump_state == LegUp) {
        SetJumpForwardBezierControlPoints(jump_forward_controller, squat_length, jump_length, tilt_length);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
            jump_forward_controller->jump_state = Land;
            jump_forward_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }

    }
    else if (jump_forward_controller->jump_state == Land) {
        SetMotor(angle, Velocity, Torque, 0, 5, KdMode);
        HAL_Delay(2000);

        jump_forward_controller->jump_state = StandUp;
    }
    else if (jump_forward_controller->jump_state == StandUp) {
        SetJumpForwardBezierControlPoints(jump_forward_controller, squat_length, jump_length, tilt_length);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
            jump_forward_controller->jump_state = EndJump;
            jump_forward_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (jump_forward_controller->jump_state == EndJump) {
        jump_forward_controller->jump_enable = 0;
        Stand();
    }
    else {
        return;
    }
}

void SetTurnBezierControlPoints (TurnController* turn_controller, float shorter_length, float longer_length, float bezier_height, int leg, int turning_state) {
    float left_length = 0;
    float right_length = 0;
    if (turn_controller->turn_angular_direction == TurnLeft) {
        left_length = shorter_length;
        right_length = longer_length;
    }
    else if (turn_controller->turn_angular_direction == TurnRight) {
        left_length = longer_length;
        right_length = shorter_length;
    }

    float symbol = turn_controller->turn_linear_direction;
    float control_points_x_left_1[4] = {0, 0, left_length * symbol, left_length * symbol};
    float control_points_x_left_2[4] = {0, 0, -left_length * symbol, -left_length * symbol};
    float control_points_x_left_3[4] = {left_length * symbol, left_length * symbol, 0, 0};
    float control_points_x_left_4[4] = {-left_length * symbol, -left_length * symbol, 0, 0};
    float control_points_x_left_5[4] = {-left_length * symbol, -left_length * symbol, left_length * symbol, left_length * symbol};
    float control_points_x_left_6[4] = {left_length * symbol, left_length * symbol, -left_length * symbol, -left_length * symbol};
    float control_points_x_right_1[4] = {0, 0, right_length * symbol, right_length * symbol};
    float control_points_x_right_2[4] = {0, 0, -right_length * symbol, -right_length * symbol};
    float control_points_x_right_3[4] = {right_length * symbol, right_length * symbol, 0, 0};
    float control_points_x_right_4[4] = {-right_length * symbol, -right_length * symbol, 0, 0};
    float control_points_x_right_5[4] = {-right_length * symbol, -right_length * symbol, right_length * symbol, right_length * symbol};
    float control_points_x_right_6[4] = {right_length * symbol, right_length * symbol, -right_length * symbol, -right_length * symbol};
    float control_points_y_1[4] = {0, 0, 0, 0};
    float control_points_y_2[4] = {0, bezier_height * 0.09 / 0.0675, bezier_height * 0.09 / 0.0675, 0};

    if (turn_controller->turn_state == PreTurn) {
        if (leg == LEFT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[0]), control_points_x_left_1, control_points_y_2);
        }
        else if (leg == RIGHT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[1]), control_points_x_right_2, control_points_y_1);
        }
        else if (leg == RIGHT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[2]), control_points_x_right_1, control_points_y_2);
        }
        else if (leg == LEFT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[3]), control_points_x_left_2, control_points_y_1);
        }
    }
    else if (turn_controller->turn_state == Turning) {
        if (turning_state == TURNING_LF_RB_SUPPORT_RF_LB_SWING) {
            if (leg == LEFT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[0]), control_points_x_left_6, control_points_y_1);
            }
            else if (leg == RIGHT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[1]), control_points_x_right_5, control_points_y_2);
            }
            else if (leg == RIGHT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[2]), control_points_x_right_6, control_points_y_1);
            }
            else if (leg == LEFT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[3]), control_points_x_left_5, control_points_y_2);
            }
        }
        else if (turning_state == TURNING_LF_RB_SWING_RF_LB_SUPPORT) {
            if (leg == LEFT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[0]), control_points_x_left_5, control_points_y_2);
            }
            else if (leg == RIGHT_FRONT_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[1]), control_points_x_right_6, control_points_y_1);
            }
            else if (leg == RIGHT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[2]), control_points_x_right_5, control_points_y_2);
            }
            else if (leg == LEFT_BACK_LEG) {
                SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[3]), control_points_x_left_6, control_points_y_1);
            }
        }
        else {
            return;
        }
    }
    else if (turn_controller->turn_state == PreEndTurn) {
        if (leg == LEFT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[0]), control_points_x_left_3, control_points_y_1);
        }
        else if (leg == RIGHT_FRONT_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[1]), control_points_x_right_4, control_points_y_2);
        }
        else if (leg == RIGHT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[2]), control_points_x_right_3, control_points_y_1);
        }
        else if (leg == LEFT_BACK_LEG) {
            SetThreeOrderBezierControlPoints(&(turn_controller->turn_bezier[3]), control_points_x_left_4, control_points_y_2);
        }
    }
    else {
        return;
    }
}

void Turn_FSM (TurnController* turn_controller) {
    float bezier_height = 0.03;
    float shorter_length = 0.06;
    float longer_length = 0.1;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&turn_controller->turn_bezier[i], 1.0);
    }

    float t_real = t / 1000;
    float t_real_2 = 0;
    float fai_swing = 1;
    float fai_support = 1;

    if (turn_controller->turn_state == PreTurn) {
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, LEFT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, RIGHT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, RIGHT_BACK_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, LEFT_BACK_LEG, NOT_TURNING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(turn_controller->turn_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && turn_controller->turn_state_change == 1) {
            turn_controller->turn_state = Turning;
            turn_controller->turn_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (turn_controller->turn_state == Turning) {
        if (t_real >= 0 && t_real < fai_support) {
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, LEFT_FRONT_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, RIGHT_BACK_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / fai_support;
        }
        else if (t_real >= fai_support && t_real <= 2.0) {
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, LEFT_FRONT_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, RIGHT_BACK_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0 - fai_support);
        }
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if (t_real >= 0 && t_real < (2.0 - fai_support)) {
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, RIGHT_FRONT_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, LEFT_BACK_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0 - fai_support);
        }
        else if (t_real >= (2.0 - fai_support) && t_real <= 2.0) {
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, RIGHT_FRONT_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTurnBezierControlPoints(turn_controller, shorter_length, longer_length, bezier_height, LEFT_BACK_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0) / fai_support;
        }
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 2000 && turn_controller->turn_state_change == 1) {
            turn_controller->turn_state = PreEndTurn;
            turn_controller->turn_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (turn_controller->turn_state == PreEndTurn) {
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, LEFT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, RIGHT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, RIGHT_BACK_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_length / 2, longer_length / 2, bezier_height, LEFT_BACK_LEG, NOT_TURNING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(turn_controller->turn_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && turn_controller->turn_state_change == 1) {
            turn_controller->turn_state = EndTurn;
            turn_controller->turn_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (turn_controller->turn_state == EndTurn) {
        turn_controller->turn_enable = 0;
        Stand();
    }
}

void Trot_to_Turn (TrotController* trot_controller, TurnController* turn_controller, 
                   float trot_length, float shorter_length, float longer_length, float bezier_height, float trotting_state) {
    float t_real = t / 1000;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    
    float left_length = 0;
    float right_length = 0;
    if (turn_controller->turn_angular_direction == TurnLeft) {
        left_length = shorter_length;
        right_length = longer_length;
    }
    else if (turn_controller->turn_angular_direction == TurnRight) {
        left_length = longer_length;
        right_length = shorter_length;
    }
    else {
        return;
    }

    float symbol = trot_controller->trot_direction;
    float control_points_x_left_1[4] = {-trot_length * symbol, -trot_length * symbol, left_length * symbol, left_length * symbol};
    float control_points_x_left_2[4] = {trot_length * symbol, trot_length * symbol, -left_length * symbol, -left_length * symbol};
    float control_points_x_right_1[4] = {-trot_length * symbol, -trot_length * symbol, right_length * symbol, right_length * symbol};
    float control_points_x_right_2[4] = {trot_length * symbol, trot_length * symbol, -right_length * symbol, -right_length * symbol};
    float control_points_y_1[4] = {0, 0, 0, 0};
    float control_points_y_2[4] = {0, bezier_height * 0.09 / 0.0675, bezier_height * 0.09 / 0.0675, 0};

    ThreeOrderBezierInformation bezier[4];

    if (trotting_state == TROTTING_LF_RB_SUPPORT_RF_LB_SWING) {
        SetThreeOrderBezierControlPoints(&bezier[0], control_points_x_left_1, control_points_y_2);
        SetThreeOrderBezierControlPoints(&bezier[1], control_points_x_right_2, control_points_y_1);
        SetThreeOrderBezierControlPoints(&bezier[2], control_points_x_right_1, control_points_y_2);
        SetThreeOrderBezierControlPoints(&bezier[3], control_points_x_left_2, control_points_y_1);
    }
    else if (trotting_state == TROTTING_LF_RB_SWING_RF_LB_SUPPORT) {
        SetThreeOrderBezierControlPoints(&bezier[0], control_points_x_left_2, control_points_y_1);
        SetThreeOrderBezierControlPoints(&bezier[1], control_points_x_right_1, control_points_y_2);
        SetThreeOrderBezierControlPoints(&bezier[2], control_points_x_right_2, control_points_y_1);
        SetThreeOrderBezierControlPoints(&bezier[3], control_points_x_left_1, control_points_y_2);
    }

    for (int i = 0; i < 4; i++) {
        ThreeOrderBezierPlan(&bezier[i], t_real, &bezier_x[i], &bezier_y[i]);
        IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
    }

    SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
}

void Turn_to_Trot (TrotController* trot_controller, TurnController* turn_controller, 
                   float trot_length, float shorter_length, float longer_length, float bezier_height, float turning_state) {
    float t_real = t / 1000;
    float robot_height = 0.2295;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb

    float left_length = 0;
    float right_length = 0;
    if (turn_controller->turn_angular_direction == TurnLeft) {
        left_length = shorter_length;
        right_length = longer_length;
    }
    else if (turn_controller->turn_angular_direction == TurnRight) {
        left_length = longer_length;
        right_length = shorter_length;
    }
    else {
        return;
    }

    float symbol = trot_controller->trot_direction;
    float control_points_x_left_1[4] = {-left_length * symbol, -left_length * symbol, trot_length * symbol, trot_length * symbol};
    float control_points_x_left_2[4] = {left_length * symbol, left_length * symbol, -trot_length * symbol, -trot_length * symbol};
    float control_points_x_right_1[4] = {-right_length * symbol, -right_length * symbol, trot_length * symbol, trot_length * symbol};
    float control_points_x_right_2[4] = {right_length * symbol, right_length * symbol, -trot_length * symbol, -trot_length * symbol};
    float control_points_y_1[4] = {0, 0, 0, 0};
    float control_points_y_2[4] = {0, bezier_height * 0.09 / 0.0675, bezier_height * 0.09 / 0.0675, 0};

    ThreeOrderBezierInformation bezier[4];

    if (turning_state == TURNING_LF_RB_SUPPORT_RF_LB_SWING) {
        SetThreeOrderBezierControlPoints(&bezier[0], control_points_x_left_1, control_points_y_2);
        SetThreeOrderBezierControlPoints(&bezier[1], control_points_x_right_2, control_points_y_1);
        SetThreeOrderBezierControlPoints(&bezier[2], control_points_x_right_1, control_points_y_2);
        SetThreeOrderBezierControlPoints(&bezier[3], control_points_x_left_2, control_points_y_1);
    }
    else if (turning_state == TURNING_LF_RB_SWING_RF_LB_SUPPORT) {
        SetThreeOrderBezierControlPoints(&bezier[0], control_points_x_left_2, control_points_y_1);
        SetThreeOrderBezierControlPoints(&bezier[1], control_points_x_right_1, control_points_y_2);
        SetThreeOrderBezierControlPoints(&bezier[2], control_points_x_right_2, control_points_y_1);
        SetThreeOrderBezierControlPoints(&bezier[3], control_points_x_left_1, control_points_y_2);
    }

    for (int i = 0; i < 4; i++) {
        ThreeOrderBezierPlan(&bezier[i], t_real, &bezier_x[i], &bezier_y[i]);
        IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]);
    }

    SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
}
