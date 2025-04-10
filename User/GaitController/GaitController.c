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

TrotController walk_slope_controller = {
    .trot_state = EndTrot,
    .trot_direction = Forward,
    .trot_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .trot_enable = 0, 
    .trot_state_change = 0,
    .swing_duty_cycle = 0.5
};

TrotController walk_LR_slope_controller = {
    .trot_state = EndTrot,
    .trot_direction = Forward,
    .trot_bezier = {  // lf, rf, rb, lb
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}, 
        {{0, 0, 0, 0}, {0, 0, 0, 0}, 1}
    },
    .trot_enable = 0, 
    .trot_state_change = 0,
    .swing_duty_cycle = 0.5
};

spi_data spi_motor_data;

float angle[4][2] = {0};
float Velocity[4][2] = {0};
float Torque[4][2] = {0};

float t = 0;
float last_t = -1;

float robot_height = 0.2069;
int isSlope = 0;

float tan_LR_slope_theta = 0.2679;

float J60Motor_StandUpData_CAN1[4] = {0.61, -2.2102, -0.308265686, 1.98978424};  // lf_out, lf_in, rf_out, rf_in
float J60Motor_StandUpData_CAN2[4] = {2.23820, -0.61748, -2.4397697444, 0.523414611};  // rb_out, rb_in, lb_out, lb_in

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
    spi_motor_data.real_motor_data[0] = angle[1][0];
    spi_motor_data.real_motor_data[1] = angle[1][1];
    spi_motor_data.real_motor_data[2] = angle[3][0];
    spi_motor_data.real_motor_data[3] = angle[3][1];
    spi_motor_data.real_motor_data[4] = Velocity[1][0];
    spi_motor_data.real_motor_data[5] = Velocity[1][1];
    spi_motor_data.real_motor_data[6] = Velocity[3][0];
    spi_motor_data.real_motor_data[7] = Velocity[3][1];
    spi_motor_data.real_motor_data[8] = Torque[1][0];
    spi_motor_data.real_motor_data[9] = Torque[1][1];
    spi_motor_data.real_motor_data[10] = Torque[3][0];
    spi_motor_data.real_motor_data[11] = Torque[3][1];
    spi_motor_data.real_motor_data[12] = Kp;
    spi_motor_data.real_motor_data[13] = Kd;
    spi_motor_data.real_motor_data[14] = motor_mode * 1.0;

    HAL_SPI_Transmit(&hspi4, spi_motor_data.send_motor_data, 60, 1000);
    
    //while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);
    while (__HAL_SPI_GET_FLAG(&hspi4, SPI_FLAG_BSY) == SET);

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
    for (int i = 0; i < 4; i++) {
        if (IK_leg(0, robot_height, &angle[i][0], &angle[i][1]) != NO_NAN) {
            return;
        }
    }
    spi_motor_data.real_motor_data[0] = angle[1][0];
    spi_motor_data.real_motor_data[1] = angle[1][1];
    spi_motor_data.real_motor_data[2] = angle[3][0];
    spi_motor_data.real_motor_data[3] = angle[3][1];
    for (int i = 4; i < 12; i++) 
    {
        spi_motor_data.real_motor_data[i] = 0;
    }
    spi_motor_data.real_motor_data[12] = 100;
    spi_motor_data.real_motor_data[13] = 5;
    spi_motor_data.real_motor_data[14] = PositionMode;

    HAL_SPI_Transmit(&hspi4, spi_motor_data.send_motor_data, 60, 1000);
    while (__HAL_SPI_GET_FLAG(&hspi4, SPI_FLAG_BSY) == SET);

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
void Trot_FSM (TrotController* trot_controller, float gait_height, float gait_length, float robot_height) {

    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&trot_controller->trot_bezier[i], 1.0);
    }
    
    float t_real = t / 1000;
    float t_real_2 = 0;
    float fai_swing = 1;
    float fai_support = 1;

    if (trot_controller->trot_state == PreTrot) 
    {
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, LEFT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, RIGHT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, RIGHT_BACK_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, LEFT_BACK_LEG, NOT_TROTTING);

        for (int i = 0; i < 4; i++) 
        {
            ThreeOrderBezierPlan(&(trot_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
            {
                trot_controller->trot_state = EndTrot;
                t = 0;
                last_t = -1;
                return;
            }
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
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, LEFT_FRONT_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, RIGHT_BACK_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / fai_support;
        }
        else if(t_real >= fai_support && t_real <= 2.0f) {
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, LEFT_FRONT_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, RIGHT_BACK_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0f - fai_support);
        }
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if(t_real >= 0 && t_real < (2.0f - fai_support)) {
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, RIGHT_FRONT_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, LEFT_BACK_LEG, TROTTING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0f - fai_support);
        }
        else if(t_real >= (2.0f - fai_support) && t_real <= 2.0f) {
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, RIGHT_FRONT_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTrotBezierControlPoints(trot_controller, gait_height, gait_length, LEFT_BACK_LEG, TROTTING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0f) / fai_support;
        }
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(trot_controller->trot_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                trot_controller->trot_state = EndTrot;
                t = 0;
                last_t = -1;
                return;
            }
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
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, LEFT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, RIGHT_FRONT_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, RIGHT_BACK_LEG, NOT_TROTTING);
        SetTrotBezierControlPoints(trot_controller, gait_height, gait_length / 2, LEFT_BACK_LEG, NOT_TROTTING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(trot_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                trot_controller->trot_state = EndTrot;
                t = 0;
                last_t = -1;
                return;
            }
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
void Rotate_FSM (RotateController* rotate_controller, float gait_height, float gait_length, float robot_height) {

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
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, LEFT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, RIGHT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, RIGHT_BACK_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, LEFT_BACK_LEG, NOT_ROTATING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                rotate_controller->rotate_state = EndRotate;
                t = 0;
                last_t = -1;
                return;
            }
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
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, LEFT_FRONT_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, RIGHT_BACK_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real/fai_support;
        }
        else if(t_real >= fai_support && t_real <= 2.0f) {
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, LEFT_FRONT_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, RIGHT_BACK_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0f - fai_support);
        }
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if(t_real >= 0 && t_real < (2.0f - fai_support)) {
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, RIGHT_FRONT_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, LEFT_BACK_LEG, ROTATING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0f - fai_support);
        }
        else if(t_real >= (2.0f - fai_support) && t_real <= 2.0f) {
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, RIGHT_FRONT_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length, LEFT_BACK_LEG, ROTATING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0f) / fai_support;
        }
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                rotate_controller->rotate_state = EndRotate;
                t = 0;
                last_t = -1;
                return;
            }
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
         (rotate_controller, gait_height, gait_length / 2, LEFT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, RIGHT_FRONT_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, RIGHT_BACK_LEG, NOT_ROTATING);
        SetRotateBezierControlPoints(rotate_controller, gait_height, gait_length / 2, LEFT_BACK_LEG, NOT_ROTATING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(rotate_controller->rotate_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                rotate_controller->rotate_state = EndRotate;
                t = 0;
                last_t = -1;
                return;
            }
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
    float squat_length = 0.07;
    float jump_length = 0.16;
    float jump_torque = 0;
    float robot_height = 0.2069;
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
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                jump_up_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
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
            if (IK_leg(0, robot_height + jump_length, &angle[i][0], &angle[i][1]) != NO_NAN) {
                jump_up_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
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

//        SetMotor(angle, Velocity, Torque, 200, 5, PositionTorqueMode);
SetMotor(angle, Velocity, Torque, 200, 5, PositionMode);
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
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                jump_up_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
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
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                jump_up_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
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

void SetJumpForwardBezierControlPoints0(JumpController* jump_forward_controller, float squat_length, float jump_length, float tilt_length) {
    float robot_height = 0.2069;
    float tilt_length_2 = tilt_length * ((robot_height + jump_length) / (robot_height - squat_length));
    float control_points_x_0[4] = {0, 0, tilt_length, tilt_length};
    float control_points_x_1[4] = {tilt_length, tilt_length, -tilt_length, -tilt_length};
    float control_points_x_2[4] = {-tilt_length_2, -tilt_length_2, tilt_length, tilt_length};
    float control_points_x_3[4] = {tilt_length, tilt_length, 0, 0};
    float control_points_y_0[4] = {0, 0, squat_length, squat_length};
    float control_points_y_1[4] = {squat_length, squat_length, squat_length, squat_length};
    float control_points_y_2[4] = {-jump_length, squat_length, squat_length, squat_length};
    float control_points_y_3[4] = {squat_length, squat_length, 0, 0};//三阶贝赛尔曲线，需要四个点
 
    if (jump_forward_controller->jump_state == Recline) {
        for (int i = 0; i < 4; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_0, control_points_y_0);
        }
    }
    else if (jump_forward_controller->jump_state == Squat) {
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

//void JumpForward_FSM (JumpController* jump_forward_controller) {
//    float squat_length = 0.07;
//    float jump_length = 0.07;
//    float tilt_length = 0.07;
//    float jump_torque = 0;
//    float robot_height = 0.2295;
//    float bezier_x[4] = {0};  // lf, rf, rb, lb
//    float bezier_y[4] = {0};  // lf, rf, rb, lb
//    for (int i = 0; i < 4; i++) {
//        SetThreeOrderBezierPeriod(&jump_forward_controller->jump_bezier[i], 1.0);
//    }
//
//    float t_real = t / 1000;
//
//    if (jump_forward_controller->jump_state == Squat)//蹲下
//    {
//        SetJumpForwardBezierControlPoints(jump_forward_controller, squat_length, jump_length, tilt_length);
//
//        for (int i = 0; i < 4; i++) {
//            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
//            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
//            {
//                jump_forward_controller->jump_state = EndJump;
//                t = 0;
//                last_t = -1;
//                return;
//            }
//        }
//
//        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
//
//        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
//            jump_forward_controller->jump_state = JumpUp;
//            jump_forward_controller->jump_state_change = 0;
//            t = 0;
//            last_t = -1;
//            HAL_Delay(10);
//        }
//    }
//    else if (jump_forward_controller->jump_state == JumpUp) //起跳
//    {
//        float tilt_length_2 = tilt_length * ((robot_height + jump_length) / (robot_height - squat_length));
//        for (int i = 0; i < 4; i++) {
//            if (IK_leg(-tilt_length_2, robot_height + jump_length, &angle[i][0], &angle[i][1]) != NO_NAN) {
//                jump_forward_controller->jump_state = EndJump;
//                t = 0;
//                last_t = -1;
//                return;
//            }
//        }
//
//        jump_torque = 10 * (fabs(J60Motor_CAN1[0].ReceiveMotorData.CurrentPosition - J60Motor_StandUpData_CAN1[0]) + fabs(angle[0][1]));
//        if (jump_torque > TORQUE_MAX) {
//            jump_torque = TORQUE_MAX;
//        }
//
//        for (int i = 0; i < 4; i++) {
//            for (int j = 0; j < 2; j++) {
//                Velocity[i][j] = 0;
//                if (angle[i][j] > 0) {
//                    if ( i == 0 || i == 3) {  // left foot
//                        if (j == 1) {  // out leg
//                            Torque[i][j] = -jump_torque;
//                        }
//                        else {  // in leg
//                            Torque[i][j] = jump_torque;
//                        }
//                    }
//                    else {  // right foot
//                        if (j == 1) {  // out leg
//                            Torque[i][j] = jump_torque;
//                        }
//                        else {  // in leg
//                            Torque[i][j] = -jump_torque;
//                        }
//                    }
//                }
//                else {
//                    if (i == 0 || i == 3) {  // left foot
//                        if (j == 1) {  // out leg
//                            Torque[i][j] = jump_torque;
//                        }
//                        else {  // in leg
//                            Torque[i][j] = -jump_torque;
//                        }
//                    }
//                    else {  // right foot
//                        if (j == 1) {  // out leg
//                            Torque[i][j] = -jump_torque;
//                        }
//                        else {  // in leg
//                            Torque[i][j] = jump_torque;
//                        }
//                    }
//                }
//            }
//        }
//
//        SetMotor(angle, Velocity, Torque, 400, 5, PositionTorqueMode);
//
//        for (int i = 0; i < 4; i++) {
//            for (int j = 0; j < 2; j++) {
//                Velocity[i][j] = 0;
//                Torque[i][j] = 0;
//            }
//        }
//
//        HAL_Delay(200);
//        jump_forward_controller->jump_state = LegUp;
//    }
//    else if (jump_forward_controller->jump_state == LegUp) 
//    {
//        SetJumpForwardBezierControlPoints(jump_forward_controller, squat_length, jump_length, tilt_length);
//
//        for (int i = 0; i < 4; i++) {
//            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
//            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
//                jump_forward_controller->jump_state = EndJump;
//                t = 0;
//                last_t = -1;
//                return;
//            }
//        }
//
//        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
//
//        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
//            jump_forward_controller->jump_state = Land;
//            jump_forward_controller->jump_state_change = 0;
//            t = 0;
//            last_t = -1;
//        }
//
//    }
//    else if (jump_forward_controller->jump_state == Land) {
//        SetMotor(angle, Velocity, Torque, 0, 5, KdMode);
//        HAL_Delay(2000);
//
//        jump_forward_controller->jump_state = StandUp;
//    }
//    else if (jump_forward_controller->jump_state == StandUp) {
//        SetJumpForwardBezierControlPoints(jump_forward_controller, squat_length, jump_length, tilt_length);
//
//        for (int i = 0; i < 4; i++) {
//            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
//            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
//                jump_forward_controller->jump_state = EndJump;
//                t = 0;
//                last_t = -1;
//                return;
//            }
//        }
//
//        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
//
//        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
//            jump_forward_controller->jump_state = EndJump;
//            jump_forward_controller->jump_state_change = 0;
//            t = 0;
//            last_t = -1;
//        }
//    }
//    else if (jump_forward_controller->jump_state == EndJump) {
//        jump_forward_controller->jump_enable = 0;
//        Stand();
//    }
//    else {
//        return;
//    }
//}

void SetJumpForwardBezierControlPoints1 (JumpController* jump_forward_controller, float squat_length0, float jump_length0, float tilt_length0, float squat_length1, float jump_length1, float tilt_length1) //与0的区别是这个的前后腿分别控制
{
    float robot_height = 0.2069;
    float tilt_length_2_0 = tilt_length0 * ((robot_height + jump_length0) / (robot_height - squat_length0));
    float control_points_x_0_0[4] = {0, 0, -tilt_length0, -tilt_length0};//此时不用这个
    float control_points_x_1_0[4] = {0, 0, -tilt_length0, -tilt_length0};
    float control_points_x_2_0[4] = {-tilt_length_2_0, -tilt_length0, 0, tilt_length0};
    float control_points_x_3_0[4] = {tilt_length0, tilt_length0, 0, 0};
    float control_points_y_0_0[4] = {0, 0, squat_length0, squat_length0};//这个也不用
    float control_points_y_1_0[4] = {0, 0, squat_length0, squat_length0};
    float control_points_y_2_0[4] = {-jump_length0, squat_length0, squat_length0, squat_length0};
    float control_points_y_3_0[4] = {squat_length0, squat_length0, 0, 0};
    float tilt_length_2_1 = tilt_length1 * ((robot_height + jump_length1) / (robot_height - squat_length1));
    float control_points_x_0_1[4] = {0, 0, -tilt_length1, -tilt_length1};//这个也不用
    float control_points_x_1_1[4] = {0, 0, -tilt_length1, -tilt_length1};
    float control_points_x_2_1[4] = {-tilt_length_2_1, -tilt_length1, 0, tilt_length1};
    float control_points_x_3_1[4] = {tilt_length1, tilt_length1, 0, 0};
    float control_points_y_0_1[4] = {0, 0, 0 ,0};//这个也不用
    float control_points_y_1_1[4] = {0, 0, squat_length1, squat_length1};
    float control_points_y_2_1[4] = {-jump_length1, squat_length1, squat_length1, squat_length1};
    float control_points_y_3_1[4] = {squat_length1, squat_length1, 0, 0};

    if (jump_forward_controller->jump_state == Recline) 
    {
        for (int i = 0; i < 2; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_0_0, control_points_y_0_0);
        }
        for (int i = 2; i < 4; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_0_1, control_points_y_0_1);
        }
    }
    else if (jump_forward_controller->jump_state == Squat) 
    {
        for (int i = 0; i < 2; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_1_0, control_points_y_1_0);
        }
        for (int i = 2; i < 4; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_1_1, control_points_y_1_1);
        }
    }
    else if (jump_forward_controller->jump_state == LegUp) 
    {
        for (int i = 0; i < 2; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_2_0, control_points_y_2_0);
        }
        for (int i = 2; i < 4; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_2_1, control_points_y_2_1);
        }
    }
    else if (jump_forward_controller->jump_state == StandUp) 
    {
        for (int i = 0; i < 2; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_3_0, control_points_y_3_0);
        }
        for (int i = 2; i < 4; i++) 
        {
            SetThreeOrderBezierControlPoints(&(jump_forward_controller->jump_bezier[i]), control_points_x_3_1, control_points_y_3_1);
        }//只是把这些东西放到另一个地方
    }
    else {
        return;
    }
}

void JumpForward_FSM (JumpController* jump_forward_controller) 
{
    float recline_squat_length = 0.09;
    float recline_jump_length = 0.06;
    float recline_tilt_length = 0.09;//用于计算后仰的位置
  
    float init_squat_length = 0.07;
    float init_jump_length = 0.07;
    float init_tilt_length = 0.07;//用于计算落地后起立的位置
    
    float squat_length0 = 0.08;
    float jump_length0 = 0.08;
    float tilt_length0 = 0.09; //用于计算前腿的位置，若四条腿的步态一直时，只用这个
    
    float squat_length1 = 0.06;
    float jump_length1 = 0.09;
    float tilt_length1 = 0.09;//用于计算后腿的位置
    float jump_torque = 0;
    float robot_height = 0.2069
      
      
      ;
    float bezier_x[4] = {0};  // lf, rf, rb, lb
    float bezier_y[4] = {0};  // lf, rf, rb, lb
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&jump_forward_controller->jump_bezier[i], 1.0);
    }

    float t_real = t / 1000;

//     if (jump_forward_controller->jump_state == Recline)//后仰
//    {
//        SetJumpForwardBezierControlPoints0(jump_forward_controller, recline_squat_length, recline_jump_length, recline_tilt_length);
//
//        for (int i = 0; i < 4; i++) 
//        {
//            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);//点放入贝塞尔曲线的函数
//            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) //真正的逆解函数，把逆解的函数放到angle里面
//            {
//                jump_forward_controller->jump_state = EndJump;
//                t = 0;
//                last_t = -1;
//                return;
//            }
//        }
              
//        SetMotor(angle, Velocity, Torque, 25, 5, PositionMode);
//
//        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) {
//            jump_forward_controller->jump_state = Squat;
//            jump_forward_controller->jump_state_change = 0;
//            t = 0;
//            last_t = -1;
//            HAL_Delay(10);
//        }
//    }
    
     if (jump_forward_controller->jump_state == Squat)//蹲下
    {
        SetJumpForwardBezierControlPoints1(jump_forward_controller, squat_length0, jump_length0, tilt_length0, squat_length1, jump_length1, tilt_length1);

        for (int i = 0; i < 4; i++) 
        {
            LinePlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);//点放入直线规划的函数
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) //真正的逆解函数，把逆解的函数放到angle里面
            {
                jump_forward_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
        }
           
                     
        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) 
        {
            jump_forward_controller->jump_state = JumpUp;
            jump_forward_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
            HAL_Delay(10);
        }
    }
    
    
    else if (jump_forward_controller->jump_state == JumpUp) //起跳
    {
        float tilt_length_2_0 =  tilt_length0 * ((robot_height + jump_length0) / (robot_height - squat_length0));
        float tilt_length_2_1 =  tilt_length1 * ((robot_height + jump_length1) / (robot_height - squat_length1));
        for (int i = 0; i < 2; i++) 
        {
            if (IK_leg(-tilt_length_2_0, robot_height + jump_length0, &angle[i][0], &angle[i][1]) != NO_NAN) {
                jump_forward_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
        }
        
        for (int i = 2; i <4; i++) 
        {
            if (IK_leg(-tilt_length_2_1, robot_height + jump_length1, &angle[i][0], &angle[i][1]) != NO_NAN) {
                jump_forward_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
        }

        jump_torque = 10 * (fabs(J60Motor_CAN1[0].ReceiveMotorData.CurrentPosition - J60Motor_StandUpData_CAN1[0]) + fabs(angle[0][1]));
        if (jump_torque > TORQUE_MAX) 
        {
            jump_torque = TORQUE_MAX;
        }

        for (int i = 0; i < 4; i++) 
        {
            for (int j = 0; j < 2; j++) 
            {
                Velocity[i][j] = 0;
                if (angle[i][j] > 0) 
                {
                    if ( i == 0 || i == 3) 
                    {  // left foot
                        if (j == 1) 
                        {  // out leg
                            Torque[i][j] = -jump_torque;
                        }
                        else 
                        {  // in leg
                            Torque[i][j] = jump_torque;
                        }
                    }
                    else 
                    {  // right foot
                        if (j == 1) 
                        {  // out leg
                            Torque[i][j] = jump_torque;
                        }
                        else 
                        {  // in leg
                            Torque[i][j] = -jump_torque;
                        }
                    }
                }
                else {
                    if (i == 0 || i == 3) 
                    {  // left foot
                        if (j == 1) {  // out leg
                            Torque[i][j] = jump_torque;
                        }
                        else 
                        {  // in leg
                            Torque[i][j] = -jump_torque;
                        }
                    }
                    else 
                    {  // right foot
                        if (j == 1) 
                        {  // out leg
                            Torque[i][j] = -jump_torque;
                        }
                        else 
                        {  // in leg
                            Torque[i][j] = jump_torque;
                        }
                    }
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 450, 5, PositionTorqueMode);

        for (int i = 0; i < 4; i++) 
        {
            for (int j = 0; j < 2; j++) 
            {
                Velocity[i][j] = 0;
                Torque[i][j] = 0;
            }
        }

        HAL_Delay(200);
        jump_forward_controller->jump_state = LegUp;
    }
    else if (jump_forward_controller->jump_state == LegUp) 
    {
      SetJumpForwardBezierControlPoints0(jump_forward_controller,  init_squat_length,  init_jump_length,  init_tilt_length);
        for (int i = 0; i < 4; i++) 
        {
            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN)
            {
                jump_forward_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) 
        {
            jump_forward_controller->jump_state = Land;
            jump_forward_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }

    }

    else if (jump_forward_controller->jump_state == Land) 
    {
        SetMotor(angle, Velocity, Torque, 0, 5, KdMode);
        HAL_Delay(2000);

        jump_forward_controller->jump_state = StandUp;
    }
    else if (jump_forward_controller->jump_state == StandUp) 
    {
    SetJumpForwardBezierControlPoints0(jump_forward_controller, squat_length0,jump_length0, tilt_length0);        
    for (int i = 0; i < 4; i++) 
        {
            ThreeOrderBezierPlan(&(jump_forward_controller->jump_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
            {
                jump_forward_controller->jump_state = EndJump;
                t = 0;
                last_t = -1;
                return;
            }
        }
        SetMotor(angle, Velocity, Torque, 40, 5, PositionMode);
        if (t >= 1000 && jump_forward_controller->jump_state_change == 1) 
{
            jump_forward_controller->jump_state = EndJump;
            jump_forward_controller->jump_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (jump_forward_controller->jump_state == EndJump) 
    {
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

void Turn_FSM (TurnController* turn_controller, float shorter_gait_length, float longer_gait_length, float gait_height, float robot_height) {
    // float bezier_height = 0.03;
    // float shorter_length = 0.04;
    // float longer_length = 0.1;
    // float robot_height = 0.2295;
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
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, LEFT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, RIGHT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, RIGHT_BACK_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, LEFT_BACK_LEG, NOT_TURNING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(turn_controller->turn_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                turn_controller->turn_state = EndTurn;
                t = 0;
                last_t = -1;
                return;
            }
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
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, LEFT_FRONT_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, RIGHT_BACK_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / fai_support;
        }
        else if (t_real >= fai_support && t_real <= 2.0) {
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, LEFT_FRONT_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, RIGHT_BACK_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0 - fai_support);
        }
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if (t_real >= 0 && t_real < (2.0 - fai_support)) {
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, RIGHT_FRONT_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, LEFT_BACK_LEG, TURNING_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0 - fai_support);
        }
        else if (t_real >= (2.0 - fai_support) && t_real <= 2.0) {
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, RIGHT_FRONT_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            SetTurnBezierControlPoints(turn_controller, shorter_gait_length, longer_gait_length, gait_height, LEFT_BACK_LEG, TURNING_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0) / fai_support;
        }
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(turn_controller->turn_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                turn_controller->turn_state = EndTurn;
                t = 0;
                last_t = -1;
                return;
            }
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
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, LEFT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, RIGHT_FRONT_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, RIGHT_BACK_LEG, NOT_TURNING);
        SetTurnBezierControlPoints(turn_controller, shorter_gait_length / 2, longer_gait_length / 2, gait_height, LEFT_BACK_LEG, NOT_TURNING);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(turn_controller->turn_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                turn_controller->turn_state = EndTurn;
                t = 0;
                last_t = -1;
                return;
            }
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
                   float trot_length, float shorter_length, float longer_length, float bezier_height, float trotting_state, float robot_height) {
    float t_real = t / 1000;
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
        if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
            Stand();
            return;
        }
    }

    SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
}

void Turn_to_Trot (TrotController* trot_controller, TurnController* turn_controller, 
                   float trot_length, float shorter_length, float longer_length, float bezier_height, float turning_state, float robot_height) {
    float t_real = t / 1000;
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
        if (IK_leg(bezier_x[i], robot_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
            Stand();
            return;
        }
    }

    SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);
}

void Stand_on_slope (float tan_slope_theta) 
{
    float length_between_legs = 0.4; //两条腿之间的举例（这里是前后腿的）
    float delta_height = length_between_legs * tan_slope_theta;//高度之差，两腿间的距离 * 倾斜角
    float delta_height_1 = delta_height / 3;
    float delta_height_2 = -delta_height / 3 * 2;// 两个不同程度的高度差？

    for (int i = 0; i < 4; i++) 
    {
        if (i < 2) 
        {
            if (IK_leg(0, robot_height - delta_height_1, &angle[i][0], &angle[i][1]) != NO_NAN) //给xy的值，返回电机角度
            {
                Stand();
                return;
            }
        }
        else {
            if (IK_leg(0, robot_height - delta_height_2, &angle[i][0], &angle[i][1]) != NO_NAN) {
                Stand();
                return;
            }
        }
    }

    SetMotor(angle, Velocity, Torque, 50, 5, PositionMode);
}

void SetWalkSlopeBezierControlPoints (TrotController* walk_slope_controller, float bezier_length, float delta_height, int leg, int walking_state) 
{
    if (3 * delta_height <= bezier_length - 0.001 || 3 * delta_height >= bezier_length + 0.001) 
    {
        return;
    }
    
    float control_points_x_1[4] = {0, 0, bezier_length, bezier_length};
    float control_points_x_2[4] = {0, 0, -bezier_length, -bezier_length};
    float control_points_x_3[4] = {-bezier_length / 2, -bezier_length / 2, bezier_length / 2, bezier_length / 2};
    float control_points_x_4[4] = {bezier_length / 2, bezier_length / 2, -bezier_length / 2, -bezier_length / 2};
    float control_points_x_5[4] = {bezier_length, bezier_length, 0, 0};
    float control_points_x_6[4] = {-bezier_length, -bezier_length, 0 ,0};
    float control_points_y_1[4] = {0, delta_height + 0.01, delta_height + 0.01, delta_height};
    float control_points_y_2[4] = {0, 0, -delta_height, -delta_height};
    float control_points_y_3[4] = {-delta_height / 2, delta_height / 2 + 0.01, delta_height / 2 + 0.01, delta_height / 2};
    float control_points_y_4[4] = {delta_height / 2, delta_height / 2, -delta_height / 2, -delta_height / 2};
    float control_points_y_5[4] = {delta_height, delta_height, 0, 0};
    float control_points_y_6[4] = {-delta_height, 0.01, 0.01, 0};

    if (walk_slope_controller->trot_state == PreTrot) 
    {
        if (leg == LEFT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[0]), control_points_x_1, control_points_y_1);
        }
        else if (leg == RIGHT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[1]), control_points_x_2, control_points_y_2);
        }
        else if (leg == RIGHT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[2]), control_points_x_1, control_points_y_1);
        }
        else if (leg == LEFT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[3]), control_points_x_2, control_points_y_2);
        }
    }
    else if (walk_slope_controller->trot_state == Trotting) 
    {
        if (walking_state == SLOPE_LF_RB_SUPPORT_RF_LB_SWING) 
        {
            if (leg == LEFT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[0]), control_points_x_4, control_points_y_4);
            }
            else if (leg == RIGHT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[1]), control_points_x_3, control_points_y_3);
            }
            else if (leg == RIGHT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[2]), control_points_x_4, control_points_y_4);
            }
            else if (leg == LEFT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[3]), control_points_x_3, control_points_y_3);
            }
        }
        else if (walking_state == SLOPE_LF_RB_SWING_RF_LB_SUPPORT) 
        {
            if (leg == LEFT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[0]), control_points_x_3, control_points_y_3);
            }
            else if (leg == RIGHT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[1]), control_points_x_4, control_points_y_4);
            }
            else if (leg == RIGHT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[2]), control_points_x_3, control_points_y_3);
            }
            else if (leg == LEFT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[3]), control_points_x_4, control_points_y_4);
            }
        }
        else {
            return;
        }
    }
    else if (walk_slope_controller->trot_state == PreEndTrot) 
    {
        if (leg == LEFT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[0]), control_points_x_5, control_points_y_5);
        }
        else if (leg == RIGHT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[1]), control_points_x_6, control_points_y_6);
        }
        else if (leg == RIGHT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[2]), control_points_x_5, control_points_y_5);
        }
        else if (leg == LEFT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_slope_controller->trot_bezier[3]), control_points_x_6, control_points_y_6);
        }
    }
    else {
        return;
    }
}

void WalkSlope_FSM (TrotController* walk_slope_controller, float tan_slope_theta, float length_between_legs, float robot_height, float gait_length, float delta_height) {
    float t_real = t / 1000;
    float t_real_2 = 0;
    // float tan_slope_theta = 1 / 3;
    // float origin_height = 0.2259;
    // float length_between_legs = 0;  // Not certain
    float shorter_height = robot_height - length_between_legs * tan_slope_theta / 3;
    float longer_height = robot_height + length_between_legs * tan_slope_theta / 3 * 2;//为什么不是二等分
    // float bezier_length = 0.12;
    // float max_height_1 = 0.03;
    // float max_height_2 = 0.01;
    // float delta_height = 0.04;
    for (int i = 0; i < 4; i++) {
        SetThreeOrderBezierPeriod(&walk_slope_controller->trot_bezier[i], 1.0);
    }

    float bezier_x[4];
    float bezier_y[4];
    float fai_swing = 1;
    float fai_support = 1;

    if (walk_slope_controller->trot_state == PreTrot) {
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, LEFT_FRONT_LEG, NOT_SLOPE);
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, RIGHT_FRONT_LEG, NOT_SLOPE);
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, RIGHT_BACK_LEG, NOT_SLOPE);
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, LEFT_BACK_LEG, NOT_SLOPE);

        for (int i = 0; i < 4; i++) 
        {
            ThreeOrderBezierPlan(&(walk_slope_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (i < 2) 
            {
                if (IK_leg(bezier_x[i], + - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
                {
                    Stand_on_slope((1.0 / 3));
                    return;
                }
            }
            else 
            {
                if (IK_leg(bezier_x[i], longer_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
                {
                    Stand_on_slope((1.0 / 3));
                    return;
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && walk_slope_controller->trot_state_change == 1) 
        {
            walk_slope_controller->trot_state = Trotting;
            walk_slope_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (walk_slope_controller->trot_state == Trotting)
    {
        if (t_real >= 0 && t_real < fai_support) 
        {
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, LEFT_FRONT_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, RIGHT_BACK_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / fai_support;
        }
        else if (t_real >= fai_support && t_real <= 2.0) 
        {
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, LEFT_FRONT_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, RIGHT_BACK_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0 - fai_support);
        }
        ThreeOrderBezierPlan(&(walk_slope_controller->trot_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(walk_slope_controller->trot_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if (t_real >= 0 && t_real < (2.0 - fai_support)) {
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, RIGHT_FRONT_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, LEFT_BACK_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0 - fai_support);
        }
        else if (t_real >= (2.0 - fai_support) && t_real <= 2.0) {
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, RIGHT_FRONT_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length, delta_height, LEFT_BACK_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0) / fai_support;
        }
        ThreeOrderBezierPlan(&(walk_slope_controller->trot_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(walk_slope_controller->trot_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            if (i < 2) {
                if (IK_leg(bezier_x[i], shorter_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_slope((1.0 / 3));
                    return;
                }
            }
            else {
                if (IK_leg(bezier_x[i], longer_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_slope((1.0 / 3));
                    return;
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 2000 && walk_slope_controller->trot_state_change == 1) {
            walk_slope_controller->trot_state = PreEndTrot;
            walk_slope_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (walk_slope_controller->trot_state == PreEndTrot) {
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, LEFT_FRONT_LEG, NOT_SLOPE);
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, RIGHT_FRONT_LEG, NOT_SLOPE);
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, RIGHT_BACK_LEG, NOT_SLOPE);
        SetWalkSlopeBezierControlPoints(walk_slope_controller, gait_length / 2, delta_height / 2, LEFT_BACK_LEG, NOT_SLOPE);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(walk_slope_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (i < 2) {
                if (IK_leg(bezier_x[i], shorter_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_slope((1.0 / 3));
                    return;
                }
            }
            else {
                if (IK_leg(bezier_x[i], longer_height - bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_slope((1.0 / 3));
                    return;
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && walk_slope_controller->trot_state_change == 1) {
            walk_slope_controller->trot_state = EndTrot;
            walk_slope_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (walk_slope_controller->trot_state == EndTrot) {
        walk_slope_controller->trot_enable = 0;
        Stand_on_slope((1.0 / 3));
    }
    else {
        return;
    }
}

void Stand_on_LR_slope (float tan_slope_theta) 
{
    float length_between_legs = 0.35; //两条腿之间的距离（这里是左右腿的）
    float delta_height = length_between_legs * tan_slope_theta;//高度之差，两腿间的距离 * 倾斜角
    float delta_height_1 = delta_height * 0.5;   //左腿抬腿高度？
    float delta_height_2 = - delta_height * 0.5;// 右腿的抬腿高度？

    for (int i = 0; i < 4; i++) 
    {
        if (i == 1 || i == 2) 
        {
            if (IK_leg(0, robot_height - delta_height_1, &angle[i][0], &angle[i][1]) != NO_NAN) //给xy的值，返回电机角度
            {
                Stand();
                return;
            }
        }

        else {
            if (IK_leg(0, robot_height - delta_height_2, &angle[i][0], &angle[i][1]) != NO_NAN) {
                Stand();
                return;
            }
        }
    }

    SetMotor(angle, Velocity, Torque, 50, 5, PositionMode);
}

void SetWalk_LR_SlopeBezierControlPoints (TrotController* walk_LR_slope_controller, float bezier_length, float delta_height, int leg, int walking_state) 
{
    float robot_height = 0.2069;
    float left_side = robot_height + 0.5 * delta_height;
    float right_side = robot_height - 0.5 * delta_height;
    
    float control_points_x_1[4] = {0, 0, bezier_length, bezier_length};
    float control_points_x_2[4] = {0, 0, -bezier_length, -bezier_length};
    float control_points_x_3[4] = {-bezier_length / 2, -bezier_length / 2, bezier_length / 2, bezier_length / 2};
    float control_points_x_4[4] = {bezier_length / 2, bezier_length / 2, -bezier_length / 2, -bezier_length / 2};
    float control_points_x_5[4] = {bezier_length, bezier_length, 0, 0};
    float control_points_x_6[4] = {-bezier_length, -bezier_length, 0 ,0};
    float control_points_y_1[4] = {left_side, left_side - 0.04, left_side - 0.02, left_side};
    float control_points_y_2[4] = {right_side, right_side - 0.04, right_side - 0.02, right_side};
//    float control_points_y_3[4] = (left_side, left_side, left_side, left_side);
//    float control_points_y_4[4] = (right_side, right_side, right_side, right_side);
    float control_points_y_3[4] = {left_side, left_side , left_side , left_side};
    float control_points_y_4[4] = {right_side, right_side , right_side , right_side};
//x方向的值不变，只修改y方向!!!
    if (walk_LR_slope_controller->trot_state == PreTrot) 
    {
        if (leg == LEFT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[0]), control_points_x_1, control_points_y_1);
        }
        else if (leg == RIGHT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[1]), control_points_x_2, control_points_y_4);
        }
        else if (leg == RIGHT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[2]), control_points_x_1, control_points_y_2);
        }
        else if (leg == LEFT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[3]), control_points_x_2, control_points_y_3);
        }
    }
    else if (walk_LR_slope_controller->trot_state == Trotting) 
    {
        if (walking_state == SLOPE_LF_RB_SUPPORT_RF_LB_SWING) 
        {
            if (leg == LEFT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[0]), control_points_x_4, control_points_y_3);
            }
            else if (leg == RIGHT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[1]), control_points_x_3, control_points_y_2);
            }
            else if (leg == RIGHT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[2]), control_points_x_4, control_points_y_4);
            }
            else if (leg == LEFT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[3]), control_points_x_3, control_points_y_1);
            }
        }
        else if (walking_state == SLOPE_LF_RB_SWING_RF_LB_SUPPORT) 
        {
            if (leg == LEFT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[0]), control_points_x_3, control_points_y_1);
            }
            else if (leg == RIGHT_FRONT_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[1]), control_points_x_4, control_points_y_4);
            }
            else if (leg == RIGHT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[2]), control_points_x_3, control_points_y_2);
            }
            else if (leg == LEFT_BACK_LEG) 
            {
                SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[3]), control_points_x_4, control_points_y_3);
            }
        }
        else {
            return;
        }
    }
    else if (walk_LR_slope_controller->trot_state == PreEndTrot) 
    {
        if (leg == LEFT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[0]), control_points_x_5, control_points_y_3);
        }
        else if (leg == RIGHT_FRONT_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[1]), control_points_x_6, control_points_y_2);
        }
        else if (leg == RIGHT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[2]), control_points_x_5, control_points_y_4);
        }
        else if (leg == LEFT_BACK_LEG) 
        {
            SetThreeOrderBezierControlPoints(&(walk_LR_slope_controller->trot_bezier[3]), control_points_x_6, control_points_y_1);
        }
    }
    else {
        return;
    }
}

void WalkSlope_LR_FSM (TrotController* walk_LR_slope_controller, float tan_slope_theta, float length_between_legs, float robot_height, float gait_length, float delta_height) 
{
    float t_real = t / 1000;
    float t_real_2 = 0;
    // float tan_slope_theta = 1 / 3;
    // float origin_height = 0.2259;
    // float length_between_legs = 0;  // Not certain
    // float bezier_length = 0.12;
    // float max_height_1 = 0.03;
    // float max_height_2 = 0.01;
    // float delta_height = 0.04;
    for (int i = 0; i < 4; i++) 
    {
        SetThreeOrderBezierPeriod(&walk_LR_slope_controller->trot_bezier[i], 1.0);
    }

    float bezier_x[4];
    float bezier_y[4];
    float fai_swing = 1;
    float fai_support = 1;

    if (walk_LR_slope_controller->trot_state == PreTrot) 
    {
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, LEFT_FRONT_LEG, NOT_SLOPE);
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, RIGHT_FRONT_LEG, NOT_SLOPE);
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, RIGHT_BACK_LEG, NOT_SLOPE);
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, LEFT_BACK_LEG, NOT_SLOPE);
        for (int i = 0; i < 4; i++) 
        {
            ThreeOrderBezierPlan(&(walk_LR_slope_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (i == 1 || i == 2) {
                if (IK_leg(bezier_x[i], bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
                {
                    Stand_on_LR_slope((0.2679));
                    return;
                }
            }
            else {
                if (IK_leg(bezier_x[i], bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_LR_slope((0.2679));
                    return;
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && walk_LR_slope_controller->trot_state_change == 1) {
            walk_LR_slope_controller->trot_state = Trotting;
            walk_LR_slope_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (walk_LR_slope_controller->trot_state == Trotting)
    {
        if (t_real >= 0 && t_real < fai_support) 
        {
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, LEFT_FRONT_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, RIGHT_BACK_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / fai_support;
        }
        else if (t_real >= fai_support && t_real <= 2.0) 
        {
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, LEFT_FRONT_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, RIGHT_BACK_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real - fai_support) / (2.0 - fai_support);
        }
        ThreeOrderBezierPlan(&(walk_LR_slope_controller->trot_bezier[0]), t_real_2, &bezier_x[0], &bezier_y[0]);
        ThreeOrderBezierPlan(&(walk_LR_slope_controller->trot_bezier[2]), t_real_2, &bezier_x[2], &bezier_y[2]);

        if (t_real >= 0 && t_real < (2.0 - fai_support)) {
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, RIGHT_FRONT_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, LEFT_BACK_LEG, SLOPE_LF_RB_SUPPORT_RF_LB_SWING);
            t_real_2 = t_real / (2.0 - fai_support);
        }
        else if (t_real >= (2.0 - fai_support) && t_real <= 2.0) {
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, RIGHT_FRONT_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length, delta_height, LEFT_BACK_LEG, SLOPE_LF_RB_SWING_RF_LB_SUPPORT);
            t_real_2 = (t_real + fai_support - 2.0) / fai_support;
        }
        ThreeOrderBezierPlan(&(walk_LR_slope_controller->trot_bezier[1]), t_real_2, &bezier_x[1], &bezier_y[1]);
        ThreeOrderBezierPlan(&(walk_LR_slope_controller->trot_bezier[3]), t_real_2, &bezier_x[3], &bezier_y[3]);

        for (int i = 0; i < 4; i++) {
            if (i == 1 || i == 2) {
                if (IK_leg(bezier_x[i], bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_LR_slope((0.2679));
                    return;
                }
            }
            else {
                if (IK_leg(bezier_x[i], bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) {
                    Stand_on_LR_slope((0.2679));
                    return;
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 2000 && walk_LR_slope_controller->trot_state_change == 1) {
            walk_LR_slope_controller->trot_state = PreEndTrot;
            walk_LR_slope_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (walk_LR_slope_controller->trot_state == PreEndTrot) {      
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, LEFT_FRONT_LEG, NOT_SLOPE);
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, RIGHT_FRONT_LEG, NOT_SLOPE);
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, RIGHT_BACK_LEG, NOT_SLOPE);
        SetWalk_LR_SlopeBezierControlPoints(walk_LR_slope_controller, gait_length / 2, delta_height / 2, LEFT_BACK_LEG, NOT_SLOPE);

        for (int i = 0; i < 4; i++) {
            ThreeOrderBezierPlan(&(walk_LR_slope_controller->trot_bezier[i]), t_real, &bezier_x[i], &bezier_y[i]);
            if (i == 1 || i == 2) {
                if (IK_leg(bezier_x[i], bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
                {
                    Stand_on_LR_slope((0.2679));
                    return;
                }
            }
            else {
                if (IK_leg(bezier_x[i], bezier_y[i], &angle[i][0], &angle[i][1]) != NO_NAN) 
                {
                    Stand_on_LR_slope((0.2679));
                    return;
                }
            }
        }

        SetMotor(angle, Velocity, Torque, 100, 5, PositionMode);

        if (t >= 1000 && walk_LR_slope_controller->trot_state_change == 1) {
            walk_LR_slope_controller->trot_state = EndTrot;
            walk_LR_slope_controller->trot_state_change = 0;
            t = 0;
            last_t = -1;
        }
    }
    else if (walk_LR_slope_controller->trot_state == EndTrot) 
    {
        walk_LR_slope_controller->trot_enable = 0;
        Stand_on_LR_slope((0.2679));
    }
    else {
        return;
    }
}