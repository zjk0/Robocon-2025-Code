#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "DeepJ60_Motor.h"
#include "KinematicSolution.h"
#include "usart.h"
#include "math.h"
#include "CurvePlan.h"

/**
 * ----------------------------------- Marcos -----------------------------------
 */
#define LEFT_FRONT_LEG 0
#define RIGHT_FRONT_LEG 1
#define RIGHT_BACK_LEG 2
#define LEFT_BACK_LEG 3

#define TROTTING_LF_RB_SUPPORT_RF_LB_SWING 0
#define TROTTING_LF_RB_SWING_RF_LB_SUPPORT 1
#define NOT_TROTTING 2

#define ROTATING_LF_RB_SUPPORT_RF_LB_SWING 0
#define ROTATING_LF_RB_SWING_RF_LB_SUPPORT 1
#define NOT_ROTATING 2

#define TURNING_LF_RB_SUPPORT_RF_LB_SWING 0
#define TURNING_LF_RB_SWING_RF_LB_SUPPORT 1
#define NOT_TURNING 2

#define TORQUE_DEAD_AREA 0.01f
#define POSITION_DEAD_AREA 0.01f

/**
 * ----------------------------------- Typedef -----------------------------------
 */
// The state of the process of trotting
typedef enum {
    PreTrot = 0,
    Trotting,
    PreEndTrot,
    EndTrot
} TrotState;

// The direction of trotting
typedef enum {
    Back = -1,
    Forward = 1
} TrotDirection;

// The struct to describe the properties of trotting
typedef struct {
    TrotState trot_state;
    TrotDirection trot_direction;
    ThreeOrderBezierInformation trot_bezier[4];  // The bezier objects of four legs
    float swing_duty_cycle;
    int trot_enable;
    int trot_state_change;
} TrotController;

typedef enum {
    PreRotate = 0,
    Rotating,
    PreEndRotate,
    EndRotate
} RotateState;

typedef enum {
    Left = -1, 
    Right = 1
} RotateDirection;

typedef struct {
    RotateState rotate_state;
    RotateDirection rotate_direction;
    ThreeOrderBezierInformation rotate_bezier[4];  // The bezier objects of four legs
    float swing_duty_cycle;
    int rotate_enable;
    int rotate_state_change;
} RotateController;

typedef enum {
    Squat = 0,
    JumpUp,
    LegUp,
    Land,
    StandUp,
    EndJump
} JumpState;

typedef struct {
    JumpState jump_state;
    ThreeOrderBezierInformation jump_bezier[4];  // The bezier objects of four legs
    int jump_enable;
    int jump_state_change;
} JumpController;

typedef enum {
    PreTurn = 0, 
    Turning, 
    PreEndTurn, 
    EndTurn
} TurnState;

typedef enum {
    TurnLeft = 0, 
    TurnRight
} TurnAngularVelocityDirection;

typedef enum {
    LinearBack = -1, 
    LinearForward = 1
} TurnLinearVelocityDirection;

typedef struct {
    TurnState turn_state;
    TurnAngularVelocityDirection turn_angular_direction;
    TurnLinearVelocityDirection turn_linear_direction;
    ThreeOrderBezierInformation turn_bezier[4];  // The bezier objects of four legs
    float swing_duty_cycle;
    int turn_enable;
    int turn_state_change;
} TurnController;

typedef union {
    float real_motor_data[15];
    uint8_t send_motor_data[60];
} usart_data;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern TrotController trot_controller;
extern RotateController rotate_controller;
extern JumpController jump_up_controller;
extern JumpController jump_forward_controller;
extern TurnController turn_controller;

extern usart_data usart_motor_data;

extern float angle[4][2];
extern float t;
extern float last_t;

extern float J60Motor_StandUpData_CAN1[4];  // lf_out, lf_in, rf_out, rf_in
extern float J60Motor_StandUpData_CAN2[4];  // rb_out, rb_in, lb_out, lb_in

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Trot_FSM (TrotController* trot_controller);
void Rotate_FSM (RotateController* rotate_controller);
void JumpUp_FSM (JumpController* jump_up_controller);
void JumpForward_FSM (JumpController* jump_forward_controller);
void Turn_FSM (TurnController* turn_controller);
void Trot_to_Turn (TrotController* trot_controller, TurnController* turn_controller, float trot_length, float shorter_length, float longer_length, float bezier_height, float trotting_state);
void Turn_to_Trot (TrotController* trot_controller, TurnController* turn_controller, float trot_length, float shorter_length, float longer_length, float bezier_height, float turning_state);
