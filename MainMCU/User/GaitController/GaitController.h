#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "DeepJ60_Motor.h"
#include "KinematicSolution.h"
#include "usart.h"
#include "spi.h"
#include "math.h"
#include "CurvePlan.h"
#include "FreeRTOS.h"
#include "task.h"

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

#define SLOPE_LF_RB_SUPPORT_RF_LB_SWING 0
#define SLOPE_LF_RB_SWING_RF_LB_SUPPORT 1
#define NOT_SLOPE 2

#define LEFT_UP 0
#define RIGHT_UP 1
#define NOT_MOVING 2

#define NO_SLOPE 0
#define SLOPE 1
#define SLOPE_LR 2

#define NO_STOP 0
#define NEED_TO_STOP 1

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
} RotateController;

typedef enum {
    Recline = 0,
    Squat,
    JumpUp,
    LegUp,
    Land,
    StandUp,
    EndJump
} JumpState;

typedef struct {
    JumpState jump_state;
    ThreeOrderBezierInformation jump_bezier[4];  // The bezier objects of four legs
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
} TurnController;

typedef enum {
    UpSlope = 0, 
    DownSlope
} WalkSlopeDirection;

typedef enum {
    PreWalk = 0,
    Walking,
    PreEndWalk,
    EndWalk
} WalkSlopeState;

typedef struct {
    WalkSlopeState walk_slope_state;
    WalkSlopeDirection walk_slope_direction;
    ThreeOrderBezierInformation walk_slope_bezier[4];  // The bezier objects of four legs
    float swing_duty_cycle;
} WalkSlopeController;

typedef enum {
    PreMove = 0,
    Moving, 
    PreEndMove,
    EndMove
} HorizontalMoveState;

typedef enum {
    MoveLeft = 0,
    MoveRight
} HorizontalMoveDirection;

typedef struct {
    HorizontalMoveState move_state;
    HorizontalMoveDirection move_direction;
    ThreeOrderBezierInformation move_bezier[4];
    float swing_duty_cycle;
} HorizontalMoveController;

typedef union {
    float real_motor_data[15];
    uint8_t send_motor_data[60];
} spi_data;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern TrotController trot_controller;
extern RotateController rotate_controller;
extern JumpController jump_up_controller;
extern JumpController jump_forward_controller;
extern TurnController turn_controller;
extern TrotController walk_slope_controller;
extern TrotController walk_LR_slope_controller;
extern HorizontalMoveController move_controller;

extern spi_data spi_motor_data;

extern float angle[4][2];
extern float t;
extern float pre_t;

extern float robot_height;
extern int isSlope;

extern int isStop;

extern float tan_slope_theta;
extern float tan_LR_slope_theta;

extern float J60Motor_StandUpData_CAN1[4];  // lf_out, lf_in, rf_out, rf_in
extern float J60Motor_StandUpData_CAN2[4];  // rb_out, rb_in, lb_out, lb_in

extern float left_length;
extern float right_length;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Stand (void);
void Trot_FSM (TrotController* trot_controller, float gait_height, float gait_length, float robot_height);
void Rotate_FSM (RotateController* rotate_controller, float gait_height, float gait_length, float robot_height);
void JumpUp_FSM(JumpController *jump_up_controller);
void JumpForward_FSM (JumpController* jump_forward_controller);
void Turn_FSM (TurnController* turn_controller, float shorter_gait_length, float longer_gait_length, float gait_height, float robot_height);
void Trot_to_Turn (TrotController* trot_controller, TurnController* turn_controller, float trot_length, float shorter_length, float longer_length, float bezier_height, float trotting_state, float robot_height);
void Turn_to_Trot (TrotController* trot_controller, TurnController* turn_controller, float trot_length, float shorter_length, float longer_length, float bezier_height, float turning_state, float robot_height);
void Stand_on_slope (float tan_slope_theta);
void WalkSlope_FSM (TrotController* walk_slope_controller, float tan_slope_theta, float length_between_legs, float robot_height, float gait_length, float delta_height);
void Stand_on_LR_slope (float tan_slope_theta);
void SetWalk_LR_SlopeBezierControlPoints (TrotController* walk_LR_slope_controller, float bezier_length, float delta_height, int leg, int walking_state);
void WalkSlope_LR_FSM (TrotController* walk_LR_slope_controller, float tan_slope_theta, float length_between_legs, float robot_height, float gait_length, float delta_height);
void HorizontalMove_FSM (HorizontalMoveController* move_controller, float longer_height, float shorter_height, float robot_height, float delta_height);
