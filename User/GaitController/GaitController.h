#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "DeepJ60_Motor.h"
#include "leg_control.h"
#include "main.h"
#include "usart.h"
#include "math.h"
// #include "CurvePlan.h"

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
    Forward = 0,
    Back
} TrotDirection;

typedef enum {
    Cyloid = 0,
    ThreeOrderBezier
} Curve;

// The struct to describe the properties of trotting
typedef struct {
    TrotState trot_state;
    TrotDirection trot_direction;
    Curve trot_curve;
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
    Left = 0, 
    Right
} RotateDirection;

typedef struct {
    RotateState rotate_state;
    RotateDirection rotate_direction;
    Curve rotate_curve;
    float swing_duty_cycle;
    int rotate_enable;
    int rotate_state_change;
} RotateController;

// typedef struct {
//     float init_position;
//     float goal_position;
//     float now_position;
//     float init_velocity;
//     float goal_velocity;
//     float total_time;
// } CubicSplineInformation;

typedef struct {
    int stop;
    int re_init;
    float re_init_x[4];
    float re_init_y[4];
} SuddenSituation;

typedef enum {
    Squat = 0,
    JumpUp,
    LegUp,
    Land,
    EndJump
} JumpState;

typedef struct {
    JumpState jump_state;
    int jump_enable;
    int jump_state_change;
} JumpController;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern TrotController trot_controller;
extern RotateController rotate_controller;
extern SuddenSituation sudden_situation;
extern JumpController jump_controller;

// extern CubicSplineInformation ReInit_can1[4];
// extern CubicSplineInformation ReInit_can2[4];
extern float ReInit_can1[2];
extern float ReInit_can2[2];

extern uint8_t reinit_signal[1];

extern int Debug1;
extern int Debug2;
extern int Debug3;
extern int Debug4;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void SetMotor (void);
void Trot_FSM (TrotController* trot_controller);
void Rotate_FSM (RotateController* rotate_controller);
void ReInit (float t);
