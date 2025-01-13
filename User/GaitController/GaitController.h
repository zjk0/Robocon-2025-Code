#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "DeepJ60_Motor.h"
#include "leg_control.h"
#include "main.h"
#include "usart.h"
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
} RotateController;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern TrotController trot_controller;
extern uint8_t TrotStateChange;
extern uint8_t RotateStateChange;
extern RotateController rotate_controller;
extern uint8_t TrotEnable;
extern uint8_t RotateEnable;
extern int Debug1;
extern int Debug2;
extern int Debug3;
extern int Debug4;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Trot_FSM (TrotController* trot_controller);
void Rotate_FSM (RotateController* rotate_controller);
