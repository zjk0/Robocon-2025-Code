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
} TrotCurve;

// The struct to describe the properties of trotting
typedef struct {
    TrotState trot_state;
    TrotDirection trot_direction;
    TrotCurve trot_curve;
} TrotController;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern TrotController trot_controller;
extern uint8_t TrotStateChange;
extern uint8_t rotate_direction;
extern int Debug1;
extern int Debug2;
extern int Debug3;
extern int Debug4;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Trot_FSM (TrotController* trot_controller);
