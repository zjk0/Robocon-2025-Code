#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "DeepJ60_Motor.h"
#include "leg_control.h"
#include "GenerateCurve.h"
#include "main.h"
#include "usart.h"

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

// The struct to describe the properties of trotting
typedef struct {
    TrotState trot_state;
    TrotDirection trot_direction;
    CurveType trot_curve;
} TrotController;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern TrotController trot_controller;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void Trot_FSM (TrotController* trot_controller);
