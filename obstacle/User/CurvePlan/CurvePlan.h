#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */

/**
 * ----------------------------------- Typedef -----------------------------------
 */
// The struct to describe the properties of cyloid
typedef struct {
    float height;
    float length;
    float period;
    float duty_cycle;
} CyloidInformation;

// The struct to describe the properties of three order bezier
typedef struct {
    float control_points_x[4];
    float control_points_y[4];
    float period;
} ThreeOrderBezierInformation;

/**
 * ----------------------------------- Variables -----------------------------------
 */

/**
 * ----------------------------------- Functions -----------------------------------
 */
// Cyloid
void SetCyloid (CyloidInformation* cyloid, float height, float length, float period, float duty_cycle);
void CyloidPlan (CyloidInformation* cyloid, float t, int direction, int leg_flag, float* cyloid_x, float* cyloid_y);

// Three order bezier
void SetThreeOrderBezierControlPoints (ThreeOrderBezierInformation* bezier, float* control_points_x, float* control_points_y);
void SetThreeOrderBezierPeriod (ThreeOrderBezierInformation* bezier, float period);
void SetThreeOrderBezier (ThreeOrderBezierInformation* bezier, float* control_points_x, float* control_points_y, float period);
void ThreeOrderBezierPlan (ThreeOrderBezierInformation* bezier, float t, float* bezier_x, float* bezier_y);
void LinePlan(ThreeOrderBezierInformation* bezier, float t, float* bezier_x, float* bezier_y);