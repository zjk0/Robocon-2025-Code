#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */

/**
 * ----------------------------------- Typedef -----------------------------------
 */
typedef enum {
    Cyloid = 0,
    ThreeOrderBezier
} CurveType;

typedef struct {
    float height;
    float length;
    float period;
} CyloidInformation;

typedef struct {
    float height;
    float length;
    float control_points_x[4];
    float control_points_y[4];
} ThreeOrderBezierInformation;

/**
 * ----------------------------------- Variables -----------------------------------
 */
extern CyloidInformation cyloid;
extern ThreeOrderBezierInformation bezier;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void SetCyloid (CyloidInformation* cyloid, float height, float length, float period);
void SetThreeOrderBezier (ThreeOrderBezierInformation* bezier, float* control_points_x, float* control_points_y);
float CyloidPlan (CyloidInformation* cyloid, float t);
float ThreeOrderBezierPlan (ThreeOrderBezierInformation* bezier, float t);
float CurvePlan (CurveType curve);