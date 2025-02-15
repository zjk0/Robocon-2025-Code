/**
 * ----------------------------------- Include -----------------------------------
 */
#include "CurvePlan.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */
ThreeOrderBezierInformation bezier;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void SetCyloid (CyloidInformation* cyloid, float height, float length, float period) {
    cyloid->height = height;
    cyloid->length = length;
    cyloid->period = period;
}

void SetThreeOrderBezier (ThreeOrderBezierInformation* bezier, float* control_points_x, float* control_points_y) {
    for (int i = 0; i < 4; i++) {
        bezier->control_points_x[i] = control_points_x[i];
        bezier->control_points_y[i] = control_points_y[i];
    }
}

// float CyloidPlan (CyloidInformation* cyloid, float t) {

// }

// float ThreeOrderBezierPlan (ThreeOrderBezierInformation* bezier, float t) {

// }

// float CurvePlan (CurveType curve) {
//     float t = 0;
//     float curve_point = 0;

//     if (curve == Cyloid) {
//         curve_point = GenerateCyloid(t);
//     }
//     else if (curve == ThreeOrderBezier) {
//         curve_point = GenerateBezier(t);
//     }

//     return curve_point;
// }