/**
 * ----------------------------------- Include -----------------------------------
 */
#include "CurvePlan.h"
#include "math.h"

/**
 * ----------------------------------- Variables -----------------------------------
 */

/**
 * ----------------------------------- Functions -----------------------------------
 */
void SetCyloid (CyloidInformation* cyloid, float height, float length, float period, float duty_cycle) {
    cyloid->height = height;
    cyloid->length = length;
    cyloid->period = period;
    cyloid->duty_cycle = duty_cycle;
}

void CyloidPlan (CyloidInformation* cyloid, float t, int direction, int leg_flag, float* cyloid_x, float* cyloid_y) {
    float x_start = -cyloid->length / 2 * direction;
    float x_end = cyloid->length  / 2 * direction;
    float pi = 3.1415926;
    float theta;

    if (t >= cyloid->period) {
        t = 0;
    }

    if (leg_flag == 0) {
        if (t >= 0 && t < cyloid->duty_cycle * cyloid->period) {
            theta = 2 * pi * t / (cyloid->duty_cycle * cyloid->period);
            *cyloid_x = (x_start - x_end) * (theta - sin(theta)) / (2 * pi) + x_start;
            *cyloid_y = cyloid->height / 2 * (1 - cos(theta));
        }
        else if (t >= cyloid->duty_cycle * cyloid->period && t < cyloid->period) {
            theta = 2 * pi * (t - cyloid->duty_cycle * cyloid->period) / (cyloid->period - cyloid->duty_cycle * cyloid->period);
            *cyloid_x = -((x_end - x_start) * (theta - sin(theta)) / (2 * pi) + x_start);
            *cyloid_y = 0;
        }
    }
    else if (leg_flag == 1) {
        if (t >= 0 && t < cyloid->period - cyloid->duty_cycle * cyloid->period) {
            theta = 2 * pi * t / (cyloid->period - cyloid->duty_cycle * cyloid->period);
            *cyloid_x = -((x_end - x_start) * (theta - sin(theta)) / (2 * pi) + x_start);
            *cyloid_y = 0;
        }
        else if (t >= cyloid->period - cyloid->duty_cycle * cyloid->period && t < cyloid->period) {
            theta = 2 * pi * (t - cyloid->duty_cycle * cyloid->period) / (cyloid->period - cyloid->duty_cycle * cyloid->period);
            *cyloid_x = (x_start - x_end) * (theta - sin(theta)) / (2 * pi) + x_start;
            *cyloid_y = cyloid->height / 2 * (1 - cos(theta));
        }
    }
}

void SetThreeOrderBezierControlPoints (ThreeOrderBezierInformation* bezier, float* control_points_x, float* control_points_y) {
    for (int i = 0; i < 4; i++) {
        bezier->control_points_x[i] = control_points_x[i];
        bezier->control_points_y[i] = control_points_y[i];
    }
}

void SetThreeOrderBezierPeriod (ThreeOrderBezierInformation* bezier, float period) {
    bezier->period = period;
}

void SetThreeOrderBezier (ThreeOrderBezierInformation* bezier, float* control_points_x, float* control_points_y, float period) {
    SetThreeOrderBezierControlPoints(bezier, control_points_x, control_points_y);
    SetThreeOrderBezierPeriod(bezier, period);
}

void ThreeOrderBezierPlan (ThreeOrderBezierInformation* bezier, float t, float* bezier_x, float* bezier_y) {
    if (t > bezier->period) {
        t = bezier->period;
    }
    if (t < 0) {
        t = 0;
    }
    if (t <= bezier->period) 
    {
        t = t / bezier->period;
        *bezier_x = pow((1 - t), 3) * bezier->control_points_x[0] +
                    3 * pow((1 - t), 2) * t * bezier->control_points_x[1] + 
                    3 * (1 - t) * pow(t, 2) * bezier->control_points_x[2] + 
                    pow(t, 3) * bezier->control_points_x[3];
        *bezier_y = pow((1 - t), 3) * bezier->control_points_y[0] +
                    3 * pow((1 - t), 2) * t * bezier->control_points_y[1] + 
                    3 * (1 - t) * pow(t, 2) * bezier->control_points_y[2] + 
                    pow(t, 3) * bezier->control_points_y[3];
    }
}

void LinePlan(ThreeOrderBezierInformation* bezier, float t, float* bezier_x, float* bezier_y)
{
  if (t <= bezier->period) 
  {
      t = t / bezier->period;
      *bezier_x = (bezier->control_points_x[3] - bezier->control_points_x[0]) * t + bezier->control_points_x[0];
      *bezier_y = (bezier->control_points_y[3] - bezier->control_points_y[0]) * t + bezier->control_points_y[0];
  }
}