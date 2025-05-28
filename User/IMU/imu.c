#include "imu.h"

uint8_t imu_rx_data[IMU_DATA_SIZE];

Quaternions Q_Value;
Euler EulerAngle;

float yaw_error_filter = 0;
float delta_length_filter = 0;

uint32_t u8_to_u32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return (uint32_t)a << 24 | 
           (uint32_t)b << 16 | 
           (uint32_t)c << 8  | 
           (uint32_t)d;
}

void IMU_quaterToEulerianAngles()
{
    float q0 = Q_Value.x.val_float;
    float q1 = Q_Value.y.val_float;
    float q2 = Q_Value.z.val_float;
    float q3 = Q_Value.w.val_float;
    EulerAngle.pitch = asin(-2*q1*q3 + 2*q0*q2) * 180/M_PI; // pitch
    EulerAngle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180/M_PI; // roll
    EulerAngle.yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180/M_PI; // yaw
}

int compute_lengths (float imu_yaw_error, float* left_length, float* right_length, float origin_length, float a1, float a2, float kp) {
    if (a1 < 0 || a1 > 1) {
        return PARAM_ERROR;
    }
    if (a2 < 0 || a2 > 1) {
        return PARAM_ERROR;
    }

    yaw_error_filter = a1 * imu_yaw_error + (1 - a1) * yaw_error_filter;
    if (fabs(yaw_error_filter) < YAW_DEAD_AREA) {
        yaw_error_filter = 0;
    }

    float delta_length = kp * yaw_error_filter;
    delta_length_filter = a2 * delta_length + (1 - a2) * delta_length_filter;
    if (fabs(delta_length_filter) < DELTA_LENGTH_DEAD_AREA || yaw_error_filter == 0) {
        delta_length_filter = 0;
    }

    if (yaw_error_filter < 0) {
        *left_length = origin_length + delta_length_filter / 2;
        *right_length = origin_length - delta_length_filter / 2;
        return RIGHT;
    }
    else if (yaw_error_filter > 0) {
        *left_length = origin_length - delta_length_filter / 2;
        *right_length = origin_length + delta_length_filter / 2;
        return LEFT;
    }
    else {
        *left_length = origin_length;
        *right_length = origin_length;
        return NO_TURN;
    }
}