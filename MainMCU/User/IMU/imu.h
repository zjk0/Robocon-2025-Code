#pragma once

#include "math.h"
#include "main.h"

#define YAW_DEAD_AREA 0.01f
#define DELTA_LENGTH_DEAD_AREA 0.001f

#define M_PI 3.1415926

#define PARAM_ERROR 0
#define LEFT 1
#define RIGHT 2
#define NO_TURN 3

#define IMU_DATA_SIZE 23

union Value{
	uint32_t val_32bit;
	float val_float;
};

typedef struct{
	union Value x;
	union Value y;
	union Value z;
	union Value w;
}Quaternions;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}Euler;

extern float imu_yaw_error;
extern float yaw_error_filter;
extern float delta_length_filter;

extern Quaternions Q_Value;
extern Euler EulerAngle;

extern uint8_t imu_rx_data[IMU_DATA_SIZE];

uint32_t u8_to_u32(uint8_t a, uint8_t b, uint8_t c, uint8_t d) ;
void IMU_quaterToEulerianAngles();
int compute_lengths (float imu_yaw_error, float* left_length, float* right_length, float origin_length, float a1, float a2, float kp);