#ifndef KINEMATIC_SOLUTION_H
#define KINEMATIC_SOLUTION_H

/**
 * ----------------------------------- Include -----------------------------------
 */
#include "stdint.h"

/**
 * ----------------------------------- Marcos -----------------------------------
 */
#define NAN_ERROR 0
#define NO_NAN 1

/**
 * ----------------------------------- Functions -----------------------------------
 */
int IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i);
void Direct_Solution(float angle_e, float angle_i, float* x_pos, float* y_pos);

#endif