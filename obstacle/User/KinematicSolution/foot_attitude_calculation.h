#pragma once

/**
 * ----------------------------------- Include -----------------------------------
 */

/**
 * ----------------------------------- Typedef -----------------------------------
 */
// The struct to describe the properties of cyloid
typedef struct{
	float X;
	float Y;
	
}point;

typedef struct{
	float alpha;
	float thet;
}Angle;

typedef struct {
    point FrontLeft;  // ǰ����
    point FrontRight; // ǰ����
    point RearLeft;   // ������
    point RearRight;  // ������
}All_legs_place;
/**
 * ----------------------------------- Variables -----------------------------------
 */
extern point Foot_Position;
extern All_legs_place Robot_Legs_Position;

/**
 * ----------------------------------- Functions -----------------------------------
 */
void angle_to_position();