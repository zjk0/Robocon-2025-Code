#ifndef LEG_CONTROL_H   // 头文件保护，防止重复包含
#define LEG_CONTROL_H

#include "main.h"

// 函数声明
void IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i);
void Leg_cyloid(float *t, float *angle_e, float *angle_i, int leg_flag, int direction, float fai, float step_length,float step_high);
void Cubic_Bezier_Curve(float *t,float fhi,float high,float half_wide, float *angle_e,float *angle_i);
void Quadruped_gait(float *t, float (*angle)[2],int gait_state, float fai, float step_length,float step_high);

float CubicSpline(float init_position, float goal_position, float init_velocity, float goal_velocity, float now_time, float total_time);
void Jump(float start_point, float end_point);
// 宏定义

//关于腿部的前进后退
#define forward 0
#define backward 1

//关于四足整体的前进后退左转右转
#define Advance 1
#define Retreat 2
#define Turn_left 3
#define Turn_right 4

typedef union {
  float real_motor_data[4];
  uint8_t send_motor_data[16];
} usart_data;

extern usart_data usart_motor_data;
extern float J60Motor_StandUpData_CAN1[4];
extern float J60Motor_StandUpData_CAN2[4];


#endif /* leg_control_H */