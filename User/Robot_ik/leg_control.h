#ifndef LEG_CONTROL_H   // 头文件保护，防止重复包含
#define LEG_CONTROL_H

#include "main.h"

// 函数声明
void IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i);
void Leg_cyloid(float *t, float *angle_e, float *angle_i, int leg_flag, int direction, float fai, float step_length,float step_high);
void Cubic_Bezier(float *t, float *angle_e, float *angle_i, float fai, float start_x, float start_z, float end_x, float end_z, float max_z,float Start_phase);
void Quadruped_gait(float *t, float (*angle)[2],int gait_state, float fai, float step_length,float step_high);

void Walk_straight_Bezier(float *t, float (*angle)[2], float fai, float start_x, float start_z, float end_x, float end_z, float max_z, uint8_t direction_mode);
void Walk_turn_Bezier(float *t, float (*angle)[2], float fai, float start_x, float start_z, float end_x, float end_z, float max_z, uint8_t direction_mode);
void Walk_straight_cyloid(float *t, float (*angle)[2], float fai, float step_length, float step_high, uint8_t direction_mode);

void Direct_Solution(float angle_e, float angle_i, float* x_pos, float* y_pos);
void line(float *t, float *angle_e, float *angle_i, float start_x, float start_z, float end_x, float end_z);

float CubicSpline(float init_position, float goal_position, float init_velocity, float goal_velocity, float now_time, float total_time);
// 宏定义

//关于腿部的前进后退
#define forward 0
#define backward 1

//关于四足整体的前进后退左转右转
#define Advance 0
#define Retreat 1
#define Turn_left 2
#define Turn_right 3

//curve_line_mode足端轨迹曲线的模式
#define cyloid 0    //摆线
#define Bezier 1    //贝塞尔曲线

//direction_mode对于足端轨迹为贝塞尔曲线时，腿部的方向
#define ahead_ward 0
#define back_ward 1
#define left 0
#define right 1

typedef union {
  float real_motor_data[15];
  uint8_t send_motor_data[60];
} usart_data;

extern usart_data usart_motor_data;
extern float J60Motor_StandUpData_CAN1[4];
extern float J60Motor_StandUpData_CAN2[4];


#endif /* leg_control_H */