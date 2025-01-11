#ifndef LEG_CONTROL_H   // 头文件保护，防止重复包含
#define LEG_CONTROL_H

#include "main.h"

// 函数声明
void IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i);
void Leg_cyloid(float *t, float *angle_e, float *angle_i, int leg_flag, int direction, float fai, float step_length,float step_high);
void Cubic_Bezier(float *t, float *angle_e, float *angle_i,uint8_t leg_flag, float fai, float start_x, float start_z, float end_x, float end_z, float max_z);
void Quadruped_gait(float *t, float (*angle)[2],int gait_state, float fai, float step_length,float step_high);

void Walk_straight_Bezier(float *t, float (*angle)[2], float fai, float start_x, float start_z, float end_x, float end_z, float max_z, uint8_t direction_mode);
void Walk_turn_Bezier(float *t, float (*angle)[2], float fai, float start_x, float start_z, float end_x, float end_z, float max_z, uint8_t direction_mode);
void Walk_straight_cyloid(float *t, float (*angle)[2], float fai, float step_length, float step_high, uint8_t direction_mode);



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

//curve_line_mode足端轨迹曲线的模式
#define cyloid 1    //摆线
#define Bezier 2    //贝塞尔曲线

//direction_mode对于足端轨迹为贝塞尔曲线时，腿部的方向
#define ahead_ward 1
#define back_ward 2
#define left 3
#define right 4

typedef union {
  float real_motor_data[4];
  uint8_t send_motor_data[16];
} usart_data;

typedef enum {
    PreTrot = 0,
    Troting,
    PreEndTrot,
    EndTrot
} TrotState;

extern TrotState trot_state;
extern usart_data usart_motor_data;
extern float J60Motor_StandUpData_CAN1[4];
extern float J60Motor_StandUpData_CAN2[4];


#endif /* leg_control_H */