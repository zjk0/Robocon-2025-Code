#ifndef LEG_CONTROL_H   // 头文件保护，防止重复包含
#define LEG_CONTROL_H

// 函数声明
void IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i);
void Leg_cyloid(float *t, float *angle_e, float *angle_i, int leg_flag, int direction, float fai, float step_length,float step_high);
void Cubic_Bezier(float *t, float *angle_e, float *angle_i, float fai, float start_x, float start_z, float end_x, float end_z, float max_z,float Start_phase);
void Quadruped_gait(float *t, float (*angle)[2],int gait_state, float fai, float step_length,float step_high);
void Direct_Solution(float angle_e, float angle_i, float* x_pos, float* y_pos);
// 宏定义

//关于腿部的前进后退
#define forward 0
#define backward 1

//关于四足整体的前进后退左转右转
#define Advance 1
#define Retreat 2
#define Turn_left 3
#define Turn_right 4

extern float J60Motor_StandUpData_CAN1[4]; // lf_out, lf_in, rf_out, rf_in
extern float J60Motor_StandUpData_CAN2[4];     // rb_out, rb_in, lb_out, lb_in

extern float ReInit_can1[2];
extern float ReInit_can2[2];

extern float re_init_x[2];
extern float re_init_y[2];


#endif /* leg_control_H */