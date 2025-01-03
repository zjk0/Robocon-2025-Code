#ifndef LEG_CONTROL_H   // 头文件保护，防止重复包含
#define LEG_CONTROL_H

// 函数声明
void IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i);
void Leg_cyloid(float *t, float *angle_e,float *angle_i,int lag_flag);

// 宏定义


// 结构体定义


#endif /* leg_control_H */