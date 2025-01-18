#include "leg_control.h" // 包含头文件
#include <stdio.h>       // 包含标准库
#include <math.h>
#include "DeepJ60_Motor.h"
#include "main.h"
#include "usart.h"

usart_data usart_motor_data;
float J60Motor_StandUpData_CAN1[4] = {0.647773742, -1.65866851, -0.340309143, 1.9198226922}; // lf_out, lf_in, rf_out, rf_in
float J60Motor_StandUpData_CAN2[4] = {2.20684433, -0.66669464111, -1.8807601922, 0.51845550533};     // rb_out, rb_in, lb_out, lb_in

void IK_leg(float x_pos, float y_pos, float *angle_e, float *angle_i)
{
    float l1 = 0.09, l2 = 0.24, l3 = 0.025;                  // 并联腿的长度
    float lod2, lbd2, loc2;                                  // 逆解的并联腿之外的一些长度
    float angle_link_i, angle_link_e;                        // 定义求解的两个电机转动角度
    float cosgama, cosCOD;                                   // 定义一些逆解需要角的cos值
    float angle_gama, angle_xOD, angle_COD, angle_xOC, bata; // 定义一些逆解需要的角

    lod2 = x_pos * x_pos + y_pos * y_pos;
    lbd2 = (l3 + l2) * (l3 + l2);
    cosgama = (l1 * l1 + lbd2 - lod2) / (2 * l1 * (l2 + l3));
    angle_gama = acos(cosgama);

    loc2 = l1 * l1 + l2 * l2 - 2 * l1 * l2 * cosgama;

    angle_xOD = acos(x_pos / sqrt(lod2));
    cosCOD = (lod2 + loc2 - l3 * l3) / (2 * sqrt(loc2) * sqrt(lod2));
    angle_COD = acos(cosCOD);

    angle_xOC = angle_xOD + angle_COD;

    bata = acos((l1 * l1 + loc2 - l2 * l2) / (2 * l1 * sqrt(loc2)));

    angle_link_e = bata - angle_xOC;
    angle_link_i = 3.14159 - angle_xOC - bata;

    *angle_e = round(angle_link_e * 1000) / 1000;
    *angle_i = round(angle_link_i * 1000) / 1000;
}

// void leg_cyloid(float *t,float fai, float *angle_e,float *angle_i)
// {
//   float Ts=1;   //周期
//   float fai=0.5;    //支撑相占空比
//   float xs=0;   //起点x位置
//   float xf=0.24;  //终点x位置
//   float zs=0.2457;   //z起点高度
//   float h=0.045;   //抬腿高度
//   float pi=3.14159;
//   float xep,zep,sigma;

//   if(*t>=Ts*fai){
//     *t=0;
//   }
//      printf("%f\n",*t);
//     sigma=2*pi*(*t)/(fai*Ts);
//         printf("%f\n",sigma);
//     xep=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs;

//     zep=zs-h/2*(1-cos(sigma));
//     printf("%f,%f\n",xep,zep);

//   IK_leg(xep,zep,angle_e,angle_i);

// }
/*
The lag_flag of lf_leg and the lag_flag of rb_leg are 0;
The lag_flag of rf_leg and the lag_flag of lb_leg are 1;
*/
void Leg_cyloid(float *t, float *angle_e, float *angle_i, int leg_flag, int direction, float fai, float step_length,float step_high)
{
  float Ts=1;   //周期 

  float zs=0.2457;   //z起点高度
  float h=step_high;   //抬腿高度
  float xs,xf;

  if(direction==forward){         //前进
    xs=-step_length/2.0;
    xf=step_length/2.0;
  }
  else if(direction==backward){   //后退
    xs=step_length/2.0;
    xf=-step_length/2.0;
  }

  float pi=3.14159;
  float xep,zep,sigma,sita;
  float tme=*t;
  int flag=0;


  if(*t>=Ts){
    *t=0;
  }

  if(leg_flag==0){
    if((*t)>=0&&(*t)<fai*Ts){
      sita=2*pi*(*t)/(fai*Ts);
      flag=0;
    }
    else if((*t)>=fai*Ts&&(*t)<Ts){
      sita=2*pi*(*t-fai*Ts)/(Ts-fai*Ts);
      flag=1;
    }

    if(flag==0){
      xep=(xf-xs)*(sita-sin(sita))/(2*pi)+xs;
      zep=zs-h/2*(1-cos(sita));
    }
    else if(flag==1){
      xep=-1*((xf-xs)*(sita-sin(sita))/(2*pi)+xs);
      zep=zs;
    }
  }

  if(leg_flag==1){
    if((*t)>=0&&(*t)<(Ts-fai*Ts)){
      sita=2*pi*(*t)/(Ts-fai*Ts);
      flag=0;
    }
    if((*t)>=(Ts-fai*Ts)&&(*t)<Ts){
      sita=2*pi*(*t+fai*Ts-Ts)/(fai*Ts);
      flag=1;
    }

    if(flag==1){
      xep=(xf-xs)*(sita-sin(sita))/(2*pi)+xs;
      zep=zs-h/2*(1-cos(sita));
    }
    if(flag==0){
      xep=-1*((xf-xs)*(sita-sin(sita))/(2*pi)+xs);
      zep=zs;
    }
  }
  // printf("%f,%f\n",xep,zep);
  IK_leg(xep,zep,angle_e,angle_i);
}

/**
  * @brief  利用贝塞尔曲线函数计算足端曲线轨迹
  * @param  
  * @param  
  * @retval 
  */
void Cubic_Bezier(float *t, float *angle_e, float *angle_i, float fai, float start_x, float start_z, float end_x, float end_z, float max_z,float Start_phase)
{
    float Ts = 1.0f; // 一个曲线的周期,总周期乘占空比
    float pi = 3.14159;
    float xep, zep;
    int flag = 0;
    float zs=0.2457;   //z起点高度

    float P3[2] = {end_x, end_z}, P2[2] = {end_x, max_z * 0.09f / 0.0675f}, P1[2] = {start_x, max_z * 0.09f / 0.0675f}, P0[2] = {start_x, start_z};
    float time=*t/Ts+Start_phase;
  // if(leg_flag==0){
    if (*t<=Ts && time<=Ts&&time>=0)
    {
        time=*t/Ts;
        xep = pow((1 - time), 3) * P0[0] +
              3.0f * pow((1 - time), 2) * (time)*P1[0] +
              3.0f * (1 - time) * pow((time), 2) * P2[0] +
              pow((time), 3) * P3[0];
        zep = zs-(pow((1 - time), 3) * P0[1] +
              3.0f * pow((1 - time), 2) * (time)*P1[1] +
              3.0f * (1 - time) * pow((time), 2) * P2[1] +
              pow((time), 3) * P3[1]);
    }

    IK_leg(xep, zep, angle_e, angle_i);
}

//---------------------------------------------------整体步态+摆线-----------------------------------------------------------
void Quadruped_gait(float *t, float (*angle)[2], int gait_state, float fai, float step_length, float step_high)
{

    if (gait_state == Advance)
    {
        Leg_cyloid(t, &angle[0][0], &angle[0][1], 0, forward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[1][0], &angle[1][1], 1, forward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[2][0], &angle[2][1], 0, forward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[3][0], &angle[3][1], 1, forward, fai, step_length, step_high);
    }
    else if (gait_state == Retreat)
    {
        Leg_cyloid(t, &angle[0][0], &angle[0][1], 0, backward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[1][0], &angle[1][1], 1, backward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[2][0], &angle[2][1], 0, backward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[3][0], &angle[3][1], 1, backward, fai, step_length, step_high);
    }
    else if (gait_state == Turn_left)
    {
        Leg_cyloid(t,&angle[0][0],&angle[0][1],0, backward, fai, step_length,step_high);
        Leg_cyloid(t,&angle[1][0],&angle[1][1],1, forward, fai, step_length,step_high);
        Leg_cyloid(t,&angle[2][0],&angle[2][1],0, forward, fai, step_length,step_high);
        Leg_cyloid(t,&angle[3][0],&angle[3][1],1, backward, fai, step_length,step_high);

    }
    else if (gait_state == Turn_right)
    {
        Leg_cyloid(t,&angle[0][0],&angle[0][1],0, forward, fai, step_length,step_high);
        Leg_cyloid(t,&angle[1][0],&angle[1][1],1, backward, fai, step_length,step_high);
        Leg_cyloid(t,&angle[2][0],&angle[2][1],0, backward, fai, step_length,step_high);
        Leg_cyloid(t,&angle[3][0],&angle[3][1],1, forward, fai, step_length,step_high);

    }
}
//-----------------------------------------------------直走+摆线-----------------------------------------------------------------
void Walk_straight_cyloid(float *t, float (*angle)[2], float fai, float step_length, float step_high, uint8_t direction_mode)
{
    if(direction_mode==ahead_ward){
        Leg_cyloid(t, &angle[0][0], &angle[0][1], 0, forward, fai, step_length, step_high);//左前腿
        Leg_cyloid(t, &angle[1][0], &angle[1][1], 1, forward, fai, step_length, step_high);//右前腿
        Leg_cyloid(t, &angle[2][0], &angle[2][1], 0, forward, fai, step_length, step_high);//右后腿
        Leg_cyloid(t, &angle[3][0], &angle[3][1], 1, forward, fai, step_length, step_high);//左后腿
    }
    if(direction_mode==back_ward){
        Leg_cyloid(t, &angle[0][0], &angle[0][1], 0, backward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[1][0], &angle[1][1], 1, backward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[2][0], &angle[2][1], 0, backward, fai, step_length, step_high);
        Leg_cyloid(t, &angle[3][0], &angle[3][1], 1, backward, fai, step_length, step_high);
    }
}

//-------------------------------------------------------整体步态+贝塞尔曲线----------------------------------------------------------
void Quadruped_gait_Bezier(float *t, float (*angle)[2], int gait_state, float fai, float start_x, float start_z, float end_x, float end_z, float max_z)
{
    if (gait_state == Advance)
    {
        Walk_straight_Bezier(t, angle, fai, start_x, start_z, end_x, end_z, max_z, ahead_ward);
    }
    else if (gait_state == Retreat)
    {
        Walk_straight_Bezier(t, angle, fai, start_x, start_z, end_x, end_z, max_z, back_ward);
    }
    else if (gait_state == Turn_left)
    {
        Walk_turn_Bezier(t, angle, fai, start_x, start_z, end_x, end_z, max_z, left);
    }
    else if (gait_state == Turn_right)
    {
        Walk_turn_Bezier(t, angle, fai, start_x, start_z, end_x, end_z, max_z, right);
    }
}


//---------------------------------------------------直走+贝塞尔曲线-------------------------------------------------------------
//leg_flag=0表示前半周期为摆动相，leg_flag=1表示前半周期为支撑相
void Walk_straight_Bezier(float *t, float (*angle)[2], float fai, float start_x, float start_z, float end_x, float end_z, float max_z, uint8_t direction_mode)
{
    if(direction_mode==ahead_ward){
        //LF_leg_ID1左前腿
        Cubic_Bezier(t, &angle[0][0], &angle[0][1], 0, fai, start_x, start_z, end_x, end_z, max_z);
        //RF_leg_ID2右前腿
        Cubic_Bezier(t, &angle[1][0], &angle[1][1], 1, fai, end_x, end_z, start_x, start_z, max_z);
        //RD_leg_ID3右后腿
        Cubic_Bezier(t, &angle[2][0], &angle[2][1], 0, fai, start_x, start_z, end_x, end_z, max_z);
        //LD_leg_ID4左后腿
        Cubic_Bezier(t, &angle[3][0], &angle[3][1], 1, fai, end_x, end_z, start_x, start_z, max_z);

    }
    if(direction_mode==back_ward){
        //LF_leg_ID1左前腿
        Cubic_Bezier(t, &angle[0][0], &angle[0][1], 0, fai, end_x, end_z, start_x, start_z, max_z);
        //RF_leg_ID2右前腿
        Cubic_Bezier(t, &angle[1][0], &angle[1][1], 1, fai, start_x, start_z, end_x, end_z, max_z);
        //RD_leg_ID3右后腿
        Cubic_Bezier(t, &angle[2][0], &angle[2][1], 0, fai, end_x, end_z, start_x, start_z, max_z);
        //LD_leg_ID4左后腿
        Cubic_Bezier(t, &angle[3][0], &angle[3][1], 1, fai, start_x, start_z, end_x, end_z, max_z);
    }
}

//----------------------------------------------------------转弯加贝塞尔曲线-------------------------------------------------------------------------
void Walk_turn_Bezier(float *t, float (*angle)[2], float fai, float start_x, float start_z, float end_x, float end_z, float max_z, uint8_t direction_mode)
{
    if(direction_mode == left ){
        //LF_leg_ID1左前腿
        Cubic_Bezier(t, &angle[0][0], &angle[0][1], 0, fai, end_x, end_z, start_x, start_z, max_z);
        //RF_leg_ID2右前腿
        Cubic_Bezier(t, &angle[1][0], &angle[1][1], 1, fai, end_x, end_z, start_x, start_z, max_z);
        //RD_leg_ID3右后腿
        Cubic_Bezier(t, &angle[2][0], &angle[2][1], 0, fai, start_x, start_z, end_x, end_z, max_z);
        //LD_leg_ID4左后腿
        Cubic_Bezier(t, &angle[3][0], &angle[3][1], 1, fai, start_x, start_z, end_x, end_z, max_z);
    }
    if(direction_mode == right ){
        //LF_leg_ID1左前腿
        Cubic_Bezier(t, &angle[0][0], &angle[0][1], 0, fai, start_x, start_z, end_x, end_z, max_z);
        //RF_leg_ID2右前腿
        Cubic_Bezier(t, &angle[1][0], &angle[1][1], 1, fai, start_x, start_z, end_x, end_z, max_z);
        //RD_leg_ID3右后腿
        Cubic_Bezier(t, &angle[2][0], &angle[2][1], 0, fai, end_x, end_z, start_x, start_z, max_z);
        //LD_leg_ID4左后腿
        Cubic_Bezier(t, &angle[3][0], &angle[3][1], 1, fai, end_x, end_z, start_x, start_z, max_z);
    }
}

float CubicSpline(float init_position, float goal_position, float init_velocity, float goal_velocity, float now_time, float total_time) {
    float a, b, c, d;
    a = (goal_velocity * total_time + init_velocity * total_time - 2 * goal_position + 2 * init_position) / pow(total_time, 3);
    b = (3 * goal_position - 3 * init_position - 2 * init_velocity * total_time - goal_velocity * total_time) / pow(total_time, 2);
    c = init_velocity;
    d = init_position;

    float now_position = a * pow(now_time, 3) + b * pow(now_time, 2) + c * now_time + d;

    return now_position;
}


void Jump(float squat_length, float up_length) {
    float original_position = 0.2457;
    float angle_in = 0;
    float angle_out = 0;

    // Squat
    for (float i = 0; i <= squat_length; i += (squat_length / 10)) {
        IK_leg(0, original_position - i, &angle_in, &angle_out);

        usart_motor_data.real_motor_data[0] = angle_in;
        usart_motor_data.real_motor_data[1] = angle_out;
        usart_motor_data.real_motor_data[2] = angle_in;
        usart_motor_data.real_motor_data[3] = angle_out;

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
    }

    // Jump
    IK_leg(0, original_position + up_length, &angle_out, &angle_in);

    usart_motor_data.real_motor_data[0] = angle_in;
    usart_motor_data.real_motor_data[1] = angle_out;
    usart_motor_data.real_motor_data[2] = angle_in;
    usart_motor_data.real_motor_data[3] = angle_out;

    HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

    RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle_out, 0, 0, 100, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle_in, 0, 0, 100, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle_out, 0, 0, 100, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle_in, 0, 0, 100, 5, PositionMode);
    HAL_Delay(500);

    // Reset legs
    usart_motor_data.real_motor_data[0] = 0;
    usart_motor_data.real_motor_data[1] = 0;
    usart_motor_data.real_motor_data[2] = 0;
    usart_motor_data.real_motor_data[3] = 0;

    HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

    RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);

    // Waiting for landing
    float torque = J60Motor_CAN1[0].ReceiveMotorData.CurrentTorque;
    while (J60Motor_CAN1[0].ReceiveMotorData.CurrentTorque - torque < 0.001);

    // Cushioning
    for (float i = 0; i <= squat_length; i += squat_length / 10) {
        IK_leg(0, original_position - i, &angle_in, &angle_out);

        usart_motor_data.real_motor_data[0] = angle_in;
        usart_motor_data.real_motor_data[1] = angle_out;
        usart_motor_data.real_motor_data[2] = angle_in;
        usart_motor_data.real_motor_data[3] = angle_out;

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
    }

    HAL_Delay(500);

    // Reset legs
    usart_motor_data.real_motor_data[0] = 0;
    usart_motor_data.real_motor_data[1] = 0;
    usart_motor_data.real_motor_data[2] = 0;
    usart_motor_data.real_motor_data[3] = 0;

    HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

    RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
    RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1], 0, 0, 50, 5, PositionMode);
    HAL_Delay(1);
}

void JumpForward (float squat_length, float up_length, float lean_length) {
    float original_position = 0.2457;
    float angle_in = 0;
    float angle_out = 0;

    // Squat
    for (float i = 0; i <= squat_length; i += (squat_length / 10)) {
        IK_leg(0, original_position - i, &angle_in, &angle_out);

        usart_motor_data.real_motor_data[0] = angle_in;
        usart_motor_data.real_motor_data[1] = angle_out;
        usart_motor_data.real_motor_data[2] = angle_in;
        usart_motor_data.real_motor_data[3] = angle_out;

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
    }

    // Lean forward
    for (float i = 0; i <= lean_length; i += (lean_length / 10)) {
        IK_leg(i, original_position - squat_length, &angle_in, &angle_out);

        usart_motor_data.real_motor_data[0] = angle_in;
        usart_motor_data.real_motor_data[1] = angle_out;
        usart_motor_data.real_motor_data[2] = angle_in;
        usart_motor_data.real_motor_data[3] = angle_out;

        HAL_UART_Transmit(&huart6, usart_motor_data.send_motor_data, 16, 1000);
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) != SET);

        RunJ60Motor(&J60Motor_CAN1[0], J60Motor_StandUpData_CAN1[0] - angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN1[1], J60Motor_StandUpData_CAN1[1] + angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[0], J60Motor_StandUpData_CAN2[0] + angle_out, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
        RunJ60Motor(&J60Motor_CAN2[1], J60Motor_StandUpData_CAN2[1] - angle_in, 0, 0, 50, 5, PositionMode);
        HAL_Delay(1);
    }


}

void Direct_Solution(float angle_e, float angle_i, float* x_pos, float* y_pos)
{
  float l1 = 0.09, l2 = 0.24, l3 = 0.025,loc;                  // 并联腿的长度
  float lod2, lbd2, loc2;                                  // 逆解的并联腿之外的一些长度
  float angle_link_i, angle_link_e,angle_obc;                        // 定义求解的两个电机转动角度
  float cosangle_obc,cosBOC;
  float xb,yb,xdd,ydd,xc,yc,xd,yd;                                   // 定义一些逆解需要角的cos值
  // float angle_gama, angle_xOD, angle_COD, angle_xOC, bata; // 定义一些逆解需要的角
  float link1_1_angle;

  angle_link_e=angle_e;

  xb=-l1*cos(angle_link_e);
  yb=l1*sin(angle_link_e);

  cosBOC=cos((3.14159-angle_e+angle_i)/2.0f);


  loc = (l1 * cosBOC + sqrt(l1 * l1 * cosBOC * cosBOC - l1 * l1 + l2 * l2));

  cosangle_obc=(l1*l1+l2*l2-loc*loc)/(2*l1*l2);

  angle_obc=acos(cosangle_obc);

  xdd=(l2+l3)*cos(angle_obc-angle_e);
  ydd=(l2+l3)*sin(angle_obc-angle_e);
  
  xd=xdd+xb;
  yd=ydd+yb;

  *x_pos=xd;
  *y_pos=yd;

}
