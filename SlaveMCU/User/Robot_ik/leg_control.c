#include "leg_control.h"  // 包含头文件
#include <stdio.h>    // 包含标准库
#include <math.h>

float J60Motor_StandUpData_CAN1[4] = {0.48954, -2.214015,  -0.43628692622, 1.6576004022};  // lf_out, lf_in, rf_out, rf_in
float J60Motor_StandUpData_CAN2[4] = {2.3810234, -0.597496, -2.2713127133, 0.110664367};  // rb_out, rb_in, lb_out, lb_in

float ReInit_can1[2] = {0};
float ReInit_can2[2] = {0};

float re_init_x[2] = {0};
float re_init_y[2] = {0};

void IK_leg(float x_pos,float y_pos,float *angle_e,float *angle_i)
{
  float l1=0.09,l2=0.24,l3=0.025;      //并联腿的长度
  float lod2,lbd2,loc2;               //逆解的并联腿之外的一些长度
  float angle_link_i,angle_link_e;    //定义求解的两个电机转动角度
  float cosgama,cosCOD;     //定义一些逆解需要角的cos值
  float angle_gama,angle_xOD,angle_COD,angle_xOC,bata;  //定义一些逆解需要的角
  
  lod2=x_pos*x_pos+y_pos*y_pos;
  lbd2=(l3+l2)*(l3+l2);
  cosgama=(l1*l1+lbd2-lod2)/(2*l1*(l2+l3));
  angle_gama=acos(cosgama);

  loc2=l1*l1+l2*l2-2*l1*l2*cosgama;

  angle_xOD=acos(x_pos/sqrt(lod2));
  cosCOD=(lod2+loc2-l3*l3)/(2*sqrt(loc2)*sqrt(lod2));
  angle_COD=acos(cosCOD);

  angle_xOC=angle_xOD+angle_COD;

  bata=acos((l1*l1+loc2-l2*l2)/(2*l1*sqrt(loc2)));

  angle_link_e=bata-angle_xOC;
  angle_link_i=3.14159-angle_xOC-bata;

  *angle_e=round(angle_link_e*1000)/1000;
  *angle_i=round(angle_link_i*1000)/1000;

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

  // float fai=0.3;    //摆动相占空比
  // float xs=-0.08;   //起点x位置
  // float xf=0.08;  //终点x位置
  // float zs=0.2457;   //z起点高度
  // float h=0.04;   //抬腿高度


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


//初始相
/*
    fhi_lf和fhi_rb=3.14
    fhi_rf和fhi_lb=9.423
*/

// void dog_leg_trot()
// {
//     float t=0;
//     float fai=0.5;
//     float sita=0;
//     sita=2*pi*(*t)/(fai*Ts);
    
//     if(t<=Ts*fai){
//         sigma=2*pi*(*t)/(fai*Ts);
//         xep=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs;
//         zep=zs-h/2*(1-cos(sigma));
//     }
//     if(t>Ts*fai&&t<Ts)
//     {
//         sigma=(-1)*2*pi*(*t)/(fai*Ts);
//         xep=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xs;
//     }


// }
void Quadruped_gait(float *t, float (*angle)[2],int gait_state, float fai, float step_length,float step_high)
{
  if(gait_state==Advance){
    printf("lf:\n");
    Leg_cyloid(t,&angle[0][0],&angle[0][1],0, forward, fai, step_length,step_high);
    printf("rf:\n");
    Leg_cyloid(t,&angle[1][0],&angle[1][1],1, forward, fai, step_length,step_high);
    printf("rb:\n");
    Leg_cyloid(t,&angle[2][0],&angle[2][1],0, forward, fai, step_length,step_high);
    printf("lb:\n");
    Leg_cyloid(t,&angle[3][0],&angle[3][1],1, forward, fai, step_length,step_high);
  }
  else if(gait_state==Retreat){
    printf("lf:\n");
    Leg_cyloid(t,&angle[0][0],&angle[0][1],0, backward, fai, step_length,step_high);
    printf("rf:\n");
    Leg_cyloid(t,&angle[1][0],&angle[1][1],1, backward, fai, step_length,step_high);
    printf("rb:\n");
    Leg_cyloid(t,&angle[2][0],&angle[2][1],0, backward, fai, step_length,step_high);
    printf("lb:\n");
    Leg_cyloid(t,&angle[3][0],&angle[3][1],1, backward, fai, step_length,step_high);
  }
  else if(gait_state==Turn_left){
    // printf("lf:\n");
    // Leg_cyloid(t,&angle[0][0],&angle[0][1],0, backward, fai, step_length,step_high);
    // printf("rf:\n");
    // Leg_cyloid(t,&angle[1][0],&angle[1][1],1, forward, fai, step_length,step_high);
    // printf("rb:\n");
    // Leg_cyloid(t,&angle[2][0],&angle[2][1],0, forward, fai, step_length,step_high);
    // printf("lb:\n");
    // Leg_cyloid(t,&angle[3][0],&angle[3][1],1, backward, fai, step_length,step_high);
    printf("lf:\n");
    Leg_cyloid(t,&angle[0][0],&angle[0][1],0, backward, fai, step_length,step_high);
    printf("rf:\n");
    Leg_cyloid(t,&angle[1][0],&angle[1][1],0, forward, fai, step_length,step_high);
    printf("rb:\n");
    Leg_cyloid(t,&angle[2][0],&angle[2][1],0, forward, fai, step_length,step_high);
    printf("lb:\n");
    Leg_cyloid(t,&angle[3][0],&angle[3][1],0, backward, fai, step_length,step_high);
  }
  else if(gait_state==Turn_right){
    // printf("lf:\n");
    // Leg_cyloid(t,&angle[0][0],&angle[0][1],0, forward, fai, step_length,step_high);
    // printf("rf:\n");
    // Leg_cyloid(t,&angle[1][0],&angle[1][1],1, backward, fai, step_length,step_high);
    // printf("rb:\n");
    // Leg_cyloid(t,&angle[2][0],&angle[2][1],0, backward, fai, step_length,step_high);
    // printf("lb:\n");
    // Leg_cyloid(t,&angle[3][0],&angle[3][1],1, forward, fai, step_length,step_high);
    printf("lf:\n");
    Leg_cyloid(t,&angle[0][0],&angle[0][1],0, forward, fai, step_length,step_high);
    printf("rf:\n");
    Leg_cyloid(t,&angle[1][0],&angle[1][1],0, backward, fai, step_length,step_high);
    printf("rb:\n");
    Leg_cyloid(t,&angle[2][0],&angle[2][1],0, backward, fai, step_length,step_high);
    printf("lb:\n");
    Leg_cyloid(t,&angle[3][0],&angle[3][1],0, forward, fai, step_length,step_high);
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