#include "leg_control.h"  // 包含头文件
// #include <stdio.h>    // 包含标准库
#include <math.h>


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

void Leg_cyloid(float *t, float *angle_e,float *angle_i,int lag_flag)
{
  float Ts=1;   //周期 
  float fai=0.4;    //摆动相占空比
  float xs=-0.08;   //起点x位置
  float xf=0.08;  //终点x位置
  float zs=0.2457;   //z起点高度
  float h=0.04;   //抬腿高度
  float pi=3.14159;
  float xep,zep,sigma,sita;
  float tme=*t;
  int flag=0;


  if(*t>=Ts){
    *t=0;
  }

  if(lag_flag==0){
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

  if(lag_flag==1){
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

void Cubic_Bezier_Curve(float *t,float fhi,float high,float half_wide, float *angle_e,float *angle_i)
{
  float Ts=2;   //周期 
  float pi=3.14159;
  float xep,zep;
  float tme=(*t)+fhi;
  int flag=0;
  float zs=0.2457;

  float P3[2]={-1*half_wide,0},P2[2]={-0.5*half_wide,high},P1[2]={0.5*half_wide,high},P0[2]={half_wide,0};


  if((*t)>2) *t=0;

  // if(tme>1) 
  // {
  //   tme-=1; 
  //   flag=1;
  // }
  if((tme>=1&&tme<2)||tme>=3)
  {
    flag=1;
  }
  if(tme>=2&&tme<3){
    flag=0;
  }
  if(flag==0){
    
    while(tme>1) tme-=1;
    xep = pow((1-tme),3) * P0[0] + 3 * pow((1 - tme),2)*(tme)* P1[0] + 3 * (1 - tme) * pow((tme),2) * P2[0] + pow((tme),3) * P3[0];
    zep = zs - pow((1-tme),3) * P0[1] + 3 * pow((1 - tme),2)*(tme)* P1[1] + 3 * (1 - tme) * pow((tme),2) * P2[1] + pow((tme),3) * P3[1];
  }
  if(flag==1){
    if(tme>=1&&tme<2){
      tme=2-tme;
    }
    else tme=4-tme;
    xep = pow((1-tme),3) * P0[0] + 3 * pow((1 - tme),2)*tme* P1[0] + 3 * (1 - tme) * pow(tme,2) * P2[0] + pow(tme,3) * P3[0];
    zep =  zs;
  }
  //printf("%f,%f\n",xep,zep);
  IK_leg(xep,zep,angle_e,angle_i);

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