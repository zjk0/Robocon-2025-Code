#include <math.h>
#include <stdio.h>
#include <foot_attitude_calculation.h>
#include <DeepJ60_Motor.h>
#define pi 3.1415926
#define l_1 0.09
#define l_2 0.29
#define l_3 0.025

point Foot_Position;
All_legs_place Robot_Legs_Position;

float arccosine(float l1,float l2,float l3){
	float cos=((l1*l1+l2*l2-l3*l3)/(2*l1*l2));
	return acos(cos);
}

float distance(point p1){	
	return sqrt(p1.X*p1.X+p1.Y*p1.Y);
}


point anglecalc(float alpha,float length){
	float x=length*cos(alpha);
	float y=length*sin(alpha);
	
	point result;
	result.X=x;
	result.Y=y;
	return result;
}

//计算D点坐标
//alpha和thet均为正数，输入角度即可
// void angle_to_position()
// {
	
// //	//按需求调节alpha和thet的输入正负
// //	float alpha=-1*a;
// //	float thet=1*b;
//   /*
// --------------------------------左前腿位置--------------------------------------
// */	        
//   	float alpha=-J60Motor_CAN1[0].ReceiveMotorData.CurrentPosition;
// 	float thet=J60Motor_CAN1[1].ReceiveMotorData.CurrentPosition;
	        
// 	alpha=alpha /180*pi;
// 	thet=thet /180*pi;
	
// 	float bet,C_angle,l_c;
// 	point B,C;
// 	bet=(pi/2+(alpha/2)-(thet/2));
// 	C_angle=alpha-bet;
// 	l_c=l_1*cos(bet)+sqrt(l_1*l_1*(cos(bet))*(cos(bet))+l_2*l_2);
	
// 	C.X=l_c*cos(C_angle);C.Y=l_c*sin(C_angle);
// 	B.X=l_1*cos(alpha);B.Y=l_1*sin(alpha);
	
// 	Robot_Legs_Position.FrontLeft.Y=(C.Y-B.Y)/l_2*(l_3+l_2)+B.Y;
// 	Robot_Legs_Position.FrontLeft.X=-((C.X-B.X)/l_2*(l_3+l_2)+B.X);
        

// /*
// --------------------------------右后腿位置--------------------------------------
// */	        
//         alpha=-J60Motor_CAN2[0].ReceiveMotorData.CurrentPosition;
// 	thet=J60Motor_CAN2[1].ReceiveMotorData.CurrentPosition;
        
// 	alpha=alpha /180*pi;
// 	thet=thet /180*pi;
	
// 	bet=(pi/2+(alpha/2)-(thet/2));
// 	C_angle=alpha-bet;
// 	l_c=l_1*cos(bet)+sqrt(l_1*l_1*(cos(bet))*(cos(bet))+l_2*l_2);
	
// 	C.X=l_c*cos(C_angle);C.Y=l_c*sin(C_angle);
// 	B.X=l_1*cos(alpha);B.Y=l_1*sin(alpha);
        
//         Robot_Legs_Position.RearRight.Y=(C.Y-B.Y)/l_2*(l_3+l_2)+B.Y;
// 	Robot_Legs_Position.RearRight.X=-((C.X-B.X)/l_2*(l_3+l_2)+B.X);
        
        
// /*
// --------------------------------右前腿位置--------------------------------------
// */	        
//   	alpha=-J60Motor_CAN3[0].ReceiveMotorData.CurrentPosition;
// 	thet=J60Motor_CAN3[1].ReceiveMotorData.CurrentPosition;
        
// 	alpha=alpha /180*pi;
// 	thet=thet /180*pi;
	
// 	bet=(pi/2+(alpha/2)-(thet/2));
// 	C_angle=alpha-bet;
// 	l_c=l_1*cos(bet)+sqrt(l_1*l_1*(cos(bet))*(cos(bet))+l_2*l_2);
	
// 	C.X=l_c*cos(C_angle);C.Y=l_c*sin(C_angle);
// 	B.X=l_1*cos(alpha);B.Y=l_1*sin(alpha);
        
//         Robot_Legs_Position.RearRight.Y=(C.Y-B.Y)/l_2*(l_3+l_2)+B.Y;
// 	Robot_Legs_Position.RearRight.X=-((C.X-B.X)/l_2*(l_3+l_2)+B.X);
        
        
        
// /*
// --------------------------------左后腿位置--------------------------------------
// */	        
//         alpha=-J60Motor_CAN4[0].ReceiveMotorData.CurrentPosition;
// 	thet=J60Motor_CAN4[1].ReceiveMotorData.CurrentPosition;
        
// 	alpha=alpha /180*pi;
// 	thet=thet /180*pi;
	
// 	bet=(pi/2+(alpha/2)-(thet/2));
// 	C_angle=alpha-bet;
// 	l_c=l_1*cos(bet)+sqrt(l_1*l_1*(cos(bet))*(cos(bet))+l_2*l_2);
	
// 	C.X=l_c*cos(C_angle);C.Y=l_c*sin(C_angle);
// 	B.X=l_1*cos(alpha);B.Y=l_1*sin(alpha);
        
//         Robot_Legs_Position.RearLeft.Y=(C.Y-B.Y)/l_2*(l_3+l_2)+B.Y;
// 	Robot_Legs_Position.RearLeft.X=-((C.X-B.X)/l_2*(l_3+l_2)+B.X);
// }



//逆解，输入点的坐标解出摆动角度，输出的两个角度均为正数
//本函数误差较大，手动修正之后控制在1°以内。但是当D离y轴太近的时候误差还是会很大（用到了反正atan2函数，逼近y轴时误差很大）
//仅能结算D在y轴以下的情况
//Angle position2angle(point D){
//	angle result;
//	
//	float dist=distance(D);
//	float alpha_D,gamma;
//	alpha_D=atan2(D.Y,D.X);
//	if(alpha_D>0){
//		alpha_D=alpha_D-pi;
//	}
//	gamma=arccosine(l_1,dist,l_2+l_3);
//	result.alpha=alpha_D + gamma;
//	
//	point B,C;
//	B=anglecalc(result.alpha,l_1);
//	C.X=(D.X-B.X)*l_2/(l_3+l_2)+B.X;
//	C.Y=(D.Y-B.Y)*l_2/(l_3+l_2)+B.Y;
//	
//	result.thet=2*atan2(C.Y,C.X)-result.alpha;
//	return result;
//}


//int main(){
//	float alpha;
//	float thet;
//	
//	angle d;
//	point D;
//	while(1){
//		
//		scanf("%f",&alpha);
//		scanf("%f",&thet);
//		D=angle2position(alpha,thet);
//		
//		d=position2angle(D);
//		printf("\nX=%f ",D.X);
//		printf("Y=%f",D.Y);
//		
//		printf("\nalpha=%f ",-1*d.alpha/pi*180-10);//存在误差，手动修正10°
//		printf(" thet=%f ",d.thet/pi*180+170);//存在误差，手动修正10°
//	}
//	
//
//	return 0;
//}
