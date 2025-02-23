#include "KinematicSolution.h" // 包含头文件
#include <stdio.h>             // 包含标准库
#include <math.h>
#include "DeepJ60_Motor.h"
#include "main.h"
#include "usart.h"

void IK_leg(float x_pos, float y_pos, float *angle_e, float *angle_i) {
    float l1 = 0.12, l2 = 0.24, l3 = 0.025;                  // 并联腿的长度
    float lod2, lbd2, loc2;                                  // 逆解的并联腿之外的一些长度
    float angle_link_i, angle_link_e;                        // 定义求解的两个电机转动角度
    float cosgama, cosCOD;                                   // 定义一些逆解需要角的cos值
    float angle_xOD, angle_COD, angle_xOC, bata; // 定义一些逆解需要的角

    lod2 = x_pos * x_pos + y_pos * y_pos;
    lbd2 = (l3 + l2) * (l3 + l2);
    cosgama = (l1 * l1 + lbd2 - lod2) / (2 * l1 * (l2 + l3));

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

void Direct_Solution(float angle_e, float angle_i, float *x_pos, float *y_pos) {
    float l1 = 0.09, l2 = 0.24, l3 = 0.025, loc; // 并联腿的长度�
    float angle_link_e, angle_obc;               // 定义求解的两个电机转动角度
    float cosangle_obc, cosBOC;
    float xb, yb, xdd, ydd, xd, yd; // 定义一些逆解需要角的cos值

    angle_link_e = angle_e;

    xb = -l1 * cos(angle_link_e);
    yb = l1 * sin(angle_link_e);

    cosBOC = cos((3.14159 - angle_e + angle_i) / 2.0f);

    loc = (l1 * cosBOC + sqrt(l1 * l1 * cosBOC * cosBOC - l1 * l1 + l2 * l2));

    cosangle_obc = (l1 * l1 + l2 * l2 - loc * loc) / (2 * l1 * l2);

    angle_obc = acos(cosangle_obc);

    xdd = (l2 + l3) * cos(angle_obc - angle_e);
    ydd = (l2 + l3) * sin(angle_obc - angle_e);

    xd = xdd + xb;
    yd = ydd + yb;

    *x_pos = xd;
    *y_pos = yd;
}
