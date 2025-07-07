#pragma once

typedef struct {
    float squat_l0;
    float jump_l0;
    float tilt_l0;
    float squat_l1;
    float jump_l1;
    float tilt_l1;
    float Kp;
    float Kd;
} JumpParam;

typedef struct {
    float length;
    float height;
    float Kp;
    float Kd;
} MotionParam;

extern JumpParam jump_wall_param;
extern JumpParam jump_stair_param;
extern JumpParam jump_breakbrigde_param;
extern JumpParam jump_first_stair_param;

extern MotionParam trot_param;
extern MotionParam rotate_param;
