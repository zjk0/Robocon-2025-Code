#include "param.h"

JumpParam jump_wall_param = {
    .squat_l0 = 0.08, 
    .squat_l1 = 0.08, 
    .jump_l0 = 0.15, 
    .jump_l1 = 0.15,
    .tilt_l0 = 0.07,
    .tilt_l1 = 0.07,
    .Kp = 450,
    .Kd = 4
};

JumpParam jump_stair_param = {
    .squat_l0 = 0.02, 
    .squat_l1 = 0.04, 
    .jump_l0 = 0.07, 
    .jump_l1 = 0.09,
    .tilt_l0 = 0.1,
    .tilt_l1 = 0.14,
    .Kp = 150,
    .Kd = 4
};

JumpParam jump_breakbrigde_param = {
    .squat_l0 = 0.08, 
    .squat_l1 = 0.08, 
    .jump_l0 = 0.13, 
    .jump_l1 = 0.13,
    .tilt_l0 = 0.05,
    .tilt_l1 = 0.05,
    .Kp = 450,
    .Kd = 4
};

JumpParam jump_first_stair_param = {
    .squat_l0 = 0.08, 
    .squat_l1 = 0.08, 
    .jump_l0 = 0.15, 
    .jump_l1 = 0.15,
    .tilt_l0 = 0.05,
    .tilt_l1 = 0.05,
    .Kp = 450,
    .Kd = 4
};

MotionParam trot_param = {
    .length = 0.2,
    .height = 0.03,
    .Kp = 50,
    .Kd = 4
};

MotionParam rotate_param = {
    .length = 0.1,
    .height = 0.03,
    .Kp = 100,
    .Kd = 4
};