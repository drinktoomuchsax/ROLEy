/**
 * @file ROLEY_neck.h
 * @author ROLEY
 * @brief 对达妙电机再进行一次封装，直接暴露出pitch和yaw的控制接口
 * @version 0.1
 * @date 2025-05-28
 * 
 */
#ifndef __ROLEY_NECK_H
#define __ROLEY_NECK_H

#include "dm_motor_ctrl.h"
#include <stdbool.h>

#define neck_yaw_motor motor[Motor1]
#define neck_pitch_motor motor[Motor2]

bool neck_motor_init(void);
void neck_motor_update(void);
bool neck_rotate_yaw(float angle, float left_limit, float right_limit);
bool neck_rotate_pitch(float angle, float lower_limit, float upper_limit);
void reset_neck_motor(void);


#endif