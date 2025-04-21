#ifndef __WHEELTEC_DRIVER_H__
#define __WHEELTEC_DRIVER_H__

#include "main.h"
#include "tim.h"
#include <stdbool.h>

// 电机方向枚举
typedef enum {
    MOTOR_DIRECTION_BRAKE = 0,  // 刹车
    MOTOR_DIRECTION_CW,        // 顺时针
    MOTOR_DIRECTION_CCW,       // 逆时针
} motor_direction_t;

// 电机通道枚举
typedef enum {
    MOTOR_CHANNEL_1 = 1,  // 电机通道1
    MOTOR_CHANNEL_2 = 2   // 电机通道2
} motor_channel_t;

// 函数声明
void WheelTec_Init(void);
void WheelTec_SetSpeed(motor_channel_t channel, uint8_t speed);
void WheelTec_SetDirection(motor_channel_t channel, motor_direction_t direction);
void WheelTec_Control(motor_channel_t channel, motor_direction_t direction, uint8_t speed);
void WheelTec_Stop(motor_channel_t channel);
void WheelTec_StopAll(void);

#endif /* __WHEELTEC_DRIVER_H__ */
