#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include "main.h"
#include "PID.h"

// 平衡控制状态结构体
typedef struct {
    float current_angle;     // 当前角度
    float current_speed;      // 当前角速度
    PIDController angle_pid; // 角度PID控制器
    PIDController speed_pid; // 速度PID控制器
} balance_state_t;

// 函数声明
/**
 * @brief 初始化平衡控制
 */ 
void balance_init(void);

/**
 * @brief 更新平衡控制
 */
void balance_update(float angle, float gyro);
    
/**
 * @brief 设置平衡控制参数
 */
void balance_set_params(float angle_kp, float angle_ki, float angle_kd,
                       float speed_kp, float speed_ki, float speed_kd);

/**
 * @brief 重置平衡控制
 */
void balance_reset(void);

#endif /* __BALANCE_CONTROL_H */ 