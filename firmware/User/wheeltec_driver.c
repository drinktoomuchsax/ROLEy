#include "wheeltec_driver.h"
#include <math.h>
#include <stdlib.h>

// PWM定时器的分辨率（TIM4的Period值）
#define PWM_MAX_VALUE 2525
// PWM变化的最大步长（每次调用的最大变化量）
#define PWM_MAX_STEP 100

// 记录当前PWM值
static uint32_t current_pwm_value[3] = {0, 0, 0}; // 索引0不使用，1和2对应两个通道

// 平滑设置PWM值的函数，防止突变
static void WheelTec_SetPWM_Smooth(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t target_value) 
{
    uint8_t motor_idx = 0;
    
    // 确定对应的电机索引
    if(channel == TIM_CHANNEL_3) {
        motor_idx = 1;
    } else if(channel == TIM_CHANNEL_4) {
        motor_idx = 2;
    } else {
        return; // 无效通道
    }
    
    // 如果目标值与当前值相差不大，直接设置
    if(abs((int32_t)target_value - (int32_t)current_pwm_value[motor_idx]) <= PWM_MAX_STEP) {
        current_pwm_value[motor_idx] = target_value;
        __HAL_TIM_SET_COMPARE(htim, channel, target_value);
        return;
    }
    
    // 逐步调整PWM值
    if(target_value > current_pwm_value[motor_idx]) {
        current_pwm_value[motor_idx] += PWM_MAX_STEP;
    } else {
        current_pwm_value[motor_idx] -= PWM_MAX_STEP;
    }
    
    __HAL_TIM_SET_COMPARE(htim, channel, current_pwm_value[motor_idx]);
}

// 电机初始化
void WheelTec_Init(void)
{
    // 启动PWM定时器
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 电机1的PWM通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 电机2的PWM通道
    // 然后确保pwm为占空比为0，电机不会转
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    
    // 初始化PWM当前值
    current_pwm_value[1] = 0;
    current_pwm_value[2] = 0;
    
    // 初始化时停止所有电机
    WheelTec_StopAll();
}

// 设置电机速度（0-100%）- 修改为使用平滑PWM设置
void WheelTec_SetSpeed(motor_channel_t channel, uint8_t speed)
{
    // 限制速度范围为0-100
    if(speed > 100) speed = 100;
    
    // 将百分比转换为PWM值
    uint32_t pwm_value = (speed * PWM_MAX_VALUE) / 100;
    
    // 平滑设置PWM通道
    if(channel == MOTOR_CHANNEL_1) {
        WheelTec_SetPWM_Smooth(&htim4, TIM_CHANNEL_3, pwm_value);
    } else if(channel == MOTOR_CHANNEL_2) {
        WheelTec_SetPWM_Smooth(&htim4, TIM_CHANNEL_4, pwm_value);
    }
}

// 设置电机方向
void WheelTec_SetDirection(motor_channel_t channel, motor_direction_t direction)
{
    if(channel == MOTOR_CHANNEL_1) {
        switch(direction) {
            case MOTOR_DIRECTION_BRAKE:
                HAL_GPIO_WritePin(MOTOR_IN_A1_GPIO_Port, MOTOR_IN_A1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_IN_B1_GPIO_Port, MOTOR_IN_B1_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_DIRECTION_CW:
                HAL_GPIO_WritePin(MOTOR_IN_A1_GPIO_Port, MOTOR_IN_A1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_IN_B1_GPIO_Port, MOTOR_IN_B1_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_DIRECTION_CCW:
                HAL_GPIO_WritePin(MOTOR_IN_A1_GPIO_Port, MOTOR_IN_A1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_IN_B1_GPIO_Port, MOTOR_IN_B1_Pin, GPIO_PIN_SET);
                break;
        }
    } else if(channel == MOTOR_CHANNEL_2) {
        switch(direction) {
            case MOTOR_DIRECTION_BRAKE:
                HAL_GPIO_WritePin(MOTOR_IN_A2_GPIO_Port, MOTOR_IN_A2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_IN_B2_GPIO_Port, MOTOR_IN_B2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_DIRECTION_CW:
                HAL_GPIO_WritePin(MOTOR_IN_A2_GPIO_Port, MOTOR_IN_A2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_IN_B2_GPIO_Port, MOTOR_IN_B2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_DIRECTION_CCW:
                HAL_GPIO_WritePin(MOTOR_IN_A2_GPIO_Port, MOTOR_IN_A2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_IN_B2_GPIO_Port, MOTOR_IN_B2_Pin, GPIO_PIN_SET);
                break;
        }
    }
}

// 综合控制电机方向和速度
void WheelTec_Control(motor_channel_t channel, motor_direction_t direction, uint8_t speed)
{
    // 先设置方向
    WheelTec_SetDirection(channel, direction);
    
    // 如果是停止或刹车模式，速度设为0
    if(direction == MOTOR_DIRECTION_BRAKE) {
        WheelTec_SetSpeed(channel, 0);
    } else {
        // 设置速度
        WheelTec_SetSpeed(channel, speed);
    }
}

// 停止指定电机
void WheelTec_Stop(motor_channel_t channel)
{
    WheelTec_Control(channel, MOTOR_DIRECTION_BRAKE, 0);
}

// 停止所有电机
void WheelTec_StopAll(void)
{
    HAL_GPIO_WritePin(MOTOR_IN_A1_GPIO_Port, MOTOR_IN_A1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN_B1_GPIO_Port, MOTOR_IN_B1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN_A2_GPIO_Port, MOTOR_IN_A2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN_B2_GPIO_Port, MOTOR_IN_B2_Pin, GPIO_PIN_RESET);
}
