#include "roley_neck.h"

/**
 * @brief 初始化颈部的两个电机，并开启定时器2的中断，1ms给电机发送一次CAN消息
 * 
 * @return true 
 * @return false 
 */
bool neck_motor_init(void)
{
    // 发送0x666的CAN消息，方便调试
    uint8_t zero_data[8] = {0,0,0,0,0,0,0,0};
    fdcanx_send_data(&hfdcan1, 0x666, zero_data, 8);


    HAL_Delay(100);
    /* 往电机的寄存器里面写入位置速度模式 */
    write_motor_data(motor[Motor1].id, 10, pos_mode, 0, 0, 0);    // 修改电机为位置速度模式
    HAL_Delay(100);
    save_motor_data(motor[Motor1].id, 10);        // 保存电机参数
    HAL_Delay(1000);
    dm_motor_enable(&hfdcan1, &motor[Motor1]);    // 使能电机
    HAL_Delay(100);

    write_motor_data(motor[Motor2].id, 10, pos_mode, 0, 0, 0);    // 修改电机为位置速度模式
    HAL_Delay(100);
    save_motor_data(motor[Motor2].id, 10);        // 保存电机参数
    HAL_Delay(100);
    dm_motor_enable(&hfdcan1, &motor[Motor2]);    // 使能电机
    HAL_Delay(1000);



    return true;
}

/**
 * @brief 更新颈部的两个电机
 * 
 */
void neck_motor_update(void)
{
    dm_motor_ctrl_send(&hfdcan1, &neck_yaw_motor);
    dm_motor_ctrl_send(&hfdcan1, &neck_pitch_motor);
}

/**
 * @brief 旋转ROLEy的yaw电机, 角度单位为rad，可以左右限位，如果反馈值超出限位则返回false
 * 
 * @param angle 输入角度
 * @param left_limit 左限位
 * @param right_limit 右限位
 * @return true 
 * @return false 
 */
bool neck_rotate_yaw(float angle, float left_limit, float right_limit)
{
    if(angle < left_limit || angle > right_limit)
    {
        return false;
    }
    neck_yaw_motor.ctrl.pos_set = angle;
    return true;
}

/**
 * @brief 旋转ROLEy的pitch电机, 角度单位为rad，可以上下限位，如果反馈值超出限位则返回false
 * 
 * @param angle 输入角度
 * @param lower_limit 下限位
 * @param upper_limit 上限位
 * @return true 
 * @return false 
 */
bool neck_rotate_pitch(float angle, float lower_limit, float upper_limit)
{
    if(angle < lower_limit || angle > upper_limit)
    {
        return false;
    }
    neck_pitch_motor.ctrl.pos_set = angle;
    return true;
}

/**
 * @brief 重置颈部的两个电机到0位置，正视前方
 * 
 */
void reset_neck_motor(void)
{
    neck_yaw_motor.ctrl.pos_set = 0;
    neck_pitch_motor.ctrl.pos_set = 0;
}