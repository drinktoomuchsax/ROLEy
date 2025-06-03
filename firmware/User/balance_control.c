#include "balance_control.h"
#include "wheeltec_driver.h"

// 定义控制状态
balance_state_t balance_state = {0};

// 初始化平衡控制
void balance_init(void)
{
    // 初始化角度PID控制器
    balance_state.angle_pid.Kp = 10.0f;
    balance_state.angle_pid.Ki = 0.0f;
    balance_state.angle_pid.Kd = 0.1f; // sbwanyi
    balance_state.angle_pid.tau = 0.02f;  // 20ms低通滤波
    balance_state.angle_pid.limMin = -40.0f;
    balance_state.angle_pid.limMax = 40.0f;
    balance_state.angle_pid.limMinInt = -20.0f;
    balance_state.angle_pid.limMaxInt = 20.0f;
    balance_state.angle_pid.T = 0.01f;    // 10ms采样时间
    PIDController_Init(&balance_state.angle_pid);

    // 初始化速度PID控制器
    balance_state.speed_pid.Kp = 0.0f;
    balance_state.speed_pid.Ki = 0.0f;
    balance_state.speed_pid.Kd = 0.0f;
    balance_state.speed_pid.tau = 0.02f;
    balance_state.speed_pid.limMin = -30.0f;
    balance_state.speed_pid.limMax = 30.0f;
    balance_state.speed_pid.limMinInt = -15.0f;
    balance_state.speed_pid.limMaxInt = 15.0f;
    balance_state.speed_pid.T = 0.01f;
    PIDController_Init(&balance_state.speed_pid);

}

// 更新平衡控制
void balance_update(float angle, float speed)
{
    float angle_output;
    float speed_output;
    float final_output;
    
    // 更新状态
    balance_state.current_angle = angle;
    balance_state.current_speed = speed;
    
    // 角度环PID控制
    angle_output = PIDController_Update(&balance_state.angle_pid, 0.0f, angle);
    
    // 速度环PID控制（使用角速度作为反馈）
    speed_output = PIDController_Update(&balance_state.speed_pid, 0.0f, speed);
    
    // 合并两个控制器的输出
    final_output = angle_output + speed_output;
    
    // 限制最终输出范围
    if(final_output > 100.0f) final_output = 100.0f;
    else if(final_output < -100.0f) final_output = -100.0f;
    
    // 更新电机输出
    if(final_output > 0)
    {
        WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CCW, (uint8_t)final_output);
        WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CCW, (uint8_t)final_output);
    }
    else
    {
        WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CW, (uint8_t)-final_output);
        WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CW, (uint8_t)-final_output);
    }
}

// 设置PID参数
void balance_set_params(float angle_kp, float angle_ki, float angle_kd,
                       float speed_kp, float speed_ki, float speed_kd)
{
    balance_state.angle_pid.Kp = angle_kp;
    balance_state.angle_pid.Ki = angle_ki;
    balance_state.angle_pid.Kd = angle_kd;
    
    balance_state.speed_pid.Kp = speed_kp;
    balance_state.speed_pid.Ki = speed_ki;
    balance_state.speed_pid.Kd = speed_kd;
}

// 重置控制状态
void balance_reset(void)
{
    PIDController_Init(&balance_state.angle_pid);
    PIDController_Init(&balance_state.speed_pid);
    WheelTec_StopAll();
}