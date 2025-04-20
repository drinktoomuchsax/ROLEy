#include "uart_bsp.h"
#include "string.h"
#include "usart.h"
#include "stdint.h"
#include <math.h>  // 添加数学库，用于角度计算

/* 私有变量 - 将rx_buff改为非静态以供外部访问 */
uint8_t rx_buff[UART_BUFF_SIZE];
static remoter_t remoter;

/* 私有辅助函数 */
static float MapRange(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  初始化UART接收处理
 * @param  无
 * @retval 无
 */
void UART_BSP_Init(void)
{
    /* 清空接收缓冲区 */
    memset(rx_buff, 0, sizeof(rx_buff));
    
    /* 清空遥控器数据结构 */
    memset(&remoter, 0, sizeof(remoter_t));
    
    /* 启动UART接收 */
    UART_BSP_StartReceive();
}

/**
 * @brief  启动UART接收
 * @param  无
 * @retval 无
 */
void UART_BSP_StartReceive(void)
{
    /* 使用DMA方式接收，接收到IDLE事件时会触发回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, UART_BUFF_SIZE*2);
}

/**
 * @brief  获取遥控器数据的引用
 * @param  无
 * @retval 指向遥控器数据结构体的指针
 */
remoter_t* UART_BSP_GetRemoterData(void)
{
    return &remoter;
}

/**
 * @brief  获取接收缓冲区的指针（兼容旧代码）
 * @param  无
 * @retval 指向接收缓冲区的指针
 */
uint8_t* UART_BSP_GetRxBuffer(void)
{
    return rx_buff;
}

/**
 * @brief  解析SBUS数据帧
 * @param  buf: 接收到的数据缓冲区
 * @retval 无
 */
void UART_BSP_ParseSbusFrame(uint8_t *buf)
{
    /* 检查帧头和帧尾是否正确 */
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;

    /* 检查连接状态 */
    if (buf[23] == SBUS_OFFLINE_FLAG)
        remoter.online = 0;
    else
        remoter.online = 1;

    /* 解析各通道数据 */
    remoter.rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    remoter.rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    remoter.rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    remoter.rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    remoter.rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter.rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter.rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter.rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter.rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    
    /* 在这里添加对遥控器摇杆和按键的映射处理 */
    remoter.joy.right_hori = remoter.rc.ch[0]; // 右摇杆水平ch0
    remoter.joy.right_vert = remoter.rc.ch[1]; // 右摇杆垂直ch1
    remoter.joy.left_vert = remoter.rc.ch[2]; // 左摇杆垂直ch2
    remoter.joy.left_hori = remoter.rc.ch[3]; // 左摇杆水平ch3

	remoter.joy_percent.left_vert = MapRange(remoter.joy.left_vert, REMOTE_CHANNEL_BOTTOM, REMOTE_CHANNEL_TOP, -100, 100); // 左摇杆垂直, -100~100%
	remoter.joy_percent.left_hori = MapRange(remoter.joy.left_hori, REMOTE_CHANNEL_BOTTOM, REMOTE_CHANNEL_TOP, -100, 100); // 左摇杆水平, -100~100%
	remoter.joy_percent.right_vert = MapRange(remoter.joy.right_vert, REMOTE_CHANNEL_BOTTOM, REMOTE_CHANNEL_TOP, -100, 100); // 右摇杆垂直, -100~100%
	remoter.joy_percent.right_hori = MapRange(remoter.joy.right_hori, REMOTE_CHANNEL_BOTTOM, REMOTE_CHANNEL_TOP, -100, 100); // 右摇杆水平, -100~100%

	remoter.joy_direction.left_angle = UART_BSP_GetStickAngle(LEFT_STICK); // 左摇杆方向角度
	remoter.joy_direction.right_angle = UART_BSP_GetStickAngle(RIGHT_STICK); // 右摇杆方向角度

	remoter.joy_magnitude.left_magnitude = UART_BSP_GetStickMagnitude(LEFT_STICK); // 左摇杆向量长度, 0~100%
	remoter.joy_magnitude.right_magnitude = UART_BSP_GetStickMagnitude(RIGHT_STICK); // 右摇杆向量长度, 0~100%

	remoter.key.left_shuolder = (key_state_t)remoter.rc.ch[4]; // 左肩键ch4
	remoter.key.left_face = (key_state_t)remoter.rc.ch[5]; // 左脸键ch5
	remoter.key.right_face = (key_state_t)remoter.rc.ch[6]; // 右脸键ch6
	remoter.key.right_shoulder = (key_state_t)remoter.rc.ch[7]; // 右肩键ch7

	remoter.rssi = remoter.rc.ch[8]; // rssi
}

/**
 * @brief  UART接收完成回调函数
 * @param  huart: UART句柄
 * @param  Size: 接收到的数据大小
 * @retval 无
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        if (Size <= UART_BUFF_SIZE)
        {
            /* 解析接收到的数据 */
            UART_BSP_ParseSbusFrame(rx_buff);
            
            /* 重新启动接收 */
            UART_BSP_StartReceive();
        }
        else /* 接收到的数据长度大于缓冲区大小，丢弃 */
        {
            /* 清空缓冲区 */
            memset(rx_buff, 0, UART_BUFF_SIZE);
            
            /* 重新启动接收 */
            UART_BSP_StartReceive();
        }
    }
}

/**
 * @brief  UART错误回调函数
 * @param  huart: UART句柄
 * @retval 无
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        /* 清空缓冲区 */
        memset(rx_buff, 0, UART_BUFF_SIZE);
        
        /* 重新启动接收 */
        UART_BSP_StartReceive();
    }
}

/* ======== 便捷功能函数实现 ======== */

/**
 * @brief  检查遥控器是否在线
 * @param  无
 * @retval 0-离线，1-在线
 */
uint8_t UART_BSP_IsRemoterOnline(void)
{
    return remoter.online;
}

/**
 * @brief  获取两个通道组合成的XY向量方向角度（弧度）
 * @param  x_channel: X轴对应的通道索引
 * @param  y_channel: Y轴对应的通道索引
 * @retval 方向角度，范围0-2π弧度
 */
float UART_BSP_GetStickAngle(whitch_stick_t stick)
{
	float x = 0.0f;
	float y = 0.0f;

	if (stick == LEFT_STICK)
	{
		/* 获取X和Y方向的百分比值，转换为-1.0到1.0范围 */
		x = remoter.joy_percent.left_hori / 100.0f;
		y = remoter.joy_percent.left_vert / 100.0f;
	}
	else if (stick == RIGHT_STICK)
	{
		/* 获取X和Y方向的百分比值，转换为-1.0到1.0范围 */
		x = remoter.joy_percent.right_hori / 100.0f;
		y = remoter.joy_percent.right_vert / 100.0f;
	}

    /* 计算角度，atan2返回-π到π的值 */
    float angle = atan2f(y, x);
    
    /* 转换为0-2π范围 */
    if (angle < 0) {
        angle += 2.0f * 3.14159f;
    }
    
    return angle;
}

/**
 * @brief  获取两个通道组合成的XY向量长度（0-100%）
 * @param  x_channel: X轴对应的通道索引
 * @param  y_channel: Y轴对应的通道索引
 * @retval 向量长度，范围0-100
 */
uint8_t UART_BSP_GetStickMagnitude(whitch_stick_t stick)
{
	float x = 0.0f;
	float y = 0.0f;

	if (stick == LEFT_STICK)
	{
		/* 获取X和Y方向的百分比值，转换为-1.0到1.0范围 */
		x = remoter.joy_percent.left_hori / 100.0f;
		y = remoter.joy_percent.left_vert / 100.0f;
	}
	else if (stick == RIGHT_STICK)
	{
		/* 获取X和Y方向的百分比值，转换为-1.0到1.0范围 */
		x = remoter.joy_percent.right_hori / 100.0f;
		y = remoter.joy_percent.right_vert / 100.0f;
	}
    
    /* 计算向量长度（欧几里得距离） */
    float magnitude = sqrtf(x*x + y*y);
    
    /* 限制在0-1范围内并转换为百分比 */
    if (magnitude > 1.0f) magnitude = 1.0f;
    
    return (uint8_t)(magnitude * 100.0f);
}