#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "main.h"
#include "usart.h"
#include <stdint.h>

/* 宏定义 */
#define UART_BUFF_SIZE     25
#define SBUS_HEAD          0x0F
#define SBUS_END           0x00
#define SBUS_OFFLINE_FLAG  0x0C

/* 兼容旧代码的宏定义 */
#define BUFF_SIZE          UART_BUFF_SIZE

/* 通道数量定义 */
#define REMOTE_CHANNEL_COUNT  9 // 8个遥控器通道+1个rssi

/* 通道中值和范围定义 */
#define REMOTE_CHANNEL_MID   0x03E0 // 中值,992
#define REMOTE_CHANNEL_TOP   0x06DF // 最大值,1759
#define REMOTE_CHANNEL_BOTTOM 0x00E0 // 最小值,224


typedef enum
{
    LEFT_STICK = 0,
    RIGHT_STICK = 1
} whitch_stick_t;

/* 按键状态枚举 */
typedef enum {
    KEY_UP = 0x00E0,
    KEY_MID = 0x03E0,
    KEY_DOWN = 0x06DF
} key_state_t;

/* 遥控器数据结构体 */
typedef struct
{
    uint16_t online;  /* 连接状态: 1-在线, 0-离线 */

    struct
    {
        int16_t ch[REMOTE_CHANNEL_COUNT];  /* 原始通道数据 */
    } rc;

    struct
    {
        /* 摇杆数值 */
        uint16_t left_vert;   /* 左摇杆垂直方向 */
        uint16_t left_hori;   /* 左摇杆水平方向 */
        uint16_t right_vert;  /* 右摇杆垂直方向 */
        uint16_t right_hori;  /* 右摇杆水平方向 */
    } joy;

    struct
    {
        int8_t left_vert;   /* 左摇杆垂直方向，-100~100 */
        int8_t left_hori;   /* 左摇杆水平方向，-100~100 */
        int8_t right_vert;  /* 右摇杆垂直方向，-100~100 */
        int8_t right_hori;  /* 右摇杆水平方向，-100~100 */
    } joy_percent;

    struct
    {
        float left_angle;   /* 左摇杆方向角度，0~2π */
        float right_angle;  /* 右摇杆方向角度，0~2π */
    } joy_direction;

    struct
    {
        float left_magnitude;   /* 左摇杆向量长度，0~100 */
        float right_magnitude;  /* 右摇杆向量长度，0~100 */
    } joy_magnitude;

    struct
    {
        /* 按键数值 */
        key_state_t left_shoulder; // 左肩键
        key_state_t left_face; // 左脸键
        key_state_t right_face; // 右脸键
        key_state_t right_shoulder; // 右肩键
    } key;

    uint16_t rssi; // rssi
    uint8_t throttle; // 油门
} remoter_t;

/* 对外接口函数 */
/* 初始化UART接收处理 */
void UART_BSP_Init(void);

/* 启动UART接收 */
void UART_BSP_StartReceive(void);

/* 获取遥控器数据的引用 */
remoter_t* UART_BSP_GetRemoterData(void);

/* 解析SBUS数据帧 */
void UART_BSP_ParseSbusFrame(uint8_t *buf);

/* 获取接收缓冲区的指针（兼容旧代码） */
uint8_t* UART_BSP_GetRxBuffer(void);

/* 兼容旧代码，导出接收缓冲区 */
extern uint8_t rx_buff[];

/* ======== 便捷功能函数 ======== */

/* 检查遥控器是否在线 */
uint8_t UART_BSP_IsRemoterOnline(void);

/* 获取两个通道组合成的XY向量方向角度（弧度） */
float UART_BSP_GetStickAngle(whitch_stick_t stick);

/* 获取两个通道组合成的XY向量长度（0-100%） */
uint8_t UART_BSP_GetStickMagnitude(whitch_stick_t stick);


#endif /*__UART_BSP_H__ */

