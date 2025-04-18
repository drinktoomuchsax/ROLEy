/**
  ******************************************************************************
  * @file    sbus.h
  * @brief   SBUS receiver driver header file
  ******************************************************************************
  */

#ifndef __SBUS_H__
#define __SBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* SBUS Protocol Definitions -------------------------------------------------*/
#define SBUS_FRAME_SIZE          25      // SBUS帧大小：25字节
#define SBUS_NUM_CHANNELS        16      // SBUS支持的通道数
#define SBUS_HEADER              0x0F    // SBUS帧头
#define SBUS_FOOTER              0x00    // SBUS帧尾
#define SBUS_FLAG_CHANNEL_17     0x01    // 标志位：通道17
#define SBUS_FLAG_CHANNEL_18     0x02    // 标志位：通道18
#define SBUS_FLAG_FRAME_LOST     0x04    // 标志位：帧丢失
#define SBUS_FLAG_FAILSAFE       0x08    // 标志位：失效保护已激活

/* SBUS Data Structure -------------------------------------------------------*/
typedef struct {
    uint16_t channels[SBUS_NUM_CHANNELS];  // 16个通道的值 (11位每通道)
    uint8_t channel_17;                    // 数字通道17 (0或1)
    uint8_t channel_18;                    // 数字通道18 (0或1)
    uint8_t frame_lost;                    // 帧丢失标志
    uint8_t failsafe_activated;            // 失效保护激活标志
    uint32_t last_update_ms;               // 最后更新时间（毫秒）
} SBUS_Data_t;

/* Function Prototypes -------------------------------------------------------*/
void SBUS_Init(void);
void SBUS_StartReceive(void);
uint8_t SBUS_IsNewDataAvailable(void);
SBUS_Data_t* SBUS_GetData(void);
void SBUS_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H__ */ 