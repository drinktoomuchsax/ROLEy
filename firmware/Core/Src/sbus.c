/**
  ******************************************************************************
  * @file    sbus.c
  * @brief   SBUS receiver driver implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sbus.h"
#include "usart.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE];  // 接收缓冲区
static uint8_t sbus_rx_temp_buffer[SBUS_FRAME_SIZE];  // 临时缓冲区，用于DMA接收
static SBUS_Data_t sbus_data;  // SBUS解析后的数据
static uint8_t new_data_available = 0;  // 新数据标志

/* Private function prototypes -----------------------------------------------*/
static void SBUS_ParseFrame(uint8_t *frame);
static uint32_t SBUS_GetMillis(void);

/**
  * @brief  初始化SBUS接收功能
  * @retval None
  */
void SBUS_Init(void)
{
  // 清空数据
  memset(&sbus_data, 0, sizeof(SBUS_Data_t));
  memset(sbus_rx_buffer, 0, SBUS_FRAME_SIZE);
  memset(sbus_rx_temp_buffer, 0, SBUS_FRAME_SIZE);
  new_data_available = 0;
  
  // 注意：UART5已在usart.c中初始化，无需在此重复初始化
  // MX_UART5_Init();
}

/**
  * @brief  开始SBUS数据接收(使用DMA)
  * @retval None
  */
void SBUS_StartReceive(void)
{
  // 使用DMA在中断模式下接收
  HAL_UART_Receive_DMA(&huart5, sbus_rx_temp_buffer, SBUS_FRAME_SIZE);
}

/**
  * @brief  检查是否有新的SBUS数据可用
  * @retval 1:有新数据 0:无新数据
  */
uint8_t SBUS_IsNewDataAvailable(void)
{
  return new_data_available;
}

/**
  * @brief  获取解析后的SBUS数据
  * @retval SBUS_Data_t指针
  */
SBUS_Data_t* SBUS_GetData(void)
{
  new_data_available = 0;  // 重置标志
  return &sbus_data;
}

/**
  * @brief  处理接收到的SBUS数据（在DMA完成中断中调用）
  * @retval None
  */
void SBUS_Process(void)
{
  // 检查帧头是否正确
  if (sbus_rx_temp_buffer[0] == SBUS_HEADER)
  {
    // 复制接收到的数据到接收缓冲区
    memcpy(sbus_rx_buffer, sbus_rx_temp_buffer, SBUS_FRAME_SIZE);
    
    // 解析SBUS帧
    SBUS_ParseFrame(sbus_rx_buffer);
    
    // 设置新数据可用标志
    new_data_available = 1;
    
    // 更新最后接收时间
    sbus_data.last_update_ms = SBUS_GetMillis();
  }
  
  // 重新启动DMA接收
  HAL_UART_Receive_DMA(&huart5, sbus_rx_temp_buffer, SBUS_FRAME_SIZE);
}

/**
  * @brief  解析SBUS数据帧
  * @param  frame: SBUS数据帧缓冲区
  * @retval None
  */
static void SBUS_ParseFrame(uint8_t *frame)
{
  // 检查帧头和帧尾
  if (frame[0] != SBUS_HEADER || frame[24] != SBUS_FOOTER)
  {
    return;
  }
  
  // 解析16个通道的数据 (11位每通道)
  // SBUS数据是从低位到高位排列的
  sbus_data.channels[0]  = ((frame[1]       | frame[2] << 8)                     & 0x07FF);
  sbus_data.channels[1]  = ((frame[2] >> 3  | frame[3] << 5)                     & 0x07FF);
  sbus_data.channels[2]  = ((frame[3] >> 6  | frame[4] << 2  | frame[5] << 10)   & 0x07FF);
  sbus_data.channels[3]  = ((frame[5] >> 1  | frame[6] << 7)                     & 0x07FF);
  sbus_data.channels[4]  = ((frame[6] >> 4  | frame[7] << 4)                     & 0x07FF);
  sbus_data.channels[5]  = ((frame[7] >> 7  | frame[8] << 1  | frame[9] << 9)    & 0x07FF);
  sbus_data.channels[6]  = ((frame[9] >> 2  | frame[10] << 6)                    & 0x07FF);
  sbus_data.channels[7]  = ((frame[10] >> 5 | frame[11] << 3)                    & 0x07FF);
  sbus_data.channels[8]  = ((frame[12]      | frame[13] << 8)                    & 0x07FF);
  sbus_data.channels[9]  = ((frame[13] >> 3 | frame[14] << 5)                    & 0x07FF);
  sbus_data.channels[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10)  & 0x07FF);
  sbus_data.channels[11] = ((frame[16] >> 1 | frame[17] << 7)                    & 0x07FF);
  sbus_data.channels[12] = ((frame[17] >> 4 | frame[18] << 4)                    & 0x07FF);
  sbus_data.channels[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9)   & 0x07FF);
  sbus_data.channels[14] = ((frame[20] >> 2 | frame[21] << 6)                    & 0x07FF);
  sbus_data.channels[15] = ((frame[21] >> 5 | frame[22] << 3)                    & 0x07FF);
  
  // 解析标志位
  uint8_t flags = frame[23];
  sbus_data.channel_17 = (flags & SBUS_FLAG_CHANNEL_17) ? 1 : 0;
  sbus_data.channel_18 = (flags & SBUS_FLAG_CHANNEL_18) ? 1 : 0;
  sbus_data.frame_lost = (flags & SBUS_FLAG_FRAME_LOST) ? 1 : 0;
  sbus_data.failsafe_activated = (flags & SBUS_FLAG_FAILSAFE) ? 1 : 0;
}

/**
  * @brief  获取系统运行的毫秒数
  * @retval 毫秒数
  */
static uint32_t SBUS_GetMillis(void)
{
  return HAL_GetTick();
}

/**
  * @brief  UART接收完成回调函数（需在stm32h7xx_it.c的UART5中断处理中调用）
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    SBUS_Process();
  }
} 