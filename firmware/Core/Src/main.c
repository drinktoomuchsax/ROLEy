/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "sbus.h"
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 遥控器通道定义
#define RC_CH_ROLL      0  // 横滚(左右)通道
#define RC_CH_PITCH     1  // 俯仰(前后)通道
#define RC_CH_THROTTLE  2  // 油门通道
#define RC_CH_YAW       3  // 偏航通道
#define RC_CH_SWITCH1   4  // 拨杆1
#define RC_CH_SWITCH2   5  // 拨杆2
#define RC_CH_SWITCH3   6  // 拨杆3

// 遥控器值范围
#define RC_MIN_VALUE    172    // SBUS最小值
#define RC_MAX_VALUE    1811   // SBUS最大值
#define RC_RANGE        (RC_MAX_VALUE - RC_MIN_VALUE)

// 遥控器数据结构
typedef struct {
  float roll;          // 横滚，范围[-1, 1]
  float pitch;         // 俯仰，范围[-1, 1]
  float throttle;      // 油门，范围[0, 1]
  float yaw;           // 偏航，范围[-1, 1]
  int switch1;         // 拨杆1，位置[0, 1, 2]
  int switch2;         // 拨杆2，位置[0, 1, 2]
  int switch3;         // 拨杆3，位置[0, 1, 2]
  uint8_t connected;   // 连接状态
} RC_Data_t;

RC_Data_t rc_data = {0};  // 遥控器数据

// 遥控器数据处理函数
void RC_ProcessData(SBUS_Data_t *sbus_data);

// 调试辅助变量
uint32_t last_print_time = 0;
#define DEBUG_PRINT_INTERVAL 500  // 打印间隔，单位ms
char debug_buffer[128];           // 调试信息缓冲区
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  将SBUS值映射到标准范围
  * @param  sbus_value: SBUS原始值
  * @param  is_throttle: 是否为油门通道
  * @retval 标准化后的值
  */
float RC_MapValue(uint16_t sbus_value, uint8_t is_throttle)
{
  // 限制SBUS值在有效范围内
  if (sbus_value < RC_MIN_VALUE) sbus_value = RC_MIN_VALUE;
  if (sbus_value > RC_MAX_VALUE) sbus_value = RC_MAX_VALUE;
  
  // 计算标准化值
  float value = (float)(sbus_value - RC_MIN_VALUE) / RC_RANGE;
  
  // 对于非油门通道，将范围从[0, 1]调整为[-1, 1]
  if (!is_throttle) {
    value = value * 2.0f - 1.0f;
  }
  
  return value;
}

/**
  * @brief  解析SBUS数据为遥控器控制数据
  * @param  sbus_data: SBUS解析后的数据
  * @retval None
  */
void RC_ProcessData(SBUS_Data_t *sbus_data)
{
  // 只有在没有失效保护激活时才处理数据
  if (!sbus_data->failsafe_activated && !sbus_data->frame_lost) {
    // 设置连接状态
    rc_data.connected = 1;
    
    // 解析摇杆通道
    rc_data.roll = RC_MapValue(sbus_data->channels[RC_CH_ROLL], 0);         // 横滚 -1:左, 1:右
    rc_data.pitch = RC_MapValue(sbus_data->channels[RC_CH_PITCH], 0);       // 俯仰 -1:后, 1:前
    rc_data.throttle = RC_MapValue(sbus_data->channels[RC_CH_THROTTLE], 1); // 油门 0:最小, 1:最大
    rc_data.yaw = RC_MapValue(sbus_data->channels[RC_CH_YAW], 0);           // 偏航 -1:左, 1:右
    
    // 解析拨杆通道（3位置拨杆）
    uint16_t raw_switch;
    
    // 拨杆1解析
    raw_switch = sbus_data->channels[RC_CH_SWITCH1];
    if (raw_switch < RC_MIN_VALUE + RC_RANGE/3) {
      rc_data.switch1 = 0;      // 位置0
    } else if (raw_switch < RC_MIN_VALUE + 2*RC_RANGE/3) {
      rc_data.switch1 = 1;      // 位置1
    } else {
      rc_data.switch1 = 2;      // 位置2
    }
    
    // 拨杆2解析
    raw_switch = sbus_data->channels[RC_CH_SWITCH2];
    if (raw_switch < RC_MIN_VALUE + RC_RANGE/3) {
      rc_data.switch2 = 0;      // 位置0
    } else if (raw_switch < RC_MIN_VALUE + 2*RC_RANGE/3) {
      rc_data.switch2 = 1;      // 位置1
    } else {
      rc_data.switch2 = 2;      // 位置2
    }
    
    // 拨杆3解析
    raw_switch = sbus_data->channels[RC_CH_SWITCH3];
    if (raw_switch < RC_MIN_VALUE + RC_RANGE/3) {
      rc_data.switch3 = 0;      // 位置0
    } else if (raw_switch < RC_MIN_VALUE + 2*RC_RANGE/3) {
      rc_data.switch3 = 1;      // 位置1
    } else {
      rc_data.switch3 = 2;      // 位置2
    }
  } else {
    // 失效保护激活或帧丢失，标记为未连接
    rc_data.connected = 0;
    
    // 设置安全值
    rc_data.roll = 0.0f;
    rc_data.pitch = 0.0f;
    rc_data.throttle = 0.0f;
    rc_data.yaw = 0.0f;
    rc_data.switch1 = 0;
    rc_data.switch2 = 0;
    rc_data.switch3 = 0;
  }
}

/**
  * @brief  将数据打印到串口
  * @param  data: 数据字符串
  * @retval None
  */
void Debug_Print(char *data)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  // 初始化SBUS接收
  SBUS_Init();
  // 启动SBUS接收
  SBUS_StartReceive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 检查SBUS是否有新数据
    if (SBUS_IsNewDataAvailable())
    {
      // 获取SBUS数据
      SBUS_Data_t *sbus_data = SBUS_GetData();
      
      // 处理SBUS数据，转换为标准化的控制值
      RC_ProcessData(sbus_data);
      
      // 在这里可以使用rc_data中的数据进行控制
      // 例如: 使用rc_data.roll, rc_data.pitch, rc_data.throttle, rc_data.yaw进行飞行控制
      // 或根据拨杆位置切换模式:
      
      // 示例: 根据拨杆1位置切换不同模式
      switch (rc_data.switch1) {
        case 0:
          // 拨杆位置0模式处理
          // 例如: 手动模式
          break;
        case 1:
          // 拨杆位置1模式处理
          // 例如: 姿态模式
          break;
        case 2:
          // 拨杆位置2模式处理
          // 例如: 自动模式
          break;
      }
      
      // 检查遥控器连接状态
      if (!rc_data.connected) {
        // 遥控器未连接或失效保护激活
        // 执行安全降落或停机操作
      }
    }

    // 定期通过串口输出遥控器数据进行调试
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_print_time >= DEBUG_PRINT_INTERVAL)
    {
      last_print_time = current_time;
      
      // 格式化标准化后的遥控器信息
      sprintf(debug_buffer, "RC: R=%.2f P=%.2f T=%.2f Y=%.2f S1=%d S2=%d S3=%d CON=996123\r\n",
              rc_data.roll, rc_data.pitch, rc_data.throttle, rc_data.yaw,
              rc_data.switch1, rc_data.switch2, rc_data.switch3);
      Debug_Print(debug_buffer);
      
      // 如果需要，还可以输出原始SBUS值
      if (SBUS_IsNewDataAvailable())
      {
        SBUS_Data_t *sbus_data = SBUS_GetData();
        sprintf(debug_buffer, "RAW: CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d FL=%d FS=996\r\n",
                sbus_data->channels[0], sbus_data->channels[1], 
                sbus_data->channels[2], sbus_data->channels[3],
                sbus_data->channels[4], sbus_data->frame_lost);
        Debug_Print(debug_buffer);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
