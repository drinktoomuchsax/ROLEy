/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_bsp.h"
#include <stdbool.h>
#include <stdlib.h>  // 为abs函数添加头文件
#include "wheeltec_driver.h"
#include "gmr_encoder.h"
# define PI           3.14159265358979323846  /* pi */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 定义电机控制结构体，作为示例
typedef struct {
    float left_motor_speed;
    float right_motor_speed;
    bool lights_on;
    bool grabber_active;
} robot_control_t;

// 声明实际电机控制函数（这里只是示意）
void set_motor_speeds(float left, float right);
void set_lights(bool on);
void set_grabber(bool active);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_update_time = 0;  // 上次更新时间
float encoder1_position = 0;
float encoder2_position = 0;
float encoder1_speed = 0;
float encoder2_speed = 0;
float encoder1_speed_mps = 0;
float encoder2_speed_mps = 0;
int32_t encoder1_count = 0;
int32_t encoder2_count = 0;
float encoder1_last_position = 0;
float encoder2_last_position = 0;
uint8_t throttle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t dtms;
void WheelTec_DifferentialDrive(uint8_t throttle, int8_t steering, bool reverse, uint8_t how_expert_are_u);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
remoter_t *tx_12 = NULL;

uint8_t temp_debug = 0;

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // 初始化UART BSP模块（替代直接调用HAL_UARTEx_ReceiveToIdle_DMA）
  UART_BSP_Init();
  WheelTec_Init();
  gmr_encoder_init();
  
  // 初始化机器人控制结构体
  tx_12 = UART_BSP_GetRemoterData();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dtms ++;
    // 每10ms更新一次机器人控制
    if (HAL_GetTick() - last_update_time >= 10)
    {
      last_update_time = HAL_GetTick();
      
      // 从遥控器更新机器人控制数
      
      // 应用控制命令

      if (tx_12->online&&tx_12->key.left_shoulder == KEY_DOWN && tx_12->key.right_shoulder == KEY_DOWN) {
        if (tx_12->key.left_face == KEY_MID && tx_12->key.right_face == KEY_MID) {
            // 左摇杆控制左电机
            temp_debug = 1;
            if (tx_12->joy_percent.left_vert > 10) {
                // 前进
                WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CW, abs(tx_12->joy_percent.left_vert));
            } else if (tx_12->joy_percent.left_vert < -10) {
                // 后退
                WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CCW, abs(tx_12->joy_percent.left_vert));
            } else {
                // 停止
                WheelTec_Stop(MOTOR_CHANNEL_1);
            }
            
            // 右摇杆控制右电机
            if (tx_12->joy_percent.right_vert > 10) {
                // 前进
                WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CW, abs(tx_12->joy_percent.right_vert));
            } else if (tx_12->joy_percent.right_vert < -10) {
                // 后退
                WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CCW, abs(tx_12->joy_percent.right_vert));
            } else {
                // 停止
                WheelTec_Stop(MOTOR_CHANNEL_2);
            }
        }else if (tx_12->key.left_face == KEY_DOWN && tx_12->key.right_face == KEY_DOWN) {
          // 运动学控制，两轮差速，left_vert控制前进的油门（-100为0油门，100为100油门，没有后退）,，right_hori控制转向
          WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.right_hori, false, 30);
        }else if (tx_12->key.left_face == KEY_DOWN && tx_12->key.right_face == KEY_UP) {
          // 倒挡
          WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.right_hori, true, 30);
        }else if (tx_12->key.left_face == KEY_UP && tx_12->key.right_face == KEY_DOWN) {
          // 前进
          WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.right_hori, false, 100);
        }else if (tx_12->key.left_face == KEY_UP && tx_12->key.right_face == KEY_UP) {
          // 倒挡
          WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.right_hori, true, 100);
        }
        else{
          // 停止所有电机
          WheelTec_StopAll();
        }
      }
      else if(tx_12->online&&tx_12->key.left_shoulder == KEY_UP && tx_12->key.right_shoulder == KEY_UP)
      {
        gmr_encoder_init();
      }else {
          // 遥控器离线，停止所有电机
          temp_debug = 0;
          WheelTec_StopAll();
          // WheelTec_DifferentialDrive(20, 0, false, 100);
      }

      if (tx_12->online&&tx_12->key.left_shoulder == KEY_UP && tx_12->key.right_shoulder == KEY_DOWN) {
        throttle = 0;
      }else if (tx_12->online&&tx_12->key.left_shoulder == KEY_DOWN && tx_12->key.right_shoulder == KEY_DOWN) {
        throttle = tx_12->throttle;
      }else {
        throttle = 0;
      }
      encoder1_last_position = encoder1_position;
      encoder2_last_position = encoder2_position;

      encoder1_position = gmr_encoder_get_position(ENCODER_1);
      encoder2_position = gmr_encoder_get_position(ENCODER_2);

      // 计算两个轮子的速度
      encoder1_speed = (encoder1_position - encoder1_last_position) / 0.01f;  // 度/秒
      encoder2_speed = (encoder2_position - encoder2_last_position) / 0.01f;
      
      
      // 换算成米每秒
      float wheel_diameter = 8*0.0254f; // 轮子直径,inch to meter
      float wheel_circumference = wheel_diameter * PI; // 轮子周长
      // 周长 = 直径*pi
      // 直线速度 = 轮子转速 * 周长
      // 轮子转速: 每秒转几圈
      // encoder1_speed：度每秒
      float angular_speed1 = encoder1_speed * (PI / 180.0f);  // 弧度/秒
      float angular_speed2 = encoder2_speed * (PI / 180.0f);  // 弧度/秒
      encoder1_speed_mps = angular_speed1 * wheel_circumference;
      encoder2_speed_mps = angular_speed2 * wheel_circumference;

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
/**
  * @brief  差速驱动控制函数
  * @param  throttle: 油门值 (0-100)
  * @param  steering: 转向值 (-100~100)
  * @param  reverse: 是否倒车模式
  * @param  how_expert_are_u: 你有多专业 (0-100)，越专业速度越快
  * @retval None
  */
void WheelTec_DifferentialDrive(uint8_t throttle, int8_t steering, bool reverse, uint8_t how_expert_are_u)
{
  // 计算左右轮的转速
  uint8_t left_speed = 0;
  uint8_t right_speed = 0;
  
  // 映射油门值
  if(how_expert_are_u > 100) how_expert_are_u = 100;

  if(how_expert_are_u < 100){
    throttle = throttle * how_expert_are_u / 100;
    steering = steering * how_expert_are_u / 100;
    steering = steering * 2;
  }


  if(throttle > 1){
    // 限制转速在0到100之间
    left_speed = (uint8_t)(throttle*(1 + (float)steering/100));
    right_speed = (uint8_t)(throttle*(1 - (float)steering/100));
    if (left_speed < 0) left_speed = 0;
    if (left_speed > 100) left_speed = 100;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 100) right_speed = 100;
    
    if (reverse) {
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CCW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CCW, right_speed);
    } else {
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CW, right_speed);
    }
  } else {
    // 如果油门为0，则只控制转向
    if(steering > 0){
      left_speed = steering/2;
      right_speed = steering/2;
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CCW, right_speed);
    } else if (steering < 0){
      left_speed = -steering/2;
      right_speed = -steering/2;
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CCW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CW, right_speed);
    } else {
      WheelTec_Stop(MOTOR_CHANNEL_1);
      WheelTec_Stop(MOTOR_CHANNEL_2);
    }
  }
}
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
