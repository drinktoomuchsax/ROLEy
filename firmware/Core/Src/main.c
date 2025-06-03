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
#include "fdcan.h"
#include "spi.h"
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
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "roley_neck.h"
#include "BMI088driver.h"
#include "balance_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  CONTROL_STATE_MANUAL = 0,
  CONTROL_STATE_BALANCE = 1,
  CONTROL_STATE_LINE_FOLLOWING = 2,
} control_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
# define PI           3.14159265358979323846  /* pi */
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
// 平衡控制相关变量
float pitch_angle = 0.0f;    // 俯仰角
float pitch_gyro = 0.0f;     // 俯仰角速度
bool balance_mode = false;   // 平衡模式标志
control_state_t control_state = CONTROL_STATE_MANUAL; // 控制状态，默认是手动模式，可以切换到平衡小车，线控循迹
uint32_t dtms; // drinktoomuchsax，用来看程序有没有跑起来
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void WheelTec_DifferentialDrive(uint8_t throttle, int8_t steering, bool reverse, uint8_t how_expert_are_u);
void update_control_state(void);
void update_neck_motor(void);
void manual_control(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
remoter_t *tx_12 = NULL;

uint8_t temp_debug = 0;
uint8_t tx_data[8] = {6,6,6,6,6,6,6,6};
float gyro[3], accel[3], temp;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    neck_motor_update(); // 每1ms更新一次颈部的两个电机
  }
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
  MX_TIM4_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  UART_BSP_Init(); // SBUS初始化
  WheelTec_Init(); // GPIO和pwm初始化
  gmr_encoder_init();
	bsp_can_init();
  dm_motor_init();
  neck_motor_init(); // 初始化颈部的两个4310电机
  HAL_TIM_Base_Start_IT(&htim2); // 开启定时器2的中断，1ms给电机发送一次CAN消息
  balance_init(); // 初始化平衡控制


  WheelTec_PIDInit();
  // 初始化机器人控制结构体
  tx_12 = UART_BSP_GetRemoterData();

  while(BMI088_init())
  {
      ;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 每10ms更新一次机器人控制
    if (HAL_GetTick() - last_update_time >= 10)
    {
      // 调试用
      dtms ++; // 程序有没有跑起来
      if(dtms % 100 == 0){
        fdcanx_send_data(&hfdcan1, 0x666, tx_data, 8); // 每1s发送一次0x666的CAN消息
      }
      
      // 读取IMU数据
      BMI088_read(gyro, accel, &temp);
      
      // 计算俯仰角（使用加速度计数据）
      pitch_angle = atan2f(accel[2], -accel[1]) * 180.0f / PI; // deg
      pitch_gyro = gyro[0];  // 使用Y轴陀螺仪数据

      encoder1_last_position = encoder1_position;
      encoder2_last_position = encoder2_position;

      encoder1_position = gmr_encoder_get_position(ENCODER_1);
      encoder2_position = gmr_encoder_get_position(ENCODER_2);

      // 计算两个轮子的速度
      encoder1_speed = (encoder1_position - encoder1_last_position) / 0.01f;  // 度/秒
      encoder2_speed = (encoder2_position - encoder2_last_position) / 0.01f;
      
      // 模式检查，是否解锁底盘，现在是哪个模式
      bool chassis_unlock = tx_12->key.left_shoulder == KEY_DOWN;
      update_control_state();

      update_neck_motor(); // 不管什么模式，都要更新颈部电机
      if (tx_12->online) {
        if (chassis_unlock) {
          if (control_state == CONTROL_STATE_MANUAL) { // 手动模式
            manual_control();
          }else if(control_state == CONTROL_STATE_BALANCE){ // 平衡小车模式
            balance_update(pitch_angle, encoder1_speed+encoder2_speed);
          }else if(control_state == CONTROL_STATE_LINE_FOLLOWING){ // 线控循迹模式
            WheelTec_StopAll();
            // TODO: line_follow_update();
          }
        }else{
          // 底盘解锁，但是没有按下遥控器，停止所有电机
          WheelTec_StopAll();
        }
      } else {
        // 遥控器离线，停止所有电机
        temp_debug = 0;
        WheelTec_StopAll();
      }
      
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

      last_update_time = HAL_GetTick();
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
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CW, right_speed);
    } else {
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CCW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CCW, right_speed);
    }
  } else {
    // 如果油门为0，则只控制转向
    if(steering > 0){
      left_speed = steering/2;
      right_speed = steering/2;
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CCW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CW, right_speed);
    } else if (steering < 0){
      left_speed = -steering/2;
      right_speed = -steering/2;
      WheelTec_Control(MOTOR_CHANNEL_2, MOTOR_DIRECTION_CW, left_speed);
      WheelTec_Control(MOTOR_CHANNEL_1, MOTOR_DIRECTION_CCW, right_speed);
    } else {
      WheelTec_Stop(MOTOR_CHANNEL_1);
      WheelTec_Stop(MOTOR_CHANNEL_2);
    }
  }
}

/**
 * @brief 根据遥控器右肩键更新控制状态，右上为平衡，右中为线控循迹，右下为手动
 * 
 */
void update_control_state(void){
  if (tx_12->key.right_shoulder == KEY_DOWN)
  {
    control_state = CONTROL_STATE_MANUAL;
  }
  else if(tx_12->key.right_shoulder == KEY_MID)
  {
    control_state = CONTROL_STATE_LINE_FOLLOWING;
  }else if(tx_12->key.right_shoulder == KEY_UP)
  {
    control_state = CONTROL_STATE_BALANCE;
  }else{
    control_state = CONTROL_STATE_MANUAL;
  }
}

/**
 * @brief 控制颈部两个4310电机
 * 
 */
void update_neck_motor(void){
  // 颈部电机控制
  float neck_yaw_left_limit = -PI * (75.0f/180.0f); //75度
  float neck_yaw_right_limit = PI * (75.0f/180.0f); //75度
  float neck_pitch_lower_limit = -PI * (10.0f/180.0f); //按道理应该10度，这个地方是有一些上下限问题，请忽视这个命名
  float neck_pitch_upper_limit = PI * (20.0f/180.0f); //按道理20度，同上
  float yaw_angle = -(tx_12->joy_percent.right_hori/100.0f) * PI/2;
  float pitch_angle = -(tx_12->joy_percent.right_vert/100.0f) * PI/2;
  neck_rotate_yaw(yaw_angle, neck_yaw_left_limit, neck_yaw_right_limit);
  neck_rotate_pitch(pitch_angle, neck_pitch_lower_limit, neck_pitch_upper_limit);
}

/**
 * @brief 手动控制底盘，有三种模式，完全手动，带运动学控制，全速运动学控制
 * 
 */
void manual_control(void){
  if (tx_12->key.left_face == KEY_MID && tx_12->key.right_face == KEY_MID) {// 左中右中，左右摇杆各控制左右电机
    // 左摇杆控制左电机
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
  } else if (tx_12->key.left_face == KEY_DOWN && tx_12->key.right_face == KEY_DOWN) {// 左下右下，普通运动学控制
    // 运动学控制，两轮差速，left_vert控制前进的油门（-100为0油门，100为100油门，没有后退）,，right_hori控制转向
    // baby速度前进
    WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.left_hori, false, 30);
  } else if (tx_12->key.left_face == KEY_DOWN && tx_12->key.right_face == KEY_UP) { // 左下右上，倒挡
    // baby速度倒挡
    WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.left_hori, true, 30);
  } else if (tx_12->key.left_face == KEY_UP && tx_12->key.right_face == KEY_DOWN) { // 左上右下，全速运动学控制
    // 全速前进
    WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.right_hori, false, 100);
  } else if (tx_12->key.left_face == KEY_UP && tx_12->key.right_face == KEY_UP) { // 左上右上，全速倒挡
    // 全速倒挡
    WheelTec_DifferentialDrive(tx_12->throttle, tx_12->joy_percent.right_hori, true, 100);
  } else {
    // 停止所有电机
    WheelTec_StopAll();
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
