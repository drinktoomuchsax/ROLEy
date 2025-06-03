//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//单片机交流群：439190573
//淘宝店：https://futureworldrobot.taobao.com
//修改日期：2023/6/5
//版本：V5
//版权所有，盗版必究。
//未来世界机器人系列
//////////////////////////////////////////////////////////////////////////////////
#include "motor.h"	   
#include "sys.h"
#include "delay.h"


/*************************************
**函数功能：电机初始化函数
**参数：
**
**************************************/
void Motor_Config(void)	 //定时器4的3、4通道
{	 
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//打开TIM4外设时钟                     
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = 0;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能的预装载寄存器
	
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);	//使能定时器4
	TIM_Cmd(TIM4, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//推挽输出
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*************************************
** 函数: motor(int Motor_1, int Motor_2)
** 描述: 左右电机速度控制
** 参数: Motor_1, 左电机PWM，取值范围-100至100
**		 Motor_2, 右电机PWM，取值范围-100至100，负值为向后转，正值为向前转
**方向：PD2、PD3控制电机1方向，PD0、PD1控制电机2方向
**************************************/
void motor(int Motor_1, int Motor_2)
{
	if (Motor_1 > 0)	
	{
//		Motor_1 = 100-Motor_1;
 		TIM4->CCR3 =  Motor_1 * 10;	 
		AIN1 = 1;AIN2 = 0;
	}
	else if (Motor_1 < 0)	 
	{
		Motor_1 = -Motor_1;
		TIM4->CCR3 =  Motor_1 * 10;	 
		AIN1 = 0;AIN2 = 1;
	}
	else if (Motor_1 == 0)	
	{
//		Motor_1 = 100+Motor_1;
		TIM4->CCR3 =  Motor_1 * 10;	 
		AIN1 = 0;AIN2 = 0;
	}
	if (Motor_2 > 0)	 
	{
//		Motor_2 = 100-Motor_2;
		TIM4->CCR4 =  Motor_2 * 10;	 
		BIN1 = 1;BIN2 = 0;
	}
	else if (Motor_2 < 0)	 
	{
		Motor_2 = -Motor_2;
		TIM4->CCR4 = Motor_2 * 10;	 
		BIN1 = 0;BIN2 = 1;
	}
	else if (Motor_2 == 0)	
	{
//		Motor_2 = 100+Motor_2;
		TIM4->CCR4 = Motor_2 * 10;	
		BIN1 = 0;BIN2 = 0;
	}
}

