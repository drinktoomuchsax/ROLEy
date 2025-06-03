//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ���������������;
//��Ƭ������Ⱥ��439190573
//�Ա��꣺https://futureworldrobot.taobao.com
//�޸����ڣ�2023/6/5
//�汾��V5
//��Ȩ���У�����ؾ���
//δ�����������ϵ��
//////////////////////////////////////////////////////////////////////////////////
#include "motor.h"	   
#include "sys.h"
#include "delay.h"


/*************************************
**�������ܣ������ʼ������
**������
**
**************************************/
void Motor_Config(void)	 //��ʱ��4��3��4ͨ��
{	 
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//��TIM4����ʱ��                     
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
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);//ʹ�ܵ�Ԥװ�ؼĴ���
	
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);//ʹ�ܵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);	//ʹ�ܶ�ʱ��4
	TIM_Cmd(TIM4, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//�������
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*************************************
** ����: motor(int Motor_1, int Motor_2)
** ����: ���ҵ���ٶȿ���
** ����: Motor_1, ����PWM��ȡֵ��Χ-100��100
**		 Motor_2, �ҵ��PWM��ȡֵ��Χ-100��100����ֵΪ���ת����ֵΪ��ǰת
**����PD2��PD3���Ƶ��1����PD0��PD1���Ƶ��2����
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

