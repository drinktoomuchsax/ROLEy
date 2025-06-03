//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//单片机交流群：439190573
//淘宝店：https://futureworldrobot.taobao.com
//修改日期：2023/6/5
//版本：V5
//版权所有，盗版必究。
//未来世界机器人系列
//////////////////////////////////////////////////////////////////////////////////
#include "sensor.h"

/*************************************
*函数名称：SENSOR_GPIO_Config
*函数功能：GPIO管脚的配置
*参数：
*说明：
*			
**************************************/
void SENSOR_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9 |GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 |GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//数字上拉输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//数字上拉输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//数字上拉输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*************************************
*函数名称：digtal
*函数功能：获取X通道数字值
*参数：
*说明：
*			
**************************************/
unsigned char digtal(unsigned char channel)//1-ADC_N	  获取X通道数字值
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(PEin(8) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(PEin(9) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(PEin(10) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(PEin(11) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(PEin(12) == 1) value = 1;
			else value = 0;  
			break;
		case 6:  
			if(PEin(13) == 1) value = 1;
			else value = 0;  
			break;  
		case 7: 
			if(PEin(14) == 1) value = 1;
			else value = 0;  
			break;  
		case 8: 
			if(PEin(15) == 1) value = 1;
			else value = 0;  
			break;  
		case 9:
			if(PBin(10) == 1) value = 1;
			else value = 0;  
			break;
		case 10:  
			if(PBin(11) == 1) value = 1;
			else value = 0;  
			break;  
		case 11: 
			if(PDin(12) == 1) value = 1;
			else value = 0;  
			break;  
		case 12: 
			if(PDin(13) == 1) value = 1;
			else value = 0;  
			break; 
	}
	return value; 
}




