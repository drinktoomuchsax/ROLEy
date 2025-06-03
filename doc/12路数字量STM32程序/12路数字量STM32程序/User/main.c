//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//单片机交流群：439190573
//淘宝店：https://futureworldrobot.taobao.com
//修改日期：2023/6/5
//版本：V5
//版权所有，盗版必究。
//未来世界机器人系列
//////////////////////////////////////////////////////////////////////////////////
#include "stm32f10x.h"
#include "delay.h"
#include "sensor.h"
#include "usart.h"
#include "sys.h"
#include "motor.h"
#include "Line.h"
#include "iic.h"


int main(void)
{		
	Delay_Init();						//延时函数初始化
	Motor_Config();					//电机初始化
	SENSOR_GPIO_Config();	  //数字口初始化
	Usart3_Init(115200);		//串口初始化
	IIC_Init(); 						//IIC接口初始化
	
	motor(0,0);
	Delay_ms(1000);		//延时1秒等待系统稳定（可不写）
	
//	Set_ID(0x57,0x01);		//IIC通信设置传感器地址
//	Set_Data1(0x01,0x01,0x03);		//串口通信设置传感器参数
	
	while(1)
	{				
//		track_zhixian1();
		track_zhixian2();		
//		track_PID1(40,0.05);
//		track_PID2(40,0.05);
//		track_PID3(40,0.05);	
	}
}



