//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ���������������;
//��Ƭ������Ⱥ��439190573
//�Ա��꣺https://futureworldrobot.taobao.com
//�޸����ڣ�2023/6/5
//�汾��V5
//��Ȩ���У�����ؾ���
//δ�����������ϵ��
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
	Delay_Init();						//��ʱ������ʼ��
	Motor_Config();					//�����ʼ��
	SENSOR_GPIO_Config();	  //���ֿڳ�ʼ��
	Usart3_Init(115200);		//���ڳ�ʼ��
	IIC_Init(); 						//IIC�ӿڳ�ʼ��
	
	motor(0,0);
	Delay_ms(1000);		//��ʱ1��ȴ�ϵͳ�ȶ����ɲ�д��
	
//	Set_ID(0x57,0x01);		//IICͨ�����ô�������ַ
//	Set_Data1(0x01,0x01,0x03);		//����ͨ�����ô���������
	
	while(1)
	{				
//		track_zhixian1();
		track_zhixian2();		
//		track_PID1(40,0.05);
//		track_PID2(40,0.05);
//		track_PID3(40,0.05);	
	}
}



