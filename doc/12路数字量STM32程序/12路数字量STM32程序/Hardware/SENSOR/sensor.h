#ifndef __SENSOR_H
#define	__SENSOR_H

#include "sys.h"
#include "stm32f10x.h"	   

/*-------------���ֶ˿�----------------*/
#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)
#define D9 digtal(9)
#define D10 digtal(10)
#define D11 digtal(11)
#define D12 digtal(12)


void SENSOR_GPIO_Config(void);
u8 digtal(u8 channel);  	//��ȡXͨ������ֵ��0��1�� 1~8						  


#endif 
