#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

//�δ�ʱ����ʼ��
void Delay_Init(void);

//��ʱus
void Delay_us(__IO uint32_t nTime);

//��ʱms
void Delay_ms(__IO uint32_t nTime);





#endif

