#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

//滴答定时器初始化
void Delay_Init(void);

//延时us
void Delay_us(__IO uint32_t nTime);

//延时ms
void Delay_ms(__IO uint32_t nTime);





#endif

