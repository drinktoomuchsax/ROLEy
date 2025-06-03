#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"


//´®¿Ú3³õÊ¼»¯
void Usart3_Init(u32 Baud);
void Read_Data1(u8 addr, u32 *Data);	       
void Read_Data2(u8 addr, u32 *Data);
void Read_Data3(u8 addr, u32 *Data);
void Read_Data4(u8 addr, u32 *Data);

void Set_Data1(u8 add,u8 Par1,u8 Par2);	
#endif



