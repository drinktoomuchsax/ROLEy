#ifndef __MOTOR_H
#define __MOTOR_H 		

#include "stm32f10x.h"

#define	AIN1		PDout(0)
#define	AIN2		PDout(1)
#define	BIN1		PDout(2)
#define	BIN2		PDout(3)

void Motor_Config(void);
void motor(int Motor_1, int Motor_2);

#endif







