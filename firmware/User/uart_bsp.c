#include "uart_bsp.h"
#include "string.h"
#include "usart.h"
#include <stdio.h>

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00

uint8_t rx_buff[BUFF_SIZE];
remoter_t remoter;

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;

    if (buf[23] == 0x0C)
        remoter->online = 0;
    else
        remoter->online = 1;

    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);

	printf("remoter->rc.ch[0]: %d\n", remoter->rc.ch[0]);
	printf("remoter->rc.ch[1]: %d\n", remoter->rc.ch[1]);
	printf("remoter->rc.ch[2]: %d\n", remoter->rc.ch[2]);
	printf("remoter->rc.ch[3]: %d\n", remoter->rc.ch[3]);
	printf("remoter->rc.ch[4]: %d\n", remoter->rc.ch[4]);
	printf("remoter->rc.ch[5]: %d\n", remoter->rc.ch[5]);
	printf("remoter->rc.ch[6]: %d\n", remoter->rc.ch[6]);
	printf("remoter->rc.ch[7]: %d\n", remoter->rc.ch[7]);
	printf("remoter->rc.ch[8]: %d\n", remoter->rc.ch[8]);
	printf("remoter->rc.ch[9]: %d\n", remoter->rc.ch[9]);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size <= BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ������Ϻ�����
			sbus_frame_parse(&remoter, rx_buff);
//			memset(rx_buff, 0, BUFF_SIZE);
		}
		else  // �������ݳ��ȴ���BUFF_SIZE��������
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ������Ϻ�����
			memset(rx_buff, 0, BUFF_SIZE);							   
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ���շ������������
		memset(rx_buff, 0, BUFF_SIZE);							   // ������ջ���		
	}
}
