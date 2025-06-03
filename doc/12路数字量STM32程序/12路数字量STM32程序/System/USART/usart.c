//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//单片机交流群：439190573
//淘宝店：https://futureworldrobot.taobao.com
//修改日期：2023/6/5
//版本：V5
//版权所有，盗版必究。
//未来世界机器人系列
//////////////////////////////////////////////////////////////////////////////////
#include "usart.h"
#include "delay.h"
#include "Line.h"
/*********************************************************************
 *  函数名称：void Usart3_Init
 *  函数功能：串口3初始化
 *  形    参：无
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Usart3_Init(u32 Baud)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	
	//USART3_TX		PC.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;					//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//USART3_RX		PC.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;			//浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//对串口3通信协议进行初始化设置
	USART_InitStructure.USART_BaudRate = Baud;						//设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;				//无奇偶效验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//双向通信
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);					//开启接收中断	
	USART_Cmd(USART3, ENABLE);

	//对串口3收发中断设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					//中断组选第二组
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  		//先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  			//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
}

u8 USART_RX_STA[31] = { 0 };    //接收状态标记	，字节数：2+3+24+2  
u8 Num = 0;              //接收数据的当前位置

/*********************************************************************
 *  函数名称：Read_Data1(unsigned char addr)
 *  函数功能：读取串口数字量数据，并存放在数组里面
 *  形    参：addr，传感器地址,*Data，指针
 *  输    出：无
 *  备    注：串口1的时候需要修改
 *			   	   
 ********************************************************************/    
void Read_Data1(u8 addr, u32 *Data)	     
{	
	u8 y=0;
	unsigned int Receive_data = 0;       //数据缓存区
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x57);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, addr);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
///////////////////////////数字量数值///////////////////////////////	
	while(1)
  {
		if(Num == 4)
		{
			Num = 0;
			if(USART_RX_STA[0x03] == 0x03)  //判断帧尾0x03,否者不赋值
			{
				Receive_data = USART_RX_STA[2];
				Receive_data <<= 8;
				Receive_data |= USART_RX_STA[1];
				
				*Data = Receive_data;    //数字量赋值     
				break;
			}        
			break;
		}
    else
    {
      Delay_ms(1);y++;
      if(y==3) { Num = 0;break; }
    }		
	}
///////////////////////////数字量数值///////////////////////////////	
}
/*********************************************************************
 *  函数名称：Read_Data2(unsigned char addr, unsigned int *Data)
 *  函数功能：读取串口偏移量数据，并存放在数组里面
 *  形    参：addr，传感器地址,*Data，指针
 *  输    出：无
 *  备    注：串口1的时候需要修改
 *			   	   
 ********************************************************************/    
void Read_Data2(u8 addr, u32 *Data)	     
{	
	unsigned char y=0;
	unsigned int Receive_data = 0;       //数据缓存区

	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x57);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, addr);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
///////////////////////////偏移量数值///////////////////////////////	
	while(1)
  {		
		if(Num == 5)
		{
			Num = 0;
			if(USART_RX_STA[0x04] == 0x04)  //判断帧尾0x04,否者不赋值
			{
				Receive_data = USART_RX_STA[2];
				Receive_data <<= 8;
				Receive_data |= USART_RX_STA[3];
				
				*Data = USART_RX_STA[1];		//位置数据赋值	  
				*(Data+1) = Receive_data;			//偏移量赋值	  
				break;
			}        
			break;
		}
    else
    {
      Delay_ms(1);y++;
      if(y==3) { Num = 0;break; }
    }		
	}
///////////////////////////偏移量数值///////////////////////////////	
}
/*********************************************************************
 *  函数名称：Read_Data3(unsigned char addr, unsigned int *Data)
 *  函数功能：读取串口模拟量数据，并存放在数组里面
 *  形    参：addr，传感器地址,*Data，指针
 *  输    出：无
 *  备    注：串口1的时候需要修改
 *			   	   
 ********************************************************************/    
void Read_Data3(u8 addr, u32 *Data)	     
{	
	unsigned char y=0,i;
	unsigned int Receive_data[ADC_N] = { 0 };       //数据缓存区,ADC_N个模拟值

	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x57);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, addr);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
///////////////////////////模拟量数值///////////////////////////////	
	while(1)
  {		
		if(Num == 26)
		{
			Num = 0;
			if(USART_RX_STA[0x19] == 0x19)  //判断帧尾0x19,否者不赋值
			{
				for(i=0;i<ADC_N;i++)
				{
					Receive_data[i] = USART_RX_STA[i*2+1];
					Receive_data[i] <<= 8;
					Receive_data[i] |= USART_RX_STA[i*2+2];
					*(Data+i) = Receive_data[i];			//模拟量赋值	  
				}
				break;
			}        
			break;
		}
    else
    {
      Delay_ms(1);y++;
      if(y==5) { Num = 0;break; }
    }		
	}
///////////////////////////模拟量数值///////////////////////////////	
}
/*********************************************************************
 *  函数名称：Read_Data4(unsigned char addr, unsigned int *Data)
 *  函数功能：读取串口全部数据，并存放在数组里面
 *  形    参：addr，传感器地址,*Data，指针
 *  输    出：无
 *  备    注：串口1的时候需要修改
 *			   	   
 ********************************************************************/    
void Read_Data4(u8 addr, u32 *Data)	     
{	
	unsigned char y=0,i;
	unsigned int Receive_data[1+ADC_N] = { 0 };       //数据缓存区,1个偏移量+ADC_N个模拟值

	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x57);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, addr);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
///////////////////////////全部数值///////////////////////////////	
	while(1)
  {		
		if(Num == 31)
		{
			Num = 0;
			if(USART_RX_STA[0x1E] == 0x1E)  //判断帧尾0x1E,否者不赋值
			{
				*Data = USART_RX_STA[1];			//数字量赋值	  
				*(Data+1) = USART_RX_STA[2];			//位置赋值	  
				for(i=0;i<(1+ADC_N);i++)
				{
					Receive_data[i] = USART_RX_STA[i*2+3];
					Receive_data[i] <<= 8;
					Receive_data[i] |= USART_RX_STA[i*2+4];
					*(Data+2+i) = Receive_data[i];			//1个偏移量+7个模拟值赋值		  
				}
				break;
			}        
			break;
		}
    else
    {
      Delay_ms(1);y++;
      if(y==5) { Num = 0;break; }
    }		
	}
///////////////////////////全部数值///////////////////////////////	
}
/*********************************************************************
 *  函数名称：Set_Data
 *  函数功能：设置功能参数
 *  形    参：add:寄存器地址,Par1: 参数1,Par2: 参数2
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Set_Data1(u8 add,u8 Par1,u8 Par2)	       
{	
///////////////////////////设置功能参数///////////////////////////////	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x4C);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, add);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, Par1);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, Par2);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
//	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
//	USART_SendData(USART3, Par3);															//参数3,Par3:设置中线参考值的时候才会用到
//	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

///////////////////////////设置功能参数///////////////////////////////		
}
/*********************************************************************
 *  函数名称：void USART3_IRQHandler
 *  函数功能：串口3中断服务函数
 *  形    参：无
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_RX_STA[Num++] =USART_ReceiveData(USART3);	//读取接收到的数据
		if(USART_RX_STA[0] != 0x75) { Num = 0; }  //判断帧头0x75,否者重新接收
	} 	
}

