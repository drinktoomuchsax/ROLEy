//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其他用途
//单片机交流群：439190573
//淘宝店：https://futureworldrobot.taobao.com
//修改日期：2023/6/5
//版本：V5
//版权所有，盗版必究。
//未来世界机器人系列
//////////////////////////////////////////////////////////////////////////////////
#include "iic.h"
#include "delay.h"
#include "Line.h"
/*********************************************************************
 *  函数名称：IIC_Init
 *  函数功能：初始化IIC
 *  形    参：无
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 	//PB6,PB7 输出高
}
/*********************************************************************
 *  函数名称：I2CStart
 *  函数功能：产生总线起始信号
 *  形    参：无
 *  输    出：无
 *  备    注：SCL线为高电平期间，SDA线由高电平向低电平的变化表示起始信号
 ********************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay_us(4);
 	IIC_SDA=0;//先拉低 SDA
	Delay_us(4);
	IIC_SCL=0;//再拉低 SCL
}	  
/*********************************************************************
 *  函数名称：IIC_Stop
 *  函数功能：产生总线停止信号
 *  形    参：无
 *  输    出：无
 *  备    注：SCL线为高电平期间，SDA线由低电平向高电平的变化表示终止信号
 ********************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//首先确保 SDA、 SCL 都是低电平
 	Delay_us(4);
	IIC_SCL=1; //先拉高 SCL
	IIC_SDA=1;//再拉高 SDA
	Delay_us(4);							   	
}
/*********************************************************************
 *  函数名称：IIC_Wait_Ack
 *  函数功能：等待应答信号到来
 *  形    参：无
 *  输    出：1，接收应答失败, 0，接收应答成功
 *  备    注：应答信号由接受设备产生，在SCL为高电平期间，接受设备将SDA拉低为低电平，表示数据传输正确，产生应答。 
 ********************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	Delay_us(3);	   
	IIC_SCL=1;		//此时刻开始，数据保持应答状态稳定
	Delay_us(3);	 
	while(READ_SDA) //读取此时的 SDA 值，即为从机的应答值
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
/*********************************************************************
 *  函数名称：IIC_Ack
 *  函数功能：产生ACK应答
 *  形    参：无
 *  输    出：无
 *  备    注：应答信号由接受设备产生，在SCL为高电平期间，接受设备将SDA拉低为低电平，表示数据传输正确，产生应答。 
 ********************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0; //8 位数据发送完后，拉低 SDA，发送应答信号
	Delay_us(3);
	IIC_SCL=1; //拉高 SCL
	Delay_us(3);
	IIC_SCL=0; //再拉低 SCL 完成应答位，并保持住总线
}
/*********************************************************************
 *  函数名称：IIC_NAck
 *  函数功能：不产生ACK应答
 *  形    参：无
 *  输    出：无
 *  备    注：应答信号由接受设备产生，在SCL为高电平期间，接受设备将SDA拉低为低电平，表示数据传输正确，产生应答。 
 ********************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;	//8 位数据发送完后，拉高 SDA，发送非应答信号
	Delay_us(3);
	IIC_SCL=1; //拉高 SCL
	Delay_us(3);
	IIC_SCL=0; //再拉低 SCL 完成非应答位，并保持住总线
}					 				     
/*********************************************************************
 *  函数名称：IIC_Write_Byte
 *  函数功能：I2C 总线写操作
 *  形    参：dat-待写入字节
 *  输    出：返回值-从机应答位的值
 *  备    注：数据位的有效性规定：IIC总线进行数据传送时，时钟信号为高电平期间，数据线上的数据必须保持稳定，
 *            只有在时钟线上的信号为低电平期间，数据线上的高电平或低电平状态才允许变化。
 ********************************************************************/			  
u8 IIC_Write_Byte(u8 dat)
{                        
  u8 mask; //用于探测字节内某一位值的掩码变量   
	
	SDA_OUT(); 	    
	IIC_SCL=0;//拉低时钟开始数据传输
	for (mask=0x80; mask!=0; mask>>=1) //从高位到低位依次进行
	{
		if ((mask&dat) == 0) //该位的值输出到 SDA 上
			IIC_SDA = 0;
		else
			IIC_SDA = 1; 
		Delay_us(3);   
		IIC_SCL=1; //拉高 SCL
		Delay_us(3); 
		IIC_SCL=0; //再拉低 SCL，完成一个位周期	
  }	 
	
	return IIC_Wait_Ack(); 
} 	    
/*********************************************************************
 *  函数名称：IIC_Read_Byte
 *  函数功能：I2C 总线读操作
 *  形    参：ack为0发送非应答信号，ack为1发送非应答信号
 *  输    出：返回值-读到的字节
 *  备    注：应答信号由接受设备产生，在SCL为高电平期间，接受设备将SDA拉低为低电平，表示数据传输正确，产生应答。
 *           数据位的有效性规定：IIC总线进行数据传送时，时钟信号为高电平期间，数据线上的数据必须保持稳定，
 *           只有在时钟线上的信号为低电平期间，数据线上的高电平或低电平状态才允许变化。
 ********************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
	unsigned char mask,dat=0;
	
	SDA_IN();//SDA设置为输入
	Delay_us(3);  
  for (mask=0x80; mask!=0; mask>>=1) //从高位到低位依次进行
	{
		Delay_us(3);       
		IIC_SCL=1; //拉高 SCL
		if(READ_SDA == 0) //读取 SDA 的值
			dat &= ~mask; //为 0 时， dat 中对应位清零
		else
			dat |= mask; //为 1 时， dat 中对应位置 1  
		Delay_us(3); 
		IIC_SCL = 0; //再拉低 SCL，以使从机发送出下一位
  }					 
	if (!ack)
		IIC_NAck();//发送nACK
	else
		IIC_Ack(); //发送ACK   
	
	return dat;
}
/*********************************************************************
 *  函数名称：Read_IICData1
 *  函数功能：读取的数字量数据
 *  形    参：addr：传感器ID地址,*Data:指针
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Read_IICData1(unsigned char addr, unsigned int *Data)
{
	unsigned int Receive_data[2] = { 0 };       //数据缓存区,2个字节数字量
	
	IIC_Start();		//开始
	IIC_Write_Byte(addr<<1); 	//写操作，从机地址	
	IIC_Write_Byte(0xA0); 	//寄存器地址	
	IIC_Stop();		//结束
	IIC_Start();		//开始
	IIC_Write_Byte((addr<<1)+1); 	//读操作，从机地址
	Receive_data[0] = IIC_Read_Byte(1); //读取一个字节数据+应答
	Receive_data[1] = IIC_Read_Byte(0); //读取一个字节数据+非应答

  IIC_Stop();		//结束

	Receive_data[1] <<= 8;
	Receive_data[1] |= Receive_data[0];

	*Data = Receive_data[1];       		//数字量赋值	
}
/*********************************************************************
 *  函数名称：Read_IICData2
 *  函数功能：读取的位置数据+偏移量数据
 *  形    参：addr：传感器ID地址,*Data:指针
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Read_IICData2(unsigned char addr, unsigned int *Data)
{
	unsigned int Receive_data[3] = { 0 };       //数据缓存区,1个字节位置数据+2个字节偏移量
	unsigned char i;
	
	IIC_Start();		//开始
	IIC_Write_Byte(addr<<1); 	//写操作，从机地址	
	IIC_Write_Byte(0xA1); 	//寄存器地址	
	IIC_Stop();		//结束
	IIC_Start();		//开始
	IIC_Write_Byte((addr<<1)+1); 	//读操作，从机地址
	for(i=0;i<3;i++)
	{
		if(i<2)
		{
			Receive_data[i] = IIC_Read_Byte(1); //读取一个字节数据+应答
		}
		else if(i==2)
		{
			Receive_data[i] = IIC_Read_Byte(0); //读取一个字节数据+非应答
		}
	}	
  IIC_Stop();		//结束

	Receive_data[1] <<= 8;
	Receive_data[1] |= Receive_data[2];

	*Data = Receive_data[0];       		//位置数据赋值	
	*(Data+1) = Receive_data[1];       //偏移量赋值	
}
/*********************************************************************
 *  函数名称：Read_IICData3
 *  函数功能：读取的模拟量数据
 *  形    参：addr：传感器ID地址,*Data:指针
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Read_IICData3(unsigned char addr, unsigned int *Data)
{
	unsigned int Receive_data[ADC_N*2] = { 0 };       //数据缓存区,ADC_N个模拟值ADC_N*2字节
	unsigned char i;
	
	IIC_Start();		//开始
	IIC_Write_Byte(addr<<1); 	//写操作，从机地址	
	IIC_Write_Byte(0xA2); 	//寄存器地址	
	IIC_Stop();		//结束
	IIC_Start();		//开始
	IIC_Write_Byte((addr<<1)+1); 	//读操作，从机地址
	for(i=0;i<ADC_N;i++)
	{
		if(i<(ADC_N-1))
		{
			Receive_data[i*2] = IIC_Read_Byte(1); //读取一个字节数据+应答
			Receive_data[i*2+1] = IIC_Read_Byte(1); //读取一个字节数据+应答
			Receive_data[i*2] <<= 8;
			Receive_data[i*2] |= Receive_data[i*2+1];
			*(Data+i) = Receive_data[i*2];       //模拟量赋值
		}
		else if(i==(ADC_N-1))
		{
			Receive_data[i*2] = IIC_Read_Byte(1); //读取一个字节数据+应答
			Receive_data[i*2+1] = IIC_Read_Byte(0); //读取一个字节数据+非应答
			Receive_data[i*2] <<= 8;
			Receive_data[i*2] |= Receive_data[i*2+1];
			*(Data+i) = Receive_data[i*2];       //模拟量赋值
		}
	}	
  IIC_Stop();		//结束
}
/*********************************************************************
 *  函数名称：Set_ID
 *  函数功能：设置ID地址
 *  形    参：add:当前地址,new_addr: 新地址
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Set_ID(unsigned char addr, unsigned char new_addr)
{
	IIC_Start();		//开始	
	IIC_Write_Byte(addr<<1); 	//写操作，从机地址		
	IIC_Write_Byte(0x10); 	//寄存器地址		
	IIC_Write_Byte(new_addr); //写入新地址数据	
	IIC_Stop();		//结束
}














