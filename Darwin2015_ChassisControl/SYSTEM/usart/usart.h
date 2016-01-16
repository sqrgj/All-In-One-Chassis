#ifndef __USART_H
#define __USART_H
#include "stdio.h"

////////////////////////////////////////////////////////////////////////////////// 

#define FRAMETYPE_MIN 0x01
#define FRAMETYPE_MAX 0x0A


extern u8 obstacle_flag;
extern u8 mode;
extern u8 charge_flag;

typedef enum 
{
	WAITRECEIVE 	= 0x00,		//接收的数据已经出来 等待接收
	WAITDATATYPE	= 0x01,		//等待指令类型
	WAITDATALENGTH 	= 0X02,		//等待接受字节长度
	WAITDATA 		= 0x03,		//数据长度接收成功 等待接收数据
	WAITCHECKSUM	= 0x04,	   	//等待接受checksum
	ENDRECEIVE 		= 0x05		//数据接收完毕
}CommuState;

typedef union 
{
	struct
	{
		s16 v;
		s16 w;
		u16 dt;
	}Odata;
	u8 Bdata[6];
}UnionCmdVel;

typedef union
{
	float fdata;
	u8 bdata[4];
}f2b;
	  
void uart_init(u32 bound);
void USART1_SendByte(u8 Data);
u8 USART1_ReceiveByte(void);
void ReturnOdoData(void);
void ProcessFrame(u8 frameType,u8* tmpDataRecv);

#endif
