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
	WAITRECEIVE 	= 0x00,		//���յ������Ѿ����� �ȴ�����
	WAITDATATYPE	= 0x01,		//�ȴ�ָ������
	WAITDATALENGTH 	= 0X02,		//�ȴ������ֽڳ���
	WAITDATA 		= 0x03,		//���ݳ��Ƚ��ճɹ� �ȴ���������
	WAITCHECKSUM	= 0x04,	   	//�ȴ�����checksum
	ENDRECEIVE 		= 0x05		//���ݽ������
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
