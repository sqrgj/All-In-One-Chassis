#ifndef __DS18B20_H
#define __DS18B20_H 
#include "sys.h"   

extern s16 temperature;

//IO��������
//Ϊ�˾�ȷ�����������üĴ���д��
#define DS18B20_IO_IN()  {GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=8<<0;}
#define DS18B20_IO_OUT() {GPIOB->CRL&=0XFFFFFFF0;GPIOB->CRL|=3<<0;}
////IO��������											   
#define	DS18B20_DQ_OUT PBout(0) //���ݶ˿�	PB0 
#define	DS18B20_DQ_IN  PBin(0)  //���ݶ˿�	PB0 
   	
u8 DS18B20_Init(void);//��ʼ��DS18B20
void DS18B20_Start_Convert(void);
short DS18B20_Get_Temp(void);//��ȡ�¶�
void DS18B20_Start(void);//��ʼ�¶�ת��
void DS18B20_Write_Byte(u8 dat);//д��һ���ֽ�
u8 DS18B20_Read_Byte(void);//����һ���ֽ�
u8 DS18B20_Read_Bit(void);//����һ��λ
u8 DS18B20_Check(void);//����Ƿ����DS18B20
void DS18B20_Rst(void);//��λDS18B20    
#endif















