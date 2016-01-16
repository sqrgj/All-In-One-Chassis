#ifndef __RED_H
#define __RED_H 
#include "sys.h"   
  
#define RDATA PCin(2)	 //红外数据输入脚
#define Dock_Left_ID 0xAA
#define Dock_Right_ID 0x55

//红外遥控识别码(ID)
#define IR_Recv_ID 0      

typedef enum
{
	NoHeadReceived = 0x00,
	WaitData = 0x01,
	WaitStopBit = 0x02,
	DataComplete = 0x03,
	WaitStopBit2 = 0x04
}IR_state;

typedef enum {L=0,C,R,N}Dock_Area;

extern Dock_Area Cur_Sta;
extern Dock_Area Pre_Sta;

extern IR_state status1;
extern u32 IR_Recv_Odr;   //命令暂存处

extern unsigned char m_IR_Addr;
extern unsigned char m_IR_Data;

void Remote_Init(void);    //红外传感器接收头引脚初始化
void Enable_Exit14_IRQ(void);
void Disable_Exit14_IRQ(void);
unsigned char IR_Recv_Process(unsigned char* Addr, unsigned char* Data);
	
#endif















