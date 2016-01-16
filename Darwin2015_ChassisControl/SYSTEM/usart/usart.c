#include "sys.h"
#include "usart.h"
#include "robotcontrol.h"
#include "sensors.h"
#include "power.h"
#include "remote.h"
#include "ds18b20.h"
#include "mpu6050.h"

u8 rand_num = 0x00;
u8 charge_flag = 0x00;
u8 mode = 0x00;
u8 status = 0x00;
u8 obstacle_flag = 0;
CommuState rxState;
UnionCmdVel CmdVelData;
u8 frameLength[]={0,8,3,3,3,3,3,3,3,14,3};	//初始化帧长度，类型为1的帧长度为8
							//					类型为2的帧长度为3
							//					类型为3的帧长度为3
							//					类型为4的帧长度为3
							//					类型为5的帧长度为3
							//					类型为6的帧长度为3
							//					类型为7的帧长度为3
							//					类型为8的帧长度为3
							//					类型为9的帧长度为14
							//					类型为10的帧长度为3
u8 tmpDataRecv[20] = {0};//数据临时接收

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 


void uart_init(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
  
   //USART 初始化设置
   
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
   

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
   
    USART_Cmd(USART1, ENABLE);                    //使能串口 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 rs232RxByte = 0;
	static u8 checksum = 0;
	static u8 frameType = 0;
	static u8 dataRecevCount = 0;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		rs232RxByte = USART_ReceiveData(USART1); 	//读取接收到的数据
		
		switch(rxState)
		{
		 	case WAITRECEIVE:
				if(0xff == rs232RxByte) {rxState = WAITDATATYPE;}	//已经收到帧头第一个字节 等待接收第二字节
				break;
			case WAITDATATYPE:
				frameType = rs232RxByte;
				if( frameType>=FRAMETYPE_MIN && frameType<=FRAMETYPE_MAX) {rxState = WAITDATALENGTH;}//接受完指令类型
				else {rxState = WAITRECEIVE;}	  //帧头失败 回到等待接收状态 
 				break;
			case WAITDATALENGTH:
				if(frameLength[frameType] == rs232RxByte) //接受完数据包长度
				{
					rxState = WAITDATA; 
					dataRecevCount = 0;
				}
				else  //帧头失败 回到等待接收状态
				{	
					rxState = WAITRECEIVE;
				}	   
 				break;
			case WAITDATA:	 				//等待接受数据
				if( dataRecevCount < (frameLength[frameType]-1) )
				{
					tmpDataRecv[dataRecevCount] = rs232RxByte;
					dataRecevCount++;
				}
				if( dataRecevCount == (frameLength[frameType]-1) )
				{
					rxState = WAITCHECKSUM;
				}
				break;
			case WAITCHECKSUM:
				checksum = rs232RxByte;
				checksum += frameType;
				checksum += frameLength[frameType];
				for(;dataRecevCount>0;dataRecevCount--)
				{
					checksum += tmpDataRecv[dataRecevCount-1];
				}
				if( 0xff == checksum )
				{

					rand_num = tmpDataRecv[ frameLength[frameType]-2 ];
					ProcessFrame(frameType,tmpDataRecv);				
				}
				rxState = WAITRECEIVE;
			default :
				break;
		} 
	}
}

void ProcessFrame(u8 frameType,u8* tmpDataRecv)
{
	u8 tmpCount = 0;
	f2b temp;
	u16 i = 0;
	switch(frameType)
	{
		case 0x01: //控制底盘运动
			//！！！！注意！！！！，发送数据时先发送低8位，再发送高8位，例如100=0x012c，先发送2c，再发送01
			for( tmpCount=0;tmpCount<6;tmpCount++ )
			{
				CmdVelData.Bdata[tmpCount] = tmpDataRecv[tmpCount];
			}
//			printf("data: v=%d, w=%d, dt=%d\n",CmdVelData.Odata.v, CmdVelData.Odata.w, CmdVelData.Odata.dt);
			//最大速度与转速的限制
			if( CmdVelData.Odata.v > VMAX)  CmdVelData.Odata.v = VMAX;
			if( CmdVelData.Odata.v < -VMAX) CmdVelData.Odata.v = -VMAX;

			if( CmdVelData.Odata.w > WMAX)  CmdVelData.Odata.w = WMAX;
			if( CmdVelData.Odata.w < -WMAX) CmdVelData.Odata.w = -WMAX;

			if( obstacle_flag == 0 )
			{
				SetSpeed((float)CmdVelData.Odata.v, (float)CmdVelData.Odata.w);	 	//设置速度与角速度
				OverTimeProtect = (u16)CmdVelData.Odata.dt;  						//设置最大时间保护
			}
			break;
		case 0x02: //改变控制模式
			switch(tmpDataRecv[0])
			{
				case 0x00:
					mode = 0;
					Disable_Exit14_IRQ();
					break;
				case 0x01:
					mode = 1;
					Enable_Exit14_IRQ();
					OverTimeProtect = 60000;
					break;
				default:
					break;
			}
			charge_flag = 0x00;
			break;
		case 0x03:	//提升或者下降电机，由机构控制器响应
			break;
		case 0x04: 	//改变颈部电机位置，由机构控制器响应
			break;
		case 0x05:	//改变LED状态，由机构控制器响应
			break;
		case 0x06:	//改变投影仪状态，由机构控制器响应
			break;
		case 0x07:	//改变音箱状态，由机构控制器响应
			break;
		case 0x08:	//改变风扇状态，由机构控制器响应
			break;
		case 0x09:  //改变底盘Odom
			//获取x的值
			for(i=0;i<4;i++)
				temp.bdata[i] = tmpDataRecv[3-i];
			Odm.x = temp.fdata;
			
			//获取y的值
			for(i=0;i<4;i++)
				temp.bdata[i] = tmpDataRecv[7-i];
			Odm.y = temp.fdata;
			
			//获取yaw的值
			for(i=0;i<4;i++)
				temp.bdata[i] = tmpDataRecv[11-i];
			Odm.theta = temp.fdata;
			SetYawAngle(Odm.theta); 
			break;
		case 0x0A:
			charge_flag = tmpDataRecv[0];
			break;
		default:
			break;
	}
}

//串口查询发送
void USART1_SendByte(u8 Data)
{
	//等待发送缓冲器为空
	while ( !( USART1->SR & USART_FLAG_TXE) )
	;
	//将数据放入缓冲器，发送数据
	USART_SendData(USART1, (u16)Data);
}

//串口查询接受
u8  USART1_ReceiveByte(void)
{
	u8 Data;
	//等待接收数据
	while ( !(USART1->SR & USART_FLAG_RXNE) )
	;
	//从缓冲器中获取并返回数据
	Data = 	(u8)(USART_ReceiveData(USART1) & 0xFF);
	return  Data;
}

void ReturnOdoData(void)
{
	u8 checksum = 0x00;
	u8 sentdata[32]={0};
	u8 i = 0;
	f2b temp;

	sentdata[0] = 0xFF;
	sentdata[1] = 0xFF;
	sentdata[2]	= 29;
	
	//随机数令牌，每次接收到上位机指令时改变
	sentdata[3] = 0; 

	//x坐标
	temp.fdata = Odm.x;
	for(i=0;i<4;i++)
	{
		sentdata[7-i] = temp.bdata[i];
	}

	//y坐标
	temp.fdata = Odm.y;
	for(i=0;i<4;i++)
	{
		sentdata[11-i] = temp.bdata[i];
	}

	//角度值
	temp.fdata = Odm.theta;
	for(i=0;i<4;i++)
	{
		sentdata[15-i] = temp.bdata[i];
	}

	//红外传感器，其中0、1位为floor传感器，2~7位为front传感器
	sentdata[16] = IRs;
	
	//保留位，为touchpad准备
	sentdata[17] = 0;
	
	//bumpers
	sentdata[18] = Bumpers;

	//工作模式
	sentdata[19] = mode;
	
	//超声1距离
	temp.fdata = US1_Dis;
	for(i=0;i<4;i++)
	{
		sentdata[23-i] = temp.bdata[i];
	}

	//超声2距离
	temp.fdata = US2_Dis;
	for(i=0;i<4;i++)
	{
		sentdata[27-i] = temp.bdata[i];
	}

	//电压值
	sentdata[28] = 0;//(u8)( ADC_Sample[0] & 0x00FF);
	sentdata[29] = 0;//(u8)( (ADC_Sample[0] & 0xFF00) >> 8 );

	status = temperature / 2;
	//状态信息(温度)
	sentdata[30] = status;

	for(i=2;i<31;i++)
	{
		checksum += sentdata[i];
	}
	sentdata[31] = ~checksum;

	//发送数据
	for(i=0;i<32;i++)
	{
		USART1_SendByte(sentdata[i]);
	}

}
