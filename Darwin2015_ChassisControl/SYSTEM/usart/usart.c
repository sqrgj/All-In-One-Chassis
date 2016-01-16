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
u8 frameLength[]={0,8,3,3,3,3,3,3,3,14,3};	//��ʼ��֡���ȣ�����Ϊ1��֡����Ϊ8
							//					����Ϊ2��֡����Ϊ3
							//					����Ϊ3��֡����Ϊ3
							//					����Ϊ4��֡����Ϊ3
							//					����Ϊ5��֡����Ϊ3
							//					����Ϊ6��֡����Ϊ3
							//					����Ϊ7��֡����Ϊ3
							//					����Ϊ8��֡����Ϊ3
							//					����Ϊ9��֡����Ϊ14
							//					����Ϊ10��֡����Ϊ3
u8 tmpDataRecv[20] = {0};//������ʱ����

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
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
    //GPIO�˿�����
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

   //Usart1 NVIC ����

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1
  
   //USART ��ʼ������
   
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
   

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
   
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 rs232RxByte = 0;
	static u8 checksum = 0;
	static u8 frameType = 0;
	static u8 dataRecevCount = 0;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		rs232RxByte = USART_ReceiveData(USART1); 	//��ȡ���յ�������
		
		switch(rxState)
		{
		 	case WAITRECEIVE:
				if(0xff == rs232RxByte) {rxState = WAITDATATYPE;}	//�Ѿ��յ�֡ͷ��һ���ֽ� �ȴ����յڶ��ֽ�
				break;
			case WAITDATATYPE:
				frameType = rs232RxByte;
				if( frameType>=FRAMETYPE_MIN && frameType<=FRAMETYPE_MAX) {rxState = WAITDATALENGTH;}//������ָ������
				else {rxState = WAITRECEIVE;}	  //֡ͷʧ�� �ص��ȴ�����״̬ 
 				break;
			case WAITDATALENGTH:
				if(frameLength[frameType] == rs232RxByte) //���������ݰ�����
				{
					rxState = WAITDATA; 
					dataRecevCount = 0;
				}
				else  //֡ͷʧ�� �ص��ȴ�����״̬
				{	
					rxState = WAITRECEIVE;
				}	   
 				break;
			case WAITDATA:	 				//�ȴ���������
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
		case 0x01: //���Ƶ����˶�
			//��������ע�⣡����������������ʱ�ȷ��͵�8λ���ٷ��͸�8λ������100=0x012c���ȷ���2c���ٷ���01
			for( tmpCount=0;tmpCount<6;tmpCount++ )
			{
				CmdVelData.Bdata[tmpCount] = tmpDataRecv[tmpCount];
			}
//			printf("data: v=%d, w=%d, dt=%d\n",CmdVelData.Odata.v, CmdVelData.Odata.w, CmdVelData.Odata.dt);
			//����ٶ���ת�ٵ�����
			if( CmdVelData.Odata.v > VMAX)  CmdVelData.Odata.v = VMAX;
			if( CmdVelData.Odata.v < -VMAX) CmdVelData.Odata.v = -VMAX;

			if( CmdVelData.Odata.w > WMAX)  CmdVelData.Odata.w = WMAX;
			if( CmdVelData.Odata.w < -WMAX) CmdVelData.Odata.w = -WMAX;

			if( obstacle_flag == 0 )
			{
				SetSpeed((float)CmdVelData.Odata.v, (float)CmdVelData.Odata.w);	 	//�����ٶ�����ٶ�
				OverTimeProtect = (u16)CmdVelData.Odata.dt;  						//�������ʱ�䱣��
			}
			break;
		case 0x02: //�ı����ģʽ
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
		case 0x03:	//���������½�������ɻ�����������Ӧ
			break;
		case 0x04: 	//�ı侱�����λ�ã��ɻ�����������Ӧ
			break;
		case 0x05:	//�ı�LED״̬���ɻ�����������Ӧ
			break;
		case 0x06:	//�ı�ͶӰ��״̬���ɻ�����������Ӧ
			break;
		case 0x07:	//�ı�����״̬���ɻ�����������Ӧ
			break;
		case 0x08:	//�ı����״̬���ɻ�����������Ӧ
			break;
		case 0x09:  //�ı����Odom
			//��ȡx��ֵ
			for(i=0;i<4;i++)
				temp.bdata[i] = tmpDataRecv[3-i];
			Odm.x = temp.fdata;
			
			//��ȡy��ֵ
			for(i=0;i<4;i++)
				temp.bdata[i] = tmpDataRecv[7-i];
			Odm.y = temp.fdata;
			
			//��ȡyaw��ֵ
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

//���ڲ�ѯ����
void USART1_SendByte(u8 Data)
{
	//�ȴ����ͻ�����Ϊ��
	while ( !( USART1->SR & USART_FLAG_TXE) )
	;
	//�����ݷ��뻺��������������
	USART_SendData(USART1, (u16)Data);
}

//���ڲ�ѯ����
u8  USART1_ReceiveByte(void)
{
	u8 Data;
	//�ȴ���������
	while ( !(USART1->SR & USART_FLAG_RXNE) )
	;
	//�ӻ������л�ȡ����������
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
	
	//��������ƣ�ÿ�ν��յ���λ��ָ��ʱ�ı�
	sentdata[3] = 0; 

	//x����
	temp.fdata = Odm.x;
	for(i=0;i<4;i++)
	{
		sentdata[7-i] = temp.bdata[i];
	}

	//y����
	temp.fdata = Odm.y;
	for(i=0;i<4;i++)
	{
		sentdata[11-i] = temp.bdata[i];
	}

	//�Ƕ�ֵ
	temp.fdata = Odm.theta;
	for(i=0;i<4;i++)
	{
		sentdata[15-i] = temp.bdata[i];
	}

	//���⴫����������0��1λΪfloor��������2~7λΪfront������
	sentdata[16] = IRs;
	
	//����λ��Ϊtouchpad׼��
	sentdata[17] = 0;
	
	//bumpers
	sentdata[18] = Bumpers;

	//����ģʽ
	sentdata[19] = mode;
	
	//����1����
	temp.fdata = US1_Dis;
	for(i=0;i<4;i++)
	{
		sentdata[23-i] = temp.bdata[i];
	}

	//����2����
	temp.fdata = US2_Dis;
	for(i=0;i<4;i++)
	{
		sentdata[27-i] = temp.bdata[i];
	}

	//��ѹֵ
	sentdata[28] = 0;//(u8)( ADC_Sample[0] & 0x00FF);
	sentdata[29] = 0;//(u8)( (ADC_Sample[0] & 0xFF00) >> 8 );

	status = temperature / 2;
	//״̬��Ϣ(�¶�)
	sentdata[30] = status;

	for(i=2;i<31;i++)
	{
		checksum += sentdata[i];
	}
	sentdata[31] = ~checksum;

	//��������
	for(i=0;i<32;i++)
	{
		USART1_SendByte(sentdata[i]);
	}

}
