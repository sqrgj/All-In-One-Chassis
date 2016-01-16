#include "remote.h"
#include "delay.h"
 	  		  
u32 IR_Recv_Odr=0;  	 //�����ݴ洦
u8  IR_Recv_Rdy=0;    //������յ�����
IR_state status1 = NoHeadReceived; 

unsigned char m_IR_Addr = 0;
unsigned char m_IR_Data = 0;

Dock_Area Cur_Sta=N;
Dock_Area Pre_Sta=N;

EXTI_InitTypeDef EXTI_InitStructure;	//�ⲿ�ж���   

void Remote_Init(void)
{							 
	GPIO_InitTypeDef GPIO_InitStructure;	//GPIO
	NVIC_InitTypeDef NVIC_InitStructure;	//�ж�
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//��ʱ��
 		 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );	  
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);  
 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource14);  	//ѡ��PA1���ڵ�GPIO�ܽ������ⲿ�ж���·EXIT1		
 
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;	//�ⲿ��·EXIT14
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//�����ⲿ�ж�ģʽ:EXTI��·Ϊ�ж�����  EXTI_Mode_Event ;//���� EXTI��·Ϊ�¼����� 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //�ⲿ�жϴ�����ѡ��:����������·���������½���Ϊ�ж�����
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		//ʹ���ⲿ�ж���״̬
	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���	
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2��     // NKJ 20141021
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	//���ö�ʱ���������ж��ĸ����ⱻ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ�������10ms
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  1Mhz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_Cmd(TIM7, ENABLE);  //��TIM7
}

void Enable_Exit14_IRQ(void)
{
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;		//ʹ���ⲿ�ж���״̬
	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���	
}

void Disable_Exit14_IRQ(void)
{
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		//ʹ���ⲿ�ж���״̬
	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
}   
		   
void EXTI15_10_IRQHandler(void)
{       
	u16 res = 0;
	static u8 bitcount = 0;
			 
	if(RDATA)  	//������������ж�
	{
		TIM_SetCounter(TIM7,0);		//��ʱ������
	}
	else if( RDATA == 0)	 //������½����ж�
	{
		res = TIM_GetCounter(TIM7);	//��ȡ��ʱ����Ϣ

		switch(status1)
		{
			case NoHeadReceived:
				if(res>=4000 && res<5000)//ǰ��λ
				{
					status1 = WaitData;
					IR_Recv_Odr = 0;
					bitcount = 0;	
				}
				else
				{
					status1 = NoHeadReceived;
				}
				break;
			case WaitData:
				if( res>=1000 && res<2000 )	//�Ե͵�ƽΪ0.565ms���ߵ�ƽ1.685ms������Ϊ2.25ms����ϱ�ʾ�����Ƶġ�1��
				{
					bitcount++;
					IR_Recv_Odr<<=1;
            		IR_Recv_Odr += 1;
				}
				else if( res>=200 && res<1000 )		//�Ե͵�ƽΪ0.565ms���ߵ�ƽ0.56ms������Ϊ1.125ms����ϱ�ʾ�����Ƶġ�0��
				{
					bitcount++;
					IR_Recv_Odr<<=1;
            		IR_Recv_Odr += 0;
				}
				else
				{
					status1 = NoHeadReceived;
				}
				if( 32 == bitcount ) 
				{
					status1 = WaitStopBit;
				}
				break;
			case WaitStopBit:
				if(res>=18000 && res<22000) //���������
				{
					status1 = WaitStopBit2;//���ܵ�����	
				}
				else
				{
					status1 = NoHeadReceived;	
				}
				break;
			case WaitStopBit2:
				if(res>=2000 && res<4000) //���������
				{
					status1 = DataComplete;//���ܵ�����	
				}
				else
				{
					status1 = NoHeadReceived;	
				}
				break;
			default:
				break;
		}
	}	 	    
	EXTI_ClearITPendingBit(EXTI_Line14);  //���EXTI14��·����λ         
} 

//return 1: Valid; 0: Invalid
unsigned char IR_Recv_Process(unsigned char* Addr, unsigned char* Data)
{               
    u8 t1,t2;   
    t1=(IR_Recv_Odr>>24)&0xff; //�õ���ַ��
    t2=(IR_Recv_Odr>>16)&0xff;//�õ���ַ���� 
    status1 = NoHeadReceived;	//�����־λ 
	
    if(t1==(u8)~t2)//����ң��ʶ����(ID)
    { 
        *Addr = t1;
        t1=(IR_Recv_Odr>>8)&0xff;
        t2=(IR_Recv_Odr)&0xff;
        if(t1==(u8)~t2)
        {
            *Data = t1;
            return 1; // Valid
        }
        else
        {
            return 0; // Invalid
        }
    }     
    else
    {
        return 0; // Invalid
    }
}






























