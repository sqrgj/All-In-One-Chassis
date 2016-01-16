#include "remote.h"
#include "delay.h"
 	  		  
u32 IR_Recv_Odr=0;  	 //命令暂存处
u8  IR_Recv_Rdy=0;    //红外接收到数据
IR_state status1 = NoHeadReceived; 

unsigned char m_IR_Addr = 0;
unsigned char m_IR_Data = 0;

Dock_Area Cur_Sta=N;
Dock_Area Pre_Sta=N;

EXTI_InitTypeDef EXTI_InitStructure;	//外部中断线   

void Remote_Init(void)
{							 
	GPIO_InitTypeDef GPIO_InitStructure;	//GPIO
	NVIC_InitTypeDef NVIC_InitStructure;	//中断
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//定时器
 		 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );	  
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);  
 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource14);  	//选择PA1所在的GPIO管脚用作外部中断线路EXIT1		
 
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;	//外部线路EXIT14
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//设外外部中断模式:EXTI线路为中断请求  EXTI_Mode_Event ;//设置 EXTI线路为事件请求 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //外部中断触发沿选择:设置输入线路上升沿与下降沿为中断请求
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		//使能外部中断新状态
	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器	
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //使能按键所在的外部中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级2级     // NKJ 20141021
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	//配置定时器，用来判断哪个红外被触发
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值，最长计数10ms
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); //设置用来作为TIMx时钟频率除数的预分频值  1Mhz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_Cmd(TIM7, ENABLE);  //打开TIM7
}

void Enable_Exit14_IRQ(void)
{
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;		//使能外部中断新状态
	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器	
}

void Disable_Exit14_IRQ(void)
{
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		//使能外部中断新状态
	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
}   
		   
void EXTI15_10_IRQHandler(void)
{       
	u16 res = 0;
	static u8 bitcount = 0;
			 
	if(RDATA)  	//如果是上升沿中断
	{
		TIM_SetCounter(TIM7,0);		//定时器清零
	}
	else if( RDATA == 0)	 //如果是下降沿中断
	{
		res = TIM_GetCounter(TIM7);	//读取定时器信息

		switch(status1)
		{
			case NoHeadReceived:
				if(res>=4000 && res<5000)//前导位
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
				if( res>=1000 && res<2000 )	//以低电平为0.565ms、高电平1.685ms、周期为2.25ms的组合表示二进制的“1”
				{
					bitcount++;
					IR_Recv_Odr<<=1;
            		IR_Recv_Odr += 1;
				}
				else if( res>=200 && res<1000 )		//以低电平为0.565ms、高电平0.56ms、周期为1.125ms的组合表示二进制的“0”
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
				if(res>=18000 && res<22000) //获得连发码
				{
					status1 = WaitStopBit2;//接受到数据	
				}
				else
				{
					status1 = NoHeadReceived;	
				}
				break;
			case WaitStopBit2:
				if(res>=2000 && res<4000) //获得连发码
				{
					status1 = DataComplete;//接受到数据	
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
	EXTI_ClearITPendingBit(EXTI_Line14);  //清除EXTI14线路挂起位         
} 

//return 1: Valid; 0: Invalid
unsigned char IR_Recv_Process(unsigned char* Addr, unsigned char* Data)
{               
    u8 t1,t2;   
    t1=(IR_Recv_Odr>>24)&0xff; //得到地址码
    t2=(IR_Recv_Odr>>16)&0xff;//得到地址反码 
    status1 = NoHeadReceived;	//清楚标志位 
	
    if(t1==(u8)~t2)//检验遥控识别码(ID)
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






























