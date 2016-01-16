#include "sensors.h"
#include "sys.h"
#include "delay.h"
#include "led.h"

u8 IRs = 0xFF;

float US1_Dis = 10000.0;
float US2_Dis = 10000.0;
BOOL IRQ_flag1 = FALSE;
BOOL IRQ_flag2 = FALSE;

u8 Bumpers = 0xFF;

EXTI_InitTypeDef EXTI_InitStructure1;
EXTI_InitTypeDef EXTI_InitStructure2;

//IR、US、Bumper初始化
void Sensors_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//-----------------------------IR sensors----------------------------------------------//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);   //使能PD端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;	    		 //PD.0, 1, 2, 4, 5 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 							//使能TIM5定时器模块时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  	//使能GPIO外设和AFIO复用功能模块时钟

    //设置该引脚为复用输出功能,输出TIM5 CH2的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 		//TIM5_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  			//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置定时器基础设置
	TIM_TimeBaseStructure.TIM_Period = 1894; 			//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =0; 		//设置用来作为TIM4时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIM4的时间基数单位

	 //使能TIMx在ARR上的预装载寄存器
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	
	//配置定时器比较匹配输出设置，CH1																				   
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 	//选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; 					//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  			//根据TIM_OCInitStruct中指定的参数初始化外设TIM4
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR2上的预装载寄存器

	//使能TIMx在ARR上的预装载寄存器
	TIM_ARRPreloadConfig(TIM5, ENABLE);
    
	//使能TIM5外设 
	TIM_Cmd(TIM5, ENABLE); 
	
	TIM_SetCompare2(TIM5, 947);	

	//-----------------------------US sensors----------------------------------------------//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);	 //使能PE端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;				//PE0->trig1 PE2->trig2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 					//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;				//PE1->echo1 PE3->echo2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 		//浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//配置定时器，从来计算超声距离
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5800为5.8ms，对应超声距离为1m
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_Cmd(TIM6, DISABLE);  //关闭TIM6

	//GPIOE.1 中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource1);

  	EXTI_InitStructure1.EXTI_Line=EXTI_Line1;
  	EXTI_InitStructure1.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure1.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure1.EXTI_LineCmd = DISABLE;
  	EXTI_Init(&EXTI_InitStructure1);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;					//使能echo1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;		//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;				//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);

	//GPIOE.1 中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);

  	EXTI_InitStructure2.EXTI_Line=EXTI_Line3;
  	EXTI_InitStructure2.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure2.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure2.EXTI_LineCmd = DISABLE;
  	EXTI_Init(&EXTI_InitStructure2);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;					//使能echo1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;		//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;				//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);
	
	//-----------------------------Bumpers----------------------------------------------//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;				//Bumper1~6 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 		//浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

}

/*---------------------------------------------Infrared Sensors-------------------------------------*/
void GetIRSensors(void)
{
	static u8 cnt = 0;

	switch(cnt)
	{
	case 0: 
		PDout(0) = 0;
		PDout(1) = 0;
		PDout(2) = 0;
		break;
	case 40:
		if(0 == PDin(8) ) IRs &= ~(1<<0);
		else IRs |= (1<<0);
 
		PDout(0) = 1;
		PDout(1) = 0;
		PDout(2) = 0;

		break;
	case 80: 
		if(0 == PDin(9) ) IRs &= ~(1<<1);
		else IRs |= (1<<1);

		PDout(0) = 0;
		PDout(1) = 1;
		PDout(2) = 0;
	
		break;
	case 120: 
		if(0 == PDin(10) ) IRs &= ~(1<<2);
		else IRs |= (1<<2);

		PDout(0) = 1;
		PDout(1) = 1;
		PDout(2) = 0;
		
		break;
	case 160:
		if(0 == PDin(11) ) IRs &= ~(1<<3);
		else IRs |= (1<<4);
		 
		PDout(0) = 0;
		PDout(1) = 0;
		PDout(2) = 1;

		break;
	case 200: 
		if(0 == PDin(12) ) IRs &= ~(1<<5);
		else IRs |= (1<<5);

		PDout(0) = 1;
		PDout(1) = 0;
		PDout(2) = 1;
	
		break;
	case 240: 
		if(0 == PDin(13) ) IRs &= ~(1<<6);
		else IRs |= (1<<6);

		PDout(0) = 0;
		PDout(1) = 1;
		PDout(2) = 1;
	
		break;
	case 280:
		if(0 == PDin(14) ) IRs &= ~(1<<0);
		else IRs |= (1<<0);
		 
		PDout(0) = 1;
		PDout(1) = 1;
		PDout(2) = 1;
		
		break;
	case 319:
		if(0 == PDin(15) ) IRs &= ~(1<<7);
		else IRs |= (1<<7);
		break;
	default:
		break;
	}
	cnt++;
	if(320 == cnt) cnt = 0;					   
}

/*---------------------------------------------UltraSonic 1-------------------------------------*/
void TrigUS1(void)
{
	//send trig signal
	TIM_Cmd(TIM6, ENABLE);  	//使能TIM1
	US1_TRIG = 1;
	delay_us(10);
	US1_TRIG = 0;

	//等待Echo变低
	TIM_SetCounter(TIM6,0);		//定时器清零
	while( (1 == US1_ECHO) && ( TIM_GetCounter(TIM6) < 10 ) ) ;	//wait for the logical high of echo，带有定时保护10us
	
	//等待Echo变高
	TIM_SetCounter(TIM6,0);		//定时器清零
	while( (0 == US1_ECHO) && ( TIM_GetCounter(TIM6) < 1000 ) ) ;	//wait for the logical high of echo，带有定时保护1ms

	//数Echo高电平的时间
	EXTI_InitStructure1.EXTI_LineCmd = ENABLE;	//使能外部中断
  	EXTI_Init(&EXTI_InitStructure1);	
	TIM_SetCounter(TIM6,0);		//定时器清零
}

void GetUS1Data(void)
{
	TIM_Cmd(TIM6, DISABLE);  	//关闭TIM1
	EXTI_InitStructure1.EXTI_LineCmd = DISABLE;	//关闭外部中断
  	EXTI_Init(&EXTI_InitStructure1);
	
	if( (FALSE == IRQ_flag1) || (US1_Dis < 3.0) )	US1_Dis = 10000.0;	//如果未采集到超声，则设为最大值10m
	IRQ_flag1 = FALSE;		//清标志位
}

void EXTI1_IRQHandler(void)
{
  	if(EXTI_GetITStatus(EXTI_Line1) != RESET)	  //检查指定的EXTI1线路触发请求发生与否
	{	  
		US1_Dis = (float)TIM_GetCounter(TIM6)/5.8;
		IRQ_flag1 = TRUE;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);  //清除EXTI1线路挂起位
}

/*---------------------------------------------UltraSonic 2-------------------------------------*/
void TrigUS2(void)
{
	//send trig signal
	TIM_Cmd(TIM6, ENABLE);  	//使能TIM1
	US2_TRIG = 1;
	delay_us(10);
	US2_TRIG = 0;

	//等待Echo角变低
	TIM_SetCounter(TIM6,0);		//定时器清零
	while( (1 == US2_ECHO) && ( TIM_GetCounter(TIM6) < 10 ) ) ;	//wait for the logical high of echo，带有定时保护10us
	
	//等待Echo角变高
	TIM_SetCounter(TIM6,0);		//定时器清零
	while( (0 == US2_ECHO) && ( TIM_GetCounter(TIM6) < 1000 ) ) ;	//wait for the logical high of echo，带有定时保护1ms
	
	//数echo高电平的时间
	EXTI_InitStructure2.EXTI_LineCmd = ENABLE;	//使能外部中断
  	EXTI_Init(&EXTI_InitStructure2);
	TIM_SetCounter(TIM6,0);		//定时器清零
}

void GetUS2Data(void)
{
	TIM_Cmd(TIM6, DISABLE);  	//关闭TIM1
	EXTI_InitStructure2.EXTI_LineCmd = DISABLE;	//关闭外部中断
  	EXTI_Init(&EXTI_InitStructure2);
	
	if( (FALSE == IRQ_flag2) || (US2_Dis < 3.0) )	US2_Dis = 10000.0;	//如果未采集到超声，则设为最大值10m
	IRQ_flag2 = FALSE;		//清标志位

}

void EXTI3_IRQHandler(void)
{
  	if(EXTI_GetITStatus(EXTI_Line3) != RESET)	  //检查指定的EXTI3线路触发请求发生与否
	{	  
		US2_Dis = (float)TIM_GetCounter(TIM6)/5.8;
		IRQ_flag2 = TRUE;
	}
	EXTI_ClearITPendingBit(EXTI_Line3);  //清除EXTI3线路挂起位
}

/*---------------------------------------------Bumpers-------------------------------------*/
void GetBumpers(void)
{
	if(BMP1 == 0) 	Bumpers &= ~(1<<0);
	else			Bumpers |= (1<<0);
	
	if(BMP2 == 0) 	Bumpers &= ~(1<<1);
	else			Bumpers |= (1<<1);
	
	if(BMP3 == 0) 	Bumpers &= ~(1<<2);
	else			Bumpers |= (1<<2);
	
	if(BMP4 == 0) 	Bumpers &= ~(1<<3);
	else			Bumpers |= (1<<3);
	
	if(BMP5 == 0) 	Bumpers &= ~(1<<4);
	else			Bumpers |= (1<<4);
	
	if(BMP6 == 0) 	Bumpers &= ~(1<<5);
	else			Bumpers |= (1<<5);  
}
