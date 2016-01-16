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

//IR��US��Bumper��ʼ��
void Sensors_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//-----------------------------IR sensors----------------------------------------------//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);   //ʹ��PD�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;	    		 //PD.0, 1, 2, 4, 5 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 							//ʹ��TIM5��ʱ��ģ��ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  	//ʹ��GPIO�����AFIO���ù���ģ��ʱ��

    //���ø�����Ϊ�����������,���TIM5 CH2��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 		//TIM5_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  			//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//���ö�ʱ����������
	TIM_TimeBaseStructure.TIM_Period = 1894; 			//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =0; 		//����������ΪTIM4ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 		//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIM4��ʱ�������λ

	 //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	
	//���ö�ʱ���Ƚ�ƥ��������ã�CH1																				   
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 	//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 					//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  			//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIM4
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR2�ϵ�Ԥװ�ؼĴ���

	//ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM5, ENABLE);
    
	//ʹ��TIM5���� 
	TIM_Cmd(TIM5, ENABLE); 
	
	TIM_SetCompare2(TIM5, 947);	

	//-----------------------------US sensors----------------------------------------------//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);	 //ʹ��PE�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;				//PE0->trig1 PE2->trig2 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 					//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;				//PE1->echo1 PE3->echo2 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 		//��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//���ö�ʱ�����������㳬������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5800Ϊ5.8ms����Ӧ��������Ϊ1m
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_Cmd(TIM6, DISABLE);  //�ر�TIM6

	//GPIOE.1 �ж����Լ��жϳ�ʼ������
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource1);

  	EXTI_InitStructure1.EXTI_Line=EXTI_Line1;
  	EXTI_InitStructure1.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure1.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure1.EXTI_LineCmd = DISABLE;
  	EXTI_Init(&EXTI_InitStructure1);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;					//ʹ��echo1���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;		//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;				//�����ȼ�1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);

	//GPIOE.1 �ж����Լ��жϳ�ʼ������
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);

  	EXTI_InitStructure2.EXTI_Line=EXTI_Line3;
  	EXTI_InitStructure2.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure2.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure2.EXTI_LineCmd = DISABLE;
  	EXTI_Init(&EXTI_InitStructure2);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;					//ʹ��echo1���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;		//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;				//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);
	
	//-----------------------------Bumpers----------------------------------------------//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;				//Bumper1~6 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 		//��������
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
	TIM_Cmd(TIM6, ENABLE);  	//ʹ��TIM1
	US1_TRIG = 1;
	delay_us(10);
	US1_TRIG = 0;

	//�ȴ�Echo���
	TIM_SetCounter(TIM6,0);		//��ʱ������
	while( (1 == US1_ECHO) && ( TIM_GetCounter(TIM6) < 10 ) ) ;	//wait for the logical high of echo�����ж�ʱ����10us
	
	//�ȴ�Echo���
	TIM_SetCounter(TIM6,0);		//��ʱ������
	while( (0 == US1_ECHO) && ( TIM_GetCounter(TIM6) < 1000 ) ) ;	//wait for the logical high of echo�����ж�ʱ����1ms

	//��Echo�ߵ�ƽ��ʱ��
	EXTI_InitStructure1.EXTI_LineCmd = ENABLE;	//ʹ���ⲿ�ж�
  	EXTI_Init(&EXTI_InitStructure1);	
	TIM_SetCounter(TIM6,0);		//��ʱ������
}

void GetUS1Data(void)
{
	TIM_Cmd(TIM6, DISABLE);  	//�ر�TIM1
	EXTI_InitStructure1.EXTI_LineCmd = DISABLE;	//�ر��ⲿ�ж�
  	EXTI_Init(&EXTI_InitStructure1);
	
	if( (FALSE == IRQ_flag1) || (US1_Dis < 3.0) )	US1_Dis = 10000.0;	//���δ�ɼ�������������Ϊ���ֵ10m
	IRQ_flag1 = FALSE;		//���־λ
}

void EXTI1_IRQHandler(void)
{
  	if(EXTI_GetITStatus(EXTI_Line1) != RESET)	  //���ָ����EXTI1��·�������������
	{	  
		US1_Dis = (float)TIM_GetCounter(TIM6)/5.8;
		IRQ_flag1 = TRUE;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);  //���EXTI1��·����λ
}

/*---------------------------------------------UltraSonic 2-------------------------------------*/
void TrigUS2(void)
{
	//send trig signal
	TIM_Cmd(TIM6, ENABLE);  	//ʹ��TIM1
	US2_TRIG = 1;
	delay_us(10);
	US2_TRIG = 0;

	//�ȴ�Echo�Ǳ��
	TIM_SetCounter(TIM6,0);		//��ʱ������
	while( (1 == US2_ECHO) && ( TIM_GetCounter(TIM6) < 10 ) ) ;	//wait for the logical high of echo�����ж�ʱ����10us
	
	//�ȴ�Echo�Ǳ��
	TIM_SetCounter(TIM6,0);		//��ʱ������
	while( (0 == US2_ECHO) && ( TIM_GetCounter(TIM6) < 1000 ) ) ;	//wait for the logical high of echo�����ж�ʱ����1ms
	
	//��echo�ߵ�ƽ��ʱ��
	EXTI_InitStructure2.EXTI_LineCmd = ENABLE;	//ʹ���ⲿ�ж�
  	EXTI_Init(&EXTI_InitStructure2);
	TIM_SetCounter(TIM6,0);		//��ʱ������
}

void GetUS2Data(void)
{
	TIM_Cmd(TIM6, DISABLE);  	//�ر�TIM1
	EXTI_InitStructure2.EXTI_LineCmd = DISABLE;	//�ر��ⲿ�ж�
  	EXTI_Init(&EXTI_InitStructure2);
	
	if( (FALSE == IRQ_flag2) || (US2_Dis < 3.0) )	US2_Dis = 10000.0;	//���δ�ɼ�������������Ϊ���ֵ10m
	IRQ_flag2 = FALSE;		//���־λ

}

void EXTI3_IRQHandler(void)
{
  	if(EXTI_GetITStatus(EXTI_Line3) != RESET)	  //���ָ����EXTI3��·�������������
	{	  
		US2_Dis = (float)TIM_GetCounter(TIM6)/5.8;
		IRQ_flag2 = TRUE;
	}
	EXTI_ClearITPendingBit(EXTI_Line3);  //���EXTI3��·����λ
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
