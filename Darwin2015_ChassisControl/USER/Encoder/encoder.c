#include "encoder.h"
#include "motor.h"

s16 SpeedL = 0, SpeedR = 0;
		 
void EncoderL_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	/* ENCODER clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable the ENCODER Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;				 //ENCODER_CHA, ENCODER_CHB�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = 0xffff;//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

// 	TIM_ARRPreloadConfig(ENCODER_TIM, ENABLE);//ʹ��ARR�Զ���װ�뻺����  

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStruct);
  	TIM_ICInitStruct.TIM_ICFilter = 6;//ICx_FILTER; //	TIM_ICFilterѡ������Ƚ��˲������ò���ȡֵ��0x0��0xF֮��
	TIM_ICInit(TIM3, &TIM_ICInitStruct);
	
	TIM3->CNT = ENCODER_INIT_VALUE;

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM3,ENABLE); 	//ʹ�ܶ�ʱ��
}

void EncoderR_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	/* ENCODER clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Enable the ENCODER Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(/*GPIO_PartialRemap1_TIM2*/GPIO_FullRemap_TIM2,ENABLE); //��ӳ��TIM2����

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //ENCODER_CHA, ENCODER_CHB�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //ENCODER_CHA, ENCODER_CHB�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = 0xffff;//�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

// 	TIM_ARRPreloadConfig(ENCODER_TIM, ENABLE);//ʹ��ARR�Զ���װ�뻺����  

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStruct);
  	TIM_ICInitStruct.TIM_ICFilter = 6;//ICx_FILTER; //	TIM_ICFilterѡ������Ƚ��˲������ò���ȡֵ��0x0��0xF֮��
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM2->CNT = ENCODER_INIT_VALUE;

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM2,ENABLE); 	//ʹ�ܶ�ʱ��
}


void GetWheelSpeed(void)
{
	SpeedR = TIM_GetCounter(TIM3) - ENCODER_INIT_VALUE;
	TIM3->CNT = ENCODER_INIT_VALUE;

	SpeedL = TIM_GetCounter(TIM2) - ENCODER_INIT_VALUE;
	TIM2->CNT = ENCODER_INIT_VALUE;

}

