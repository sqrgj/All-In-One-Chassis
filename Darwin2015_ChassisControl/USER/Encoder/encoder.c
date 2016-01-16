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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;				 //ENCODER_CHA, ENCODER_CHB端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//预分频器
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 0xffff;//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

// 	TIM_ARRPreloadConfig(ENCODER_TIM, ENABLE);//使能ARR自动重装入缓冲器  

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStruct);
  	TIM_ICInitStruct.TIM_ICFilter = 6;//ICx_FILTER; //	TIM_ICFilter选择输入比较滤波器。该参数取值在0x0和0xF之间
	TIM_ICInit(TIM3, &TIM_ICInitStruct);
	
	TIM3->CNT = ENCODER_INIT_VALUE;

	//使能定时器
	TIM_Cmd(TIM3,ENABLE); 	//使能定时器
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

	GPIO_PinRemapConfig(/*GPIO_PartialRemap1_TIM2*/GPIO_FullRemap_TIM2,ENABLE); //重映射TIM2引脚

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //ENCODER_CHA, ENCODER_CHB端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //ENCODER_CHA, ENCODER_CHB端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;//预分频器
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 0xffff;//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

// 	TIM_ARRPreloadConfig(ENCODER_TIM, ENABLE);//使能ARR自动重装入缓冲器  

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStruct);
  	TIM_ICInitStruct.TIM_ICFilter = 6;//ICx_FILTER; //	TIM_ICFilter选择输入比较滤波器。该参数取值在0x0和0xF之间
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM2->CNT = ENCODER_INIT_VALUE;

	//使能定时器
	TIM_Cmd(TIM2,ENABLE); 	//使能定时器
}


void GetWheelSpeed(void)
{
	SpeedR = TIM_GetCounter(TIM3) - ENCODER_INIT_VALUE;
	TIM3->CNT = ENCODER_INIT_VALUE;

	SpeedL = TIM_GetCounter(TIM2) - ENCODER_INIT_VALUE;
	TIM2->CNT = ENCODER_INIT_VALUE;

}

