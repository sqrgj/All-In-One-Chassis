#include "timer.h"

//������ʱ���жϣ���������NVIC����˶�ʱ���ж��¼�ʵ�ʲ��ᷢ��
void Timer1_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		
	TIM_TimeBaseStructure.TIM_Period = 5000; 		//��ʱ����5ms
	TIM_TimeBaseStructure.TIM_Prescaler = (72-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);						 
}










