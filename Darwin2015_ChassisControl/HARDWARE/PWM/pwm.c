#include "pwm.h"

//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	 
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 							//ʹ��TIM4��ʱ��ģ��ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  	//ʹ��GPIO�����AFIO���ù���ģ��ʱ��

    //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 		//TIM4_CH1 | TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  			//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//���ö�ʱ����������
	TIM_TimeBaseStructure.TIM_Period = arr-1; 			//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 		//����������ΪTIM4ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 		//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIM4��ʱ�������λ

	 //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	
	//���ö�ʱ���Ƚ�ƥ��������ã�CH1																				   
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 	//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; 					//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  			//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIM4
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  	//ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  			//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIM4
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
    
	//ʹ��TIM4���� 
	TIM_Cmd(TIM4, ENABLE);  

}



