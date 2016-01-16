#include "pwm.h"

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	 
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 							//使能TIM4定时器模块时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  	//使能GPIO外设和AFIO复用功能模块时钟

    //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 		//TIM4_CH1 | TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  			//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//配置定时器基础设置
	TIM_TimeBaseStructure.TIM_Period = arr-1; 			//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 		//设置用来作为TIM4时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIM4的时间基数单位

	 //使能TIMx在ARR上的预装载寄存器
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	
	//配置定时器比较匹配输出设置，CH1																				   
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 	//选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; 					//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  			//根据TIM_OCInitStruct中指定的参数初始化外设TIM4
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  	//使能TIM4在CCR1上的预装载寄存器
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  			//根据TIM_OCInitStruct中指定的参数初始化外设TIM4
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
    
	//使能TIM4外设 
	TIM_Cmd(TIM4, ENABLE);  

}



