#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"

void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化KEY0-->GPIOA.4  KEY1-->GPIOA.5上拉输入
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
											  
}
u8 KEY_Scan(void)
{	 
	static u8 key_up=1;//按键按松开标志	
 
	if( key_up && (KEY==0) )
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY==0)
		{ 
			return 1;
		}
	}else if(KEY==1)
		key_up=1; 	    
 
	return 0;// 无按键按下
}
