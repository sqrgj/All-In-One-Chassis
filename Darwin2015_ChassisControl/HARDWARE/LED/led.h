#ifndef __LED_H
#define __LED_H	 
#include "stm32f10x.h"

//LED端口定义
#define LED0 PAout(8)// PA8
#define LED1 PAout(11)// PA11	

void LED_Init(void);//初始化

		 				    
#endif
