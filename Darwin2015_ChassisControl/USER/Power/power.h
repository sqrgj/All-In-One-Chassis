#ifndef __POWER_H
#define __POWER_H

#include "stm32f10x.h"

#define PERIPHERAL_PWR_ON 	GPIO_SetBits(GPIOC, GPIO_Pin_10) 
#define PERIPHERAL_PWR_OFF 	GPIO_ResetBits(GPIOC, GPIO_Pin_10)

extern u16 ADC_Sample[2];

void PWR_Init(void);
float PWR_Monitor(void);

#endif
