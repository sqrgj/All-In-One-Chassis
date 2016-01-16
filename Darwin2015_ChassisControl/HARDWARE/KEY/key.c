#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"

void KEY_Init(void) //IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//��ʼ��KEY0-->GPIOA.4  KEY1-->GPIOA.5��������
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
											  
}
u8 KEY_Scan(void)
{	 
	static u8 key_up=1;//�������ɿ���־	
 
	if( key_up && (KEY==0) )
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY==0)
		{ 
			return 1;
		}
	}else if(KEY==1)
		key_up=1; 	    
 
	return 0;// �ް�������
}
