#include "pwm.h"
#include "motor.h"
#include "encoder.h"
#include "usart.h"

PID MotorLeft,MotorRight;
u8 rec_over_cur[OVR_CUR_TIM] = {0};

//��ʼ��PC6-9Ϊ�����.��ʹ�����ĸ��ڵ�ʱ��		    
//MOTOR_INT IO��ʼ��
void MOTOR_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//INT1-4��ӦPC6-9��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   //ʹ��PC�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_CTR_A|MOTOR_LEFT_CTR_B|MOTOR_RIGHT_CTR_B|MOTOR_RIGHT_CTR_A|GPIO_Pin_2|GPIO_Pin_3; 	//����������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	PCout(2) = 1;
	PCout(3) = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);   //ʹ��PE�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = OVER_CUR_1|OVER_CUR_2; 			//�������������ܹܽ�ʹ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//////////������ʼ��///////////////////
	MotorLeft.Ref = 0;			 	//�ο�����ֵ
	MotorLeft.FeedBack = 0;			//����ֵ
	MotorLeft.PreError = 0;			//ǰһ�����,ui_Ref - FeedBack
	MotorLeft.PreIntegral = 0;		//ǰһ�λ����ui_PreIntegral+ui			
	MotorLeft.Kp = KP_L;				//����ϵ��	
	MotorLeft.Ki = KI_L;				//����ϵ��	
	MotorLeft.Kd = KD_L;				//΢��ϵ��	
	MotorLeft.SpeedU = 0;			//����������ֵ
	
	MotorRight.Ref = 0;			 	//�ο�����ֵ
	MotorRight.FeedBack = 0;		//����ֵ
	MotorRight.PreError = 0;		//ǰһ�����,ui_Ref - FeedBack
	MotorRight.PreIntegral = 0;		//ǰһ�λ����ui_PreIntegral+ui			
	MotorRight.Kp = KP_R;				//����ϵ��	
	MotorRight.Ki = KI_R;				//����ϵ��	
	MotorRight.Kd = KD_R;				//΢��ϵ��	
	MotorRight.SpeedU = 0;			//����������ֵ	
 
}

void MOTOR_Sudden_Stop(void)
{
	GPIO_ResetBits(GPIOC,MOTOR_LEFT_CTR_A);  
	GPIO_ResetBits(GPIOC,MOTOR_LEFT_CTR_B); 
	GPIO_ResetBits(GPIOC,MOTOR_RIGHT_CTR_B); 
	GPIO_ResetBits(GPIOC,MOTOR_RIGHT_CTR_A);	
}

void Motor_Control(u8 motor, s16 speed) 
{ 
	if (speed > PWM_PERIOD) speed = PWM_PERIOD;	   			//����pwm���ֵ
	if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;      
	switch(motor)
	{
		case M_LEFT:  										//���������ٶ�
			if(speed >= 0)									//�ٶ�Ϊ��
			{
				GPIO_SetBits(GPIOC, MOTOR_LEFT_CTR_A);  
				GPIO_ResetBits(GPIOC,MOTOR_LEFT_CTR_B);
				TIM_SetCompare1(TIM4, speed);
			}
			else											//�ٶ�Ϊ��
			{
				GPIO_ResetBits(GPIOC,MOTOR_LEFT_CTR_A);  
				GPIO_SetBits(GPIOC,MOTOR_LEFT_CTR_B);
				TIM_SetCompare1(TIM4, -speed);
			}
			break;
		case M_RIGHT:  										//�����ҵ���ٶ�
			if(speed >= 0)	 								//�ٶ�Ϊ��
			{
				GPIO_SetBits(GPIOC,MOTOR_RIGHT_CTR_A);  
				GPIO_ResetBits(GPIOC,MOTOR_RIGHT_CTR_B);
				TIM_SetCompare2(TIM4, speed);
			}
			else 											//�ٶ�Ϊ��
			{
				GPIO_ResetBits(GPIOC,MOTOR_RIGHT_CTR_A);  
				GPIO_SetBits(GPIOC,MOTOR_RIGHT_CTR_B);
				TIM_SetCompare2(TIM4, -speed);
			}
			break;
		default:
			break;
	}
}

void CloseLoopMotorControl(void)
{
	s32 speed_error = 0;
	s32 speed_derror = 0;
//	static s32 Ditem = 0;
//	static u32 d_count = 100;

	/////////////////////////�����ٶȱջ�/////////////////////////////////////////////
	speed_error = MotorLeft.Ref - SpeedL;		// ƫ��
	if( ( speed_error < DEADLINE ) && ( speed_error > -DEADLINE ) )	//���õ�������
    {
    		
    }
	else//ִ��PID����
    {   			           	
    	speed_derror = speed_error - MotorLeft.PreError;	//����΢����ƫ��
    	
    	MotorLeft.PreIntegral += speed_error;				//�洢��ǰ����ƫ��
    	MotorLeft.PreError = speed_error;				//�洢��ǰƫ��	
    
    	if(MotorLeft.PreIntegral > IMAX)						//��������,�趨���������ޣ�
    		MotorLeft.PreIntegral = IMAX;        		
    	else if(MotorLeft.PreIntegral < -IMAX)       	
    		MotorLeft.PreIntegral = -IMAX;
		      		
//    	if( MotorLeft.PreIntegral>0 && speed_error <0 )	//����������������ʱ����
//    		MotorLeft.PreIntegral=0;       		
//    	else if( MotorLeft.PreIntegral<0 && speed_error >0 )
//    		MotorLeft.PreIntegral=0;

//		else if( MotorLeft.PreIntegral>0 && speed_derror <0 )	//�仯���Ƹı䣬���ּ���
//			MotorLeft.PreIntegral /= 2;       		
//		else if( MotorLeft.PreIntegral<0 && speed_derror >0 )
//			MotorLeft.PreIntegral /= 2;

//		Ditem *= 90;		//����ʱ����
//        Ditem /= 100;
//        		
//    	if(speed_derror != 0)
//    	{
//    		Ditem += (speed_derror * MotorLeft.Kd)/d_count;
//    		d_count = 0;
//    	}

		MotorLeft.SpeedU = MotorLeft.Kp * speed_error + MotorLeft.Ki * MotorLeft.PreIntegral + /*Ditem*/MotorLeft.Kd * speed_derror;	//λ��PID�㷨
		MotorLeft.SpeedU /= 10;	//���ڵ����ʵ������Χ

		//��ֹ�������
		if( MotorLeft.SpeedU >= SPEED_MAX ) 		
			MotorLeft.SpeedU = SPEED_MAX;
		else if( MotorLeft.SpeedU <= -SPEED_MAX ) 
			 MotorLeft.SpeedU = -SPEED_MAX;
	} 
	Motor_Control(M_LEFT,MotorLeft.SpeedU);

	/////////////////////////�ҵ���ٶȱջ�/////////////////////////////////////////////
	speed_error = MotorRight.Ref - SpeedR;		// ƫ��
	if( ( speed_error < DEADLINE ) && ( speed_error > -DEADLINE ) )	//���õ�������
    {
    		
    }
	else//ִ��PID����
    {   			           	
    	speed_derror = speed_error - MotorRight.PreError;	//����΢����ƫ��
    	
    	MotorRight.PreIntegral += speed_error;				//�洢��ǰ����ƫ��
    	MotorRight.PreError = speed_error;				//�洢��ǰƫ��	
    
    	if(MotorRight.PreIntegral > IMAX)						//��������,�趨���������ޣ�
    		MotorRight.PreIntegral = IMAX;        		
    	else if(MotorRight.PreIntegral < -IMAX)       	
    		MotorRight.PreIntegral = -IMAX;
		      		
//    	if( MotorRight.PreIntegral>0 && speed_error <0 )	//����������������ʱ����
//    		MotorRight.PreIntegral=0;       		
//    	else if( MotorRight.PreIntegral<0 && speed_error >0 )
//    		MotorRight.PreIntegral=0;

//		else if( MotorRight.PreIntegral>0 && speed_derror <0 )	//�仯���Ƹı䣬���ּ���
//			MotorRight.PreIntegral /= 2;       		
//		else if( MotorRight.PreIntegral<0 && speed_derror >0 )
//			MotorRight.PreIntegral /= 2;

//		Ditem *= 90;		//����ʱ����
//        Ditem /= 100;
//        		
//    	if(speed_derror != 0)
//    	{
//    		Ditem += (speed_derror * MotorRight.Kd)/d_count;
//    		d_count = 0;
//    	}

		MotorRight.SpeedU = MotorRight.Kp * speed_error + MotorRight.Ki * MotorRight.PreIntegral + /*Ditem*/MotorRight.Kd * speed_derror;	//λ��PID�㷨
		MotorRight.SpeedU /= 10;	//���ڵ����ʵ������Χ

		//��ֹ�������
		if( MotorRight.SpeedU >= SPEED_MAX ) 		
			MotorRight.SpeedU = SPEED_MAX;
		else if( MotorRight.SpeedU <= -SPEED_MAX ) 
			 MotorRight.SpeedU = -SPEED_MAX;
	} 
	Motor_Control(M_RIGHT,MotorRight.SpeedU);
}

//��ת��������
void OverCur_Protect(void)	
{
	static u8 cnt_over_cur = 0;
	u8 i = 0;
	u8 flag_over_cur = 0;

	if( PEin(6) == 0 || PEin(7) == 0 ) 	//����˲������ڵ�������������¼һ��
	{
		rec_over_cur[cnt_over_cur] = 1;	
	}
	else
	{
		rec_over_cur[cnt_over_cur] = 0;
	}
	cnt_over_cur++;						//��¼OVR_CUR_TIM�κ����㣬���¼���
	if( cnt_over_cur == OVR_CUR_TIM ) cnt_over_cur = 0;

	for(i=0;i<OVR_CUR_TIM;i++)				   //����OVR_CUR_TIM�μ�¼���м��ι�����
	{
		flag_over_cur += rec_over_cur[i];	
	}
	if( flag_over_cur >= 80 ) 			//����ڽ�OVR_CUR_TIM��100���μ�������80�����Ϲ���������Ϊ�����ת
	{
		MOTOR_Sudden_Stop();
		GPIO_SetBits(GPIOC, GPIO_Pin_1);	//�򿪷�����
		while(1);	
	}
		
}



