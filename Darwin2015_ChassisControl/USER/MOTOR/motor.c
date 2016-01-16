#include "pwm.h"
#include "motor.h"
#include "encoder.h"
#include "usart.h"

PID MotorLeft,MotorRight;
u8 rec_over_cur[OVR_CUR_TIM] = {0};

//初始化PC6-9为输出口.并使能这四个口的时钟		    
//MOTOR_INT IO初始化
void MOTOR_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//INT1-4对应PC6-9、
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   //使能PC端口时钟
	
	GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_CTR_A|MOTOR_LEFT_CTR_B|MOTOR_RIGHT_CTR_B|MOTOR_RIGHT_CTR_A|GPIO_Pin_2|GPIO_Pin_3; 	//电机方向控制
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	PCout(2) = 1;
	PCout(3) = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);   //使能PE端口时钟
	GPIO_InitStructure.GPIO_Pin = OVER_CUR_1|OVER_CUR_2; 			//过电流保护功能管脚使能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//////////参数初始化///////////////////
	MotorLeft.Ref = 0;			 	//参考输入值
	MotorLeft.FeedBack = 0;			//反馈值
	MotorLeft.PreError = 0;			//前一次误差,ui_Ref - FeedBack
	MotorLeft.PreIntegral = 0;		//前一次积分项，ui_PreIntegral+ui			
	MotorLeft.Kp = KP_L;				//比例系数	
	MotorLeft.Ki = KI_L;				//积分系数	
	MotorLeft.Kd = KD_L;				//微分系数	
	MotorLeft.SpeedU = 0;			//电机控制输出值
	
	MotorRight.Ref = 0;			 	//参考输入值
	MotorRight.FeedBack = 0;		//反馈值
	MotorRight.PreError = 0;		//前一次误差,ui_Ref - FeedBack
	MotorRight.PreIntegral = 0;		//前一次积分项，ui_PreIntegral+ui			
	MotorRight.Kp = KP_R;				//比例系数	
	MotorRight.Ki = KI_R;				//积分系数	
	MotorRight.Kd = KD_R;				//微分系数	
	MotorRight.SpeedU = 0;			//电机控制输出值	
 
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
	if (speed > PWM_PERIOD) speed = PWM_PERIOD;	   			//限制pwm最大值
	if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;      
	switch(motor)
	{
		case M_LEFT:  										//设置左电机速度
			if(speed >= 0)									//速度为正
			{
				GPIO_SetBits(GPIOC, MOTOR_LEFT_CTR_A);  
				GPIO_ResetBits(GPIOC,MOTOR_LEFT_CTR_B);
				TIM_SetCompare1(TIM4, speed);
			}
			else											//速度为负
			{
				GPIO_ResetBits(GPIOC,MOTOR_LEFT_CTR_A);  
				GPIO_SetBits(GPIOC,MOTOR_LEFT_CTR_B);
				TIM_SetCompare1(TIM4, -speed);
			}
			break;
		case M_RIGHT:  										//设置右电机速度
			if(speed >= 0)	 								//速度为正
			{
				GPIO_SetBits(GPIOC,MOTOR_RIGHT_CTR_A);  
				GPIO_ResetBits(GPIOC,MOTOR_RIGHT_CTR_B);
				TIM_SetCompare2(TIM4, speed);
			}
			else 											//速度为负
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

	/////////////////////////左电机速度闭环/////////////////////////////////////////////
	speed_error = MotorLeft.Ref - SpeedL;		// 偏差
	if( ( speed_error < DEADLINE ) && ( speed_error > -DEADLINE ) )	//设置调节死区
    {
    		
    }
	else//执行PID调节
    {   			           	
    	speed_derror = speed_error - MotorLeft.PreError;	//计算微分项偏差
    	
    	MotorLeft.PreIntegral += speed_error;				//存储当前积分偏差
    	MotorLeft.PreError = speed_error;				//存储当前偏差	
    
    	if(MotorLeft.PreIntegral > IMAX)						//积分修正,设定积分上下限，
    		MotorLeft.PreIntegral = IMAX;        		
    	else if(MotorLeft.PreIntegral < -IMAX)       	
    		MotorLeft.PreIntegral = -IMAX;
		      		
//    	if( MotorLeft.PreIntegral>0 && speed_error <0 )	//积分项于正负换向时清零
//    		MotorLeft.PreIntegral=0;       		
//    	else if( MotorLeft.PreIntegral<0 && speed_error >0 )
//    		MotorLeft.PreIntegral=0;

//		else if( MotorLeft.PreIntegral>0 && speed_derror <0 )	//变化趋势改变，积分减半
//			MotorLeft.PreIntegral /= 2;       		
//		else if( MotorLeft.PreIntegral<0 && speed_derror >0 )
//			MotorLeft.PreIntegral /= 2;

//		Ditem *= 90;		//加延时因子
//        Ditem /= 100;
//        		
//    	if(speed_derror != 0)
//    	{
//    		Ditem += (speed_derror * MotorLeft.Kd)/d_count;
//    		d_count = 0;
//    	}

		MotorLeft.SpeedU = MotorLeft.Kp * speed_error + MotorLeft.Ki * MotorLeft.PreIntegral + /*Ditem*/MotorLeft.Kd * speed_derror;	//位置PID算法
		MotorLeft.SpeedU /= 10;	//调节到合适的输出范围

		//防止调节溢出
		if( MotorLeft.SpeedU >= SPEED_MAX ) 		
			MotorLeft.SpeedU = SPEED_MAX;
		else if( MotorLeft.SpeedU <= -SPEED_MAX ) 
			 MotorLeft.SpeedU = -SPEED_MAX;
	} 
	Motor_Control(M_LEFT,MotorLeft.SpeedU);

	/////////////////////////右电机速度闭环/////////////////////////////////////////////
	speed_error = MotorRight.Ref - SpeedR;		// 偏差
	if( ( speed_error < DEADLINE ) && ( speed_error > -DEADLINE ) )	//设置调节死区
    {
    		
    }
	else//执行PID调节
    {   			           	
    	speed_derror = speed_error - MotorRight.PreError;	//计算微分项偏差
    	
    	MotorRight.PreIntegral += speed_error;				//存储当前积分偏差
    	MotorRight.PreError = speed_error;				//存储当前偏差	
    
    	if(MotorRight.PreIntegral > IMAX)						//积分修正,设定积分上下限，
    		MotorRight.PreIntegral = IMAX;        		
    	else if(MotorRight.PreIntegral < -IMAX)       	
    		MotorRight.PreIntegral = -IMAX;
		      		
//    	if( MotorRight.PreIntegral>0 && speed_error <0 )	//积分项于正负换向时清零
//    		MotorRight.PreIntegral=0;       		
//    	else if( MotorRight.PreIntegral<0 && speed_error >0 )
//    		MotorRight.PreIntegral=0;

//		else if( MotorRight.PreIntegral>0 && speed_derror <0 )	//变化趋势改变，积分减半
//			MotorRight.PreIntegral /= 2;       		
//		else if( MotorRight.PreIntegral<0 && speed_derror >0 )
//			MotorRight.PreIntegral /= 2;

//		Ditem *= 90;		//加延时因子
//        Ditem /= 100;
//        		
//    	if(speed_derror != 0)
//    	{
//    		Ditem += (speed_derror * MotorRight.Kd)/d_count;
//    		d_count = 0;
//    	}

		MotorRight.SpeedU = MotorRight.Kp * speed_error + MotorRight.Ki * MotorRight.PreIntegral + /*Ditem*/MotorRight.Kd * speed_derror;	//位置PID算法
		MotorRight.SpeedU /= 10;	//调节到合适的输出范围

		//防止调节溢出
		if( MotorRight.SpeedU >= SPEED_MAX ) 		
			MotorRight.SpeedU = SPEED_MAX;
		else if( MotorRight.SpeedU <= -SPEED_MAX ) 
			 MotorRight.SpeedU = -SPEED_MAX;
	} 
	Motor_Control(M_RIGHT,MotorRight.SpeedU);
}

//堵转保护功能
void OverCur_Protect(void)	
{
	static u8 cnt_over_cur = 0;
	u8 i = 0;
	u8 flag_over_cur = 0;

	if( PEin(6) == 0 || PEin(7) == 0 ) 	//如果此采样周期电机过电流，则记录一次
	{
		rec_over_cur[cnt_over_cur] = 1;	
	}
	else
	{
		rec_over_cur[cnt_over_cur] = 0;
	}
	cnt_over_cur++;						//记录OVR_CUR_TIM次后清零，重新计数
	if( cnt_over_cur == OVR_CUR_TIM ) cnt_over_cur = 0;

	for(i=0;i<OVR_CUR_TIM;i++)				   //计算OVR_CUR_TIM次记录中有几次过电流
	{
		flag_over_cur += rec_over_cur[i];	
	}
	if( flag_over_cur >= 80 ) 			//如果在近OVR_CUR_TIM（100）次计数内有80次以上过流，则认为电机堵转
	{
		MOTOR_Sudden_Stop();
		GPIO_SetBits(GPIOC, GPIO_Pin_1);	//打开蜂鸣器
		while(1);	
	}
		
}



