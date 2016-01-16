#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#define	 OVER_CUR_1				GPIO_Pin_6
#define	 OVER_CUR_2				GPIO_Pin_7 

#define	 MOTOR_RIGHT_CTR_A		GPIO_Pin_8
#define	 MOTOR_RIGHT_CTR_B		GPIO_Pin_9   
#define	 MOTOR_LEFT_CTR_A		GPIO_Pin_6
#define	 MOTOR_LEFT_CTR_B		GPIO_Pin_7

#define M_LEFT  0
#define M_RIGHT 1

#define OVR_CUR_TIM 100

#define KP_L	200
#define KI_L	30
#define KD_L	0
#define KP_R	200
#define KI_R	30
#define KD_R	0

//////////////////////////////////////////////////////////////////////////////////
typedef struct	
{	
	s32 Ref;			//参考输入值
	s32 FeedBack;		//反馈值
	s32 PreError;		//前一次误差,ui_Ref - FeedBack
	s32 PreIntegral;	//前一次积分项，ui_PreIntegral+ui			
	s32 Kp;				//比例系数	
	s32 Ki;				//积分系数	
	s32 Kd;				//微分系数	
	s32 SpeedU;			//电机控制输出值
																		
} PID;

#define DEADLINE 0		//PID调节死区
#define IMAX 4000		//积分上限
#define SPEED_MAX PWM_PERIOD

extern PID MotorLeft,MotorRight;

void MOTOR_Init(void);
void MOTOR_Sudden_Stop(void);
void Motor_Control(u8 motor, s16 speed);	//open-loop	
void CloseLoopMotorControl(void);

void OverCur_Protect(void);					//堵转保护
	 				    
#endif
