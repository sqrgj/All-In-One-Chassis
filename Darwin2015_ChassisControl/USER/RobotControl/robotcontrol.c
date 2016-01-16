#include "robotcontrol.h"
#include "encoder.h"
#include "mpu6050.h"
#include "motor.h"
#include <math.h>

Inetial_Coordinate Odm;
Ref_Coodinate Ref_Speed;
///////�����ٶȲ岹//////////
float v_req = 0;	  	//Ҫ����µ��ٶ�
float w_req = 0;	  	//Ҫ����µĽ��ٶ�
float v_cur = 0;	  	//��ǰ�ٶ�
float w_cur = 0;	  	//��ǰ���ٶ�
float dv=0;	  		  	//�ٶȱ仯��
float dw=0;			  	//���ٶȱ仯��		
float Trans_Time = 300;		//�ٶȱ仯ʱ�䣬300*5ms = 1.5s
u16 OverTimeProtect = 60000;  //�������ʱ����ʱ��Ϊ5min

void OdometryInit(void)
{
	Odm.dx = 0.0;
	Odm.dy = 0.0;
	Odm.dtheta = 0.0;
	Odm.x = 0.0;
	Odm.y = 0.0;
	Odm.theta = 0.0;

	Ref_Speed.v = 0.0;
	Ref_Speed.w = 0.0;
}

void GetOdometryData(void)
{
	//��ȡ�������ϵ��˲ʱ�ٶ�����ٶ�
	Ref_Speed.v = (float)( SpeedR * R_RATIO - SpeedL * L_RATIO ) / 2;
	
	Odm.theta = ReturnYawAngle();												//ͨ�������ǻ�ȡת���Ƕ�
	Ref_Speed.w = ReturnW();													//ͨ�������ǻ�ȡת�����ٶ�

	//��ȡ��������ϵ���ٶ���λ��
	Odm.dtheta = Ref_Speed.w;
	Odm.dx = cos(Odm.theta * PI / 180) * Ref_Speed.v;
	Odm.dy = sin(Odm.theta * PI / 180) * Ref_Speed.v;
	Odm.x += Odm.dx * TIME_INTERVAL;
	Odm.y += Odm.dy * TIME_INTERVAL;

//	fortest
//	Odm.x = 233.332;
//	Odm.y = 10.22;
//	Odm.theta = -93.2;

}

void SetSpeed(float v, float w)
{
	v_req = v;
	w_req = w;

	dv = ( v_req - v_cur ) / Trans_Time;
	dw = ( w_req - w_cur ) / Trans_Time;

	//���ٶȵ���Сֵ
	if( dv < DV_MIN && dv > 0) dv = DV_MIN;
	if( dv > -DV_MIN && dv < 0 ) dv = -DV_MIN;

	if( dw < DW_MIN && dw > 0) dw = DW_MIN;
	if( dw > -DW_MIN && dw < 0 ) dw = -DW_MIN;
}

void Speed_Interpolation(void)
{
 	if( fabs(v_cur-v_req) <= fabs(dv) ) v_cur = v_req;
	else v_cur += dv;

	if( fabs(w_cur-w_req) <= fabs(dw) ) w_cur = w_req;
	else w_cur += dw;
	
	MotorLeft.Ref = -(s32)(( v_cur - w_cur * TRACK * PI / 360 ) / L_RATIO);
   	MotorRight.Ref = (s32)(( v_cur + w_cur * TRACK * PI / 360 ) / R_RATIO); 	
}

void Stop(void)
{
	MotorLeft.Ref = 0;
	MotorRight.Ref = 0;
}

void TurnLeft(void)
{
	MotorLeft.Ref = 0;
	MotorRight.Ref = 100;
}

void TurnRight(void)
{
	MotorLeft.Ref = -100;
	MotorRight.Ref = 0;
}

void MoveStraight(void)
{
	MotorLeft.Ref = -100;
	MotorRight.Ref = 100;
}

void Turn(void)
{
	MotorLeft.Ref = 100;
	MotorRight.Ref = 100;
}
