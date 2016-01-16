#ifndef __ROBOTCONTROL_H
#define __ROBOTCONTROL_H

#include "sys.h"

#define WHEEL_RADIUS	47.5	//��λͳһΪmm
#define	TRACK			261		//�������ĵľ���
#define TIME_INTERVAL	0.005	//��������
#define	ENC_RESO		1336	//���̷ֱ���
#define	GEAR_REDU		36		//���ٱ�
#define PI				3.14159265
#define SPEED_RATIO 	1.241065	//	 SPEED_RATIO = 	( 2 * PI * WHEEL_RADIUS ) / (TIME_INTERVAL * ENC_RESO * GEAR_REDU )����λΪmm
#define L_RATIO			1.221490	//ʵ��ֵ
#define R_RATIO			1.221504	//ʵ��ֵ
#define VMAX			800		//����ٶ�800mm/s����0.8m/s
#define WMAX			200		//��߽��ٶ�200��/s

#define DV_MIN			1
#define DW_MIN			5
	

typedef struct
{
	float dx;
	float dy;
	float dtheta;
	float x;
	float y;
	float theta;	
}Inetial_Coordinate;

typedef struct
{
	float v;
	float w;
}Ref_Coodinate;

//typedef struct
//{
//	float v_req;
//	float w_req;
//	float v_pre;
//	float w_pre;
//	float v_cur;
//	float w_cur;
//	float Trans_Time;
//}Interpolation;

extern Inetial_Coordinate Odm;
extern Ref_Coodinate Ref_Speed;
extern u16 OverTimeProtect;
extern float v_cur;
extern float w_cur;

void OdometryInit(void);
void GetOdometryData(void);
void SetSpeed(float v, float w);
void Speed_Interpolation(void);

void Stop(void);
void TurnLeft(void);
void TurnRight(void);
void MoveStraight(void);
void Turn(void);

#endif		
