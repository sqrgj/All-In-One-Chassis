#ifndef __ROBOTCONTROL_H
#define __ROBOTCONTROL_H

#include "sys.h"

#define WHEEL_RADIUS	47.5	//单位统一为mm
#define	TRACK			261		//两轮中心的距离
#define TIME_INTERVAL	0.005	//控制周期
#define	ENC_RESO		1336	//码盘分辨率
#define	GEAR_REDU		36		//减速比
#define PI				3.14159265
#define SPEED_RATIO 	1.241065	//	 SPEED_RATIO = 	( 2 * PI * WHEEL_RADIUS ) / (TIME_INTERVAL * ENC_RESO * GEAR_REDU )，单位为mm
#define L_RATIO			1.221490	//实测值
#define R_RATIO			1.221504	//实测值
#define VMAX			800		//最高速度800mm/s，即0.8m/s
#define WMAX			200		//最高角速度200°/s

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
