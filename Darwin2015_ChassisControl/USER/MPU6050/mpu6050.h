#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "stm32f10x.h"

typedef struct{
	float roll;
	float pitch;
	float yaw;
	}EulerAngle;

typedef struct{
	s32 accx;
	s32 accy;
	s32 accz;
	s32 temperature;
	s32 gyrox;
	s32 gyroy;
	s32 gyroz;
	}MPU6050_DATA;

//****************************************
// ����MPU6050�ڲ���ַ
//****************************************

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define MPU6050_ADDRESS 0xD0    //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

#define GYRO_GAIN	0.00763358778626	//250
//#define GYRO_GAIN	0.015267175572519	//500
//#define GYRO_GAIN	0.0609756097561		//2000

#define YAW_THRESHOLD	25

#define OFFSETTIME 100


void InitMPU6050(void);				//��ʼ��MPU6050
void MPU6050_Read(void);
float ReturnYawAngle(void);			//����Yaw��
void SetYawAngle(float angle);		//����Yaw��
float ReturnW(void);				//����Yaw�ǽ��ٶ�

void GetInitialValue(void);


#endif // __MPU6050_H__
