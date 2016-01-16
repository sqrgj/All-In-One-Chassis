#include "mpu6050.h"
#include "myi2c.h"
#include "delay.h"
#include "robotcontrol.h"
#include "usart.h"

u8	mpu6050_buffer[14];					//iic读取后存放数据 
MPU6050_DATA IMUdata;
MPU6050_DATA IMUoffset;
EulerAngle IMU_I;

//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050()
{
	delay_ms(20); //此延迟不可去掉，保证mpu6050芯片上电时间充分
	Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x09);//disables the temperature sensor	
	Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);
	Single_Write(MPU6050_ADDRESS, CONFIG, 0x04);//Digital Low Pass Filter (DLPF) setting
	Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);//+-250 deg per s, 131 LSB/degree/s
	Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);//+-2g, 16384 LSB/mg 
	
	//offset修正
	delay_ms(20);
	GetInitialValue();
	
	IMU_I.roll = 0.0;
	IMU_I.pitch = 0.0;
	IMU_I.yaw = 0.0;
}

void GetInitialValue(void)
{
	MPU6050_DATA temp={0};
	
	int i = 100;
	//采集的前100个数不要
	while(i--)
	{
		mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_H);
		mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_L);
		mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_H);
		mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_L);
		mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_H);
		mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_L);
		mpu6050_buffer[6]=Single_Read(MPU6050_ADDRESS, TEMP_OUT_H);
		mpu6050_buffer[7]=Single_Read(MPU6050_ADDRESS, TEMP_OUT_L);
		mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
		mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
		mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
		mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
		mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
		mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);
		delay_ms(5);	
	}

	i = OFFSETTIME;
	while(i--)
	{
		//collect data
		mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_H);
		mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_L);
		mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_H);
		mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_L);
		mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_H);
		mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_L);
		mpu6050_buffer[6]=Single_Read(MPU6050_ADDRESS, TEMP_OUT_H);
		mpu6050_buffer[7]=Single_Read(MPU6050_ADDRESS, TEMP_OUT_L);
		mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
		mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
		mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
		mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
		mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
		mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);
	
		//obtain IMU data
		temp.accx += (s16)(mpu6050_buffer[0] << 8 | mpu6050_buffer[1]);
		temp.accy += (s16)(mpu6050_buffer[2] << 8 | mpu6050_buffer[3]);
		temp.accz += (s16)(mpu6050_buffer[4] << 8 | mpu6050_buffer[5]);
		temp.temperature += (s16)(mpu6050_buffer[6] << 8 | mpu6050_buffer[7]);
		temp.gyrox += (s16)(mpu6050_buffer[8] << 8 | mpu6050_buffer[9]);
		temp.gyroy += (s16)(mpu6050_buffer[10] << 8 | mpu6050_buffer[11]);
		temp.gyroz += (s16)(mpu6050_buffer[12] << 8 | mpu6050_buffer[13]);

		delay_ms(5);
	 }

	IMUoffset.accx = temp.accx/OFFSETTIME;
	IMUoffset.accy = temp.accy/OFFSETTIME;
	IMUoffset.accz = temp.accz/OFFSETTIME;

	IMUoffset.gyrox = temp.gyrox/OFFSETTIME;
	IMUoffset.gyroy = temp.gyroy/OFFSETTIME;
	IMUoffset.gyroz = temp.gyroz/OFFSETTIME;

	IMUoffset.temperature = temp.temperature/OFFSETTIME;

}


/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,获得MPU6050的数据
*******************************************************************************/
void MPU6050_Read(void)
{
	//collect data
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_H);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_L);
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_H);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_L);
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_H);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_L);
	mpu6050_buffer[6]=Single_Read(MPU6050_ADDRESS, TEMP_OUT_H);
	mpu6050_buffer[7]=Single_Read(MPU6050_ADDRESS, TEMP_OUT_L);
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);

	//obtain IMU data
	IMUdata.accx = (s16)(mpu6050_buffer[0]<<8 | mpu6050_buffer[1]) - IMUoffset.accx;
	IMUdata.accy = (s16)(mpu6050_buffer[2]<<8 | mpu6050_buffer[3]) - IMUoffset.accy;
	IMUdata.accz = (s16)(mpu6050_buffer[4]<<8 | mpu6050_buffer[5]) - IMUoffset.accz;
	IMUdata.temperature = (s16)(mpu6050_buffer[6]<<8 | mpu6050_buffer[7]) - IMUoffset.temperature;
	IMUdata.gyrox = (s16)(mpu6050_buffer[8]<<8 | mpu6050_buffer[9]) - IMUoffset.gyrox;
	IMUdata.gyroy = (s16)(mpu6050_buffer[10]<<8 | mpu6050_buffer[11]) - IMUoffset.gyroy;
	IMUdata.gyroz = (s16)(mpu6050_buffer[12]<<8 | mpu6050_buffer[13]) - IMUoffset.gyroz;
//	printf("%d,%d,%d\n",IMUdata.gyrox,IMUdata.gyroy,IMUdata.gyroz);

}

/**************************实现函数********************************************
//获取MPU6050的yaw角
*******************************************************************************/
 float ReturnYawAngle(void)
 {
	//读取MPU6050数据，并修正后放入相应变量中
	MPU6050_Read();

	if(IMUdata.gyroz > YAW_THRESHOLD || IMUdata.gyroz < -YAW_THRESHOLD)
	IMU_I.yaw += (float)IMUdata.gyroz * GYRO_GAIN * TIME_INTERVAL;

	if( IMU_I.yaw > 180) IMU_I.yaw -= 360;
	else if( IMU_I.yaw <= -180 ) IMU_I.yaw += 360;
	
	return IMU_I.yaw;
 }

 /**************************实现函数********************************************
//获取MPU6050的yaw角
*******************************************************************************/
 void SetYawAngle(float angle)
 {
	if( angle >= -180 && angle < 180 )	 IMU_I.yaw = angle;
 }

/**************************实现函数********************************************
//获取MPU6050的角速度
*******************************************************************************/
 float ReturnW(void)
 {
	float dyaw;
	dyaw = (float)IMUdata.gyroz * GYRO_GAIN * TIME_INTERVAL;
	
	return dyaw;
 }
