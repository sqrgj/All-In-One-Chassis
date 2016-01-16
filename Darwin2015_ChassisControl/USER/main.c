#include "led.h"
#include "delay.h"
#include "usart.h"
#include "sys.h"
#include "key.h"
#include "pwm.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "robotcontrol.h"
#include "myi2c.h"
#include "mpu6050.h"
#include "sensors.h"
#include "power.h"
#include "spi.h"
#include "24l01.h"
#include "remote.h"
#include "rtc.h"
#include "ds18B20.h"
#include "wkup.h"

void Init(void)
{
	SystemInit(); 			 	//ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz
	delay_init(72);	    	 	//��ʱ������ʼ��	  
	NVIC_Configuration(); 	 	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	WKUP_Init();
	PWR_Init();
	RTC_Init();
 	LED_Init();			     	//LED�˿ڳ�ʼ��
	KEY_Init();				 	//KEY��ʼ��
	Sensors_Init();				//��������ʼ��
	uart_init(115200);			//����1ʹ�ܣ�������Ϊ115200
	PWM_Init(PWM_PERIOD,1);		//PWM��ʼ����Ƶ��Ϊ20KHz
	MOTOR_Init();
	EncoderL_Init();			//�����̳�ʼ��
	EncoderR_Init();			//�����̳�ʼ��
	OdometryInit();
	I2C_GPIO_Config();			//I2C�ӿڳ�ʼ�������ڶ�ȡMPU6050����
	InitMPU6050();				//MPU6050��ʼ��
//	DS18B20_Init();				//�¶ȴ�����DS18B20��ʼ��
//
//	NRF24L01_Init();			//����ͨѶ��ʼ��
	Remote_Init();				//����ң�س�ʼ��

	Timer1_Init();				//TIM1��Ϊȫ�ֶ�ʱ����5ms�ж�һ��	
}
	
int main(void)
{
	u32 count = 0;
//	u8 tmp_buf[33];
	BOOL L_Obstacle = FALSE, R_Obstacle = FALSE;
	u16 handle_time = 40;
	u8 Lcount = 0, Rcount = 0;
	int obstacle_angle = 0;
	u8 IR_cnt = 0, IR_on_cnt = 0;
	u8 Dock_IR_cntL = 0, Dock_IR_cntR = 0;	 			//for auto find dock

 	Init();

//	MotorLeft.Ref = 200;
//	MotorRight.Ref = 200;
	SetSpeed(0,0); 

	//24L01��ʼ��
//	while(NRF24L01_Check())//��ⲻ��24L01
//	{
//		delay_ms(1000);
//		printf("24L01 Check Failed!\n");
//		LED0 = !LED0;
//	}
//	RX_Mode();

	mode =0;
//	Enable_Exit2_IRQ();

	while(1)
	{
		PWR_Monitor();				//�͵籨��
		OverCur_Protect();			//��ת����

/*------------------------------------������Ϣ�ɼ�--------------------------------------------*/
	   	GetBumpers();				//��ȡbumper��Ϣ
		GetIRSensors();				//��ȡ���⴫������Ϣ
		if( 1 == count%20 )		TrigUS1();			//������1��������	
		if( 3 == count%20 )		GetUS1Data();		//��ȡ����1�ľ���
		if( 11 == count%20 )	TrigUS2();			//������2��������	
		if( 13 == count%20 )	GetUS2Data();		//��ȡ����2�ľ���
//		if( 5 == count ) 		DS18B20_Start_Convert();	//��ʼ�¶�ת����400msһ��
//		if( 6 == count )		
//		{
//			temperature = DS18B20_Get_Temp();	//��ȡ�¶�ֵ
//			if( temperature >= 400 )	 temperature = 400;
//			if( temperature <= 0) 	temperature = 0;
//		}
//		
		GetWheelSpeed();  			//��ȡ����ٶ�
/*----------------------------------end������Ϣ�ɼ�-------------------------------------------*/

/*------------------------------------���������߲���------------------------------------------*/
		switch(mode)
		{
		case 0x00:
			if( ABS(SpeedR) < MINSPEED && ABS(SpeedL) < MINSPEED )
			{
				;//����ٶ�С��һ����Сֵ�������������Լ��������Ĳ��Ե���
			}
			else
			{
				if( (BMP1==0) /*|| (GroundIR_L!=0) */) 
				{	
					Lcount++;
					if(5 == Lcount)	L_Obstacle = TRUE;
				}
				else  
				{
					L_Obstacle = FALSE;
					Lcount = 0;
				}
				if( (BMP2==0) /*|| (GroundIR_R!=0) */) 
				{
					Rcount++;
					if(5 == Rcount) R_Obstacle = TRUE;
				}
				else 
				{	
					R_Obstacle = FALSE;
					Rcount = 0;
				}

				if( ( IRs & 0xFC ) != 0xFC  ) 			//0b11111100
				{
					obstacle_angle = 0.0;
					IR_cnt = 0;
					if( ( IRs & (1<<7) ) == 0 ) { obstacle_angle += 50; IR_cnt++; }
					if( ( IRs & (1<<6) ) == 0 ) { obstacle_angle += 30; IR_cnt++; }
					if( ( IRs & (1<<5) ) == 0 ) { obstacle_angle += 10; IR_cnt++; }
					if( ( IRs & (1<<4) ) == 0 ) { obstacle_angle -= 10; IR_cnt++; }
					if( ( IRs & (1<<3) ) == 0 ) { obstacle_angle -= 30; IR_cnt++; }
					if( ( IRs & (1<<2) ) == 0 ) { obstacle_angle -= 50; IR_cnt++; }
					obstacle_angle /= IR_cnt;
					IR_on_cnt++;
					if( obstacle_flag == 0 && IR_on_cnt == 3 )
						obstacle_flag = 5;
		//				printf("%02X\t%d\n", Front_IRs, obstacle_angle);
		
				}
				else
				{
					IR_on_cnt = 0;
				}

			}
	
			if( ( L_Obstacle || R_Obstacle ) && ( obstacle_flag == 0) )
			{
				 obstacle_flag = 1;
			}			
			
			switch(obstacle_flag)
			{
				case 0:	break;
				case 1:
					if( L_Obstacle ) 
					{
						if( R_Obstacle )
						{
							LED0 = LED1 = 0;
							obstacle_angle=0;	
						}
						else
						{
							LED0 = 0;
							obstacle_angle=90;
						}
					}
					if( R_Obstacle )
					{
						if( L_Obstacle )
						{
							LED0 = LED1 = 0;
							obstacle_angle=0;	
						}
						else
						{
							LED1 = 0;
							obstacle_angle= -90;
						}
					}
	//				SetSpeed(0,0);
					v_cur = 0;
					w_cur = 0;
					MotorLeft.Ref = 100;
					MotorRight.Ref = -100;
					obstacle_flag = 2;
					handle_time = 100;
					break;
				case 2:
					handle_time--;
					MotorLeft.Ref = 100;
					MotorRight.Ref = -100;
					if(handle_time == 0)   
					{
						obstacle_flag = 3;
						//���ת��һ���Ƕ�
	//					handle_time = rand()%400+100;
	//					temp = (rand()%99>50)?1:-1;
	//					SetSpeed(0,80*temp);
						//ת����IR��������صĽǶ�
						SetSpeed(0,80*(obstacle_angle>0?-1:1));
						OverTimeProtect = 550;
						handle_time = ( 180 - ABS(obstacle_angle) )*5/2; 
					}
					break;
				case 3:
					handle_time--;
					if(handle_time == 0)
					{
						obstacle_flag = 4;
					}
					break;
				case 4:
					SetSpeed(0,0);
					LED0 = LED1 = 1;
					obstacle_flag = 0;
					break;
				case 5:
					SetSpeed(0,0);
					handle_time = 100;
					obstacle_flag = 6;
					break;
				case 6:
					handle_time--;
					if(handle_time == 0)
					{
						SetSpeed(0,80*(obstacle_angle>0?-1:1));
						OverTimeProtect = 550;
						handle_time = ( 180 - obstacle_angle*(obstacle_angle>0?1:-1) ) * 5 /2; 
						obstacle_flag = 3;
					}
					break;
				default:
					break;
			}
	
			if( obstacle_flag != 2)			//���������������bumper����Ľ�������
				Speed_Interpolation();		//�����ٶȲ岹
			if( 80 == count ) 	count = 0;
			break;

		case 0x01:
			//Get_Dock_IR_Information
			if(status1 == DataComplete)
			{
				IR_Recv_Process(&m_IR_Addr, &m_IR_Data);
				if(m_IR_Data == Dock_Left_ID)
	            {
					LED0 = 0;
					LED1 = 1;
					Dock_IR_cntL++;
	            }
	            else if(m_IR_Data == Dock_Right_ID)
	            {
					LED0 = 1;
					LED1 = 0;
					Dock_IR_cntR++;
	            }
	            else
	            {
					LED0 = 1;
					LED1 = 1;
	            }
	            m_IR_Data = 0;	
			}
			else
			{
				LED0 = 1;
				LED1 = 1;	
			}
	
			if(80 == count)
			{
	//			printf("left = %d ,right = %d \n",Dock_IR_cntL,Dock_IR_cntR);
				//�жϻ�����λ���ĸ�����
				if( Dock_IR_cntL>=1 && Dock_IR_cntR>=1 )
				{
					Cur_Sta = C;
				}
				else
				{
					if( Dock_IR_cntL>=1 )
					{
						Cur_Sta = L;
					}
					else if( Dock_IR_cntR>=1 )
					{
						Cur_Sta = R;
					}
					else
					{
						Cur_Sta = N;
					}
				}
				//��״̬�仯ʱ����������
				if( Cur_Sta != Pre_Sta )
				{
					switch(Pre_Sta)
					{
						case N:
							MoveStraight();
							break;
						case L:
							if( Cur_Sta == C ) MoveStraight();
							else	TurnLeft();							
							break;
						case R:
							if( Cur_Sta == C ) MoveStraight();
							else	TurnRight();
							break;
						case C:
							if( Cur_Sta == R ) TurnLeft();
							else if( Cur_Sta == L )	TurnRight();
							else Turn();
							break;
						default:
							break;
					}
					Pre_Sta =  Cur_Sta;
				}
				Dock_IR_cntL = Dock_IR_cntR = 0;
				count = 0;
				
			}
	
			if( Bumpers != 0xFF || charge_flag )
			{
				Stop();
			}
			break;

		default:
			break;
		}
/*---------------------------------end���������߲���------------------------------------------*/
		
		CloseLoopMotorControl();	//����ջ�����

/*----------------------------------����Odometry����------------------------------------------*/  	  
	  	GetOdometryData();			//��ȡOdometry����
		if( count%10 == 0 )
		{
//			printf("L=%d, R=%d\n",SpeedL,SpeedR);
//			printf("v=%f, w=%f\n",Ref_Speed.v,Ref_Speed.w);
//			printf("x=%f, y=%f, theta = %f\n",Odm.x,Odm.y,Odm.theta);
//			printf("distanceL = %d, distanceR = %d\n",distanceL, distanceR);
//			printf("L=%d, R=%d\n",(int)US1_Dis,(int)US2_Dis);
			printf("IRs=0x%02X\n",IRs);
//			printf("bumpers=0x%02X\n",Bumpers);

			ReturnOdoData();
//		   	if(NRF24L01_RxPacket(tmp_buf)==0)//����24L01��Ϣ�������յ���Ϣ����tmp_bufΪ0
//			{									 
//				tmp_buf[32]=0;//�����ַ���������
//				//�д�����24L01��Э�鴦����
//				//...
//			}
		}
/*-------------------------------end����Odometry����------------------------------------------*/

		//��ʱ��������
		if( OverTimeProtect-- <= 0)
		{
			SetSpeed(0,0);
		}
 
		//�ȴ�5ms����֤������������Ϊ5ms
		//������ע�⣡��������������û�д�NVIC��Ӧ�Ķ�ʱ���жϣ����ʵ�ʲ������ж���Ϊ��ֻ�������жϱ�־λ���о�ȷ��ʱ
		while(TIM_GetITStatus(TIM1, TIM_IT_Update) != SET) ;//��û�ж�ʱ�������ж���λ����һֱ�ȴ�
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);			//���ж���λ�������ж˱�־λ
									
		count++;
	}
}

