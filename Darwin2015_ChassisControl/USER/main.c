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
	SystemInit(); 			 	//系统时钟初始化为72M	  SYSCLK_FREQ_72MHz
	delay_init(72);	    	 	//延时函数初始化	  
	NVIC_Configuration(); 	 	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	WKUP_Init();
	PWR_Init();
	RTC_Init();
 	LED_Init();			     	//LED端口初始化
	KEY_Init();				 	//KEY初始化
	Sensors_Init();				//传感器初始化
	uart_init(115200);			//串口1使能，波特率为115200
	PWM_Init(PWM_PERIOD,1);		//PWM初始化，频率为20KHz
	MOTOR_Init();
	EncoderL_Init();			//左码盘初始化
	EncoderR_Init();			//右码盘初始化
	OdometryInit();
	I2C_GPIO_Config();			//I2C接口初始化，用于读取MPU6050数据
	InitMPU6050();				//MPU6050初始化
//	DS18B20_Init();				//温度传感器DS18B20初始化
//
//	NRF24L01_Init();			//无线通讯初始化
	Remote_Init();				//红外遥控初始化

	Timer1_Init();				//TIM1作为全局定时器，5ms中断一次	
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

	//24L01初始化
//	while(NRF24L01_Check())//检测不到24L01
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
		PWR_Monitor();				//低电报警
		OverCur_Protect();			//堵转保护

/*------------------------------------传感信息采集--------------------------------------------*/
	   	GetBumpers();				//获取bumper信息
		GetIRSensors();				//获取红外传感器信息
		if( 1 == count%20 )		TrigUS1();			//给超声1出发脉冲	
		if( 3 == count%20 )		GetUS1Data();		//获取超声1的距离
		if( 11 == count%20 )	TrigUS2();			//给超声2出发脉冲	
		if( 13 == count%20 )	GetUS2Data();		//获取超声2的距离
//		if( 5 == count ) 		DS18B20_Start_Convert();	//开始温度转换，400ms一次
//		if( 6 == count )		
//		{
//			temperature = DS18B20_Get_Temp();	//读取温度值
//			if( temperature >= 400 )	 temperature = 400;
//			if( temperature <= 0) 	temperature = 0;
//		}
//		
		GetWheelSpeed();  			//获取电机速度
/*----------------------------------end传感信息采集-------------------------------------------*/

/*------------------------------------机器人行走策略------------------------------------------*/
		switch(mode)
		{
		case 0x00:
			if( ABS(SpeedR) < MINSPEED && ABS(SpeedL) < MINSPEED )
			{
				;//如果速度小于一个最小值，不做防跌落以及防碰触的策略调整
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
						//随机转过一定角度
	//					handle_time = rand()%400+100;
	//					temp = (rand()%99>50)?1:-1;
	//					SetSpeed(0,80*temp);
						//转过与IR传感器相关的角度
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
	
			if( obstacle_flag != 2)			//如果不是由于碰触bumper引起的紧急后退
				Speed_Interpolation();		//进行速度插补
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
				//判断机器人位于哪个区域
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
				//当状态变化时，产生动作
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
/*---------------------------------end机器人行走策略------------------------------------------*/
		
		CloseLoopMotorControl();	//电机闭环调速

/*----------------------------------返回Odometry数据------------------------------------------*/  	  
	  	GetOdometryData();			//获取Odometry数据
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
//		   	if(NRF24L01_RxPacket(tmp_buf)==0)//接收24L01信息，若接收到信息，则tmp_buf为0
//			{									 
//				tmp_buf[32]=0;//加入字符串结束符
//				//有待加入24L01的协议处理功能
//				//...
//			}
		}
/*-------------------------------end返回Odometry数据------------------------------------------*/

		//超时保护功能
		if( OverTimeProtect-- <= 0)
		{
			SetSpeed(0,0);
		}
 
		//等待5ms，保证整个控制周期为5ms
		//！！！注意！！！，这里由于没有打开NVIC对应的定时器中断，因此实际不发生中断行为，只是利用中断标志位进行精确计时
		while(TIM_GetITStatus(TIM1, TIM_IT_Update) != SET) ;//若没有定时器更新中断置位，则一直等待
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);			//若中断置位后，则清中端标志位
									
		count++;
	}
}

