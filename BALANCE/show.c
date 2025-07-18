#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //此任务以10Hz的频率运行
		
		//开机时蜂鸣器短暂蜂鸣，开机提醒
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //蜂鸣器蜂鸣
		else if(Buzzer_count==5)Buzzer=0;
		
		//Read the battery voltage //读取电池电压
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V，低电量时蜂鸣器第一次报警
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V，小车禁止控制时蜂鸣器第二次报警
					
		APP_Show();	 //Send data to the APP //向APP发送数据
	  oled_show(); //Tasks are displayed on the screen //显示屏显示任务
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show;
	
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //采集电位器档位信息，实时显示小车开机时要适配的小车型号
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 POT_val = Get_adc_Average(Potentiometer,10);
	 Car_Mode_Show=(int) (POT_val/Divisor_Mode);
	 if(Car_Mode_Show>5)Car_Mode_Show=5; 
	  //Car_Mode_Show=0;
	
	 Voltage_Show=Voltage*100; 
	 count++;
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM清零但不立即刷新，防止花屏
	 if(Check==0)//The car displays normally when the self-check mode is not enabled //没有开启自检模式时小车正常显示
	 {	
		 //The first line of the display displays the content//
		 //显示屏第1行显示内容//
		 switch(Car_Mode_Show)
		 {
			case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; 
			case Omni_Car:      OLED_ShowString(0,0,"Omni"); break; 
			case Akm_Car:       OLED_ShowString(0,0,"Akm "); break; 
			case Diff_Car:      OLED_ShowString(0,0,"Diff"); break; 
			case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; 
			case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; 
		 }
		 
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		 {
			 //The Mec_car and omni_car show Z-axis angular velocity
			 //麦轮、全向轮小车显示Z轴角速度
			 OLED_ShowString(55,0,"GZ");
			 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
			 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);		
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car||Car_Mode==Tank_Car)
		 {
			 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
			 //阿克曼、差速、四驱、履带车显示陀螺仪零点
			 OLED_ShowString(55,0,"BIAS");
			 if( Deviation_gyro[2]<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-Deviation_gyro[2],3,12);  //Zero-drift data of gyroscope Z axis
			 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, Deviation_gyro[2],3,12);	//陀螺仪z轴零点漂移数据	
		 }
		 //The first line of the display displays the content//
		 //显示屏第1行显示内容//
		 

		 //The second line of the display displays the content//
		 //显示屏第2行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
			//麦轮、全向轮、四驱车显示电机A的目标速度和当前实际速度
			OLED_ShowString(0,10,"A");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
			
			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
			 //阿克曼、差速、坦克小车显示Z轴角速度
			 OLED_ShowString(00,10,"GYRO_Z:");
			 if( gyro[2]<0)  OLED_ShowString(60,10,"-"),
											 OLED_ShowNumber(75,10,-gyro[2],5,12);
			 else            OLED_ShowString(60,10,"+"),
											 OLED_ShowNumber(75,10, gyro[2],5,12);			
		 }	 
		 //The second line of the display displays the content//
		 //显示屏第2行显示内容//
		 
		 //Lines 3 and 4 of the display screen display content//
		 //显示屏第3、4行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
			//麦轮、全向轮、四驱车显示电机B的目标速度和当前实际速度
			OLED_ShowString(0,20,"B");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
			
			//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
			//麦轮、全向轮、四驱车显示电机C的目标速度和当前实际速度
			OLED_ShowString(0,30,"C");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
														OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
														OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
				
			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
			 //阿克曼、差速、履带车显示电机A的目标速度和当前实际速度
			 OLED_ShowString(0,20,"L:");
			 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
			 else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
			 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
			 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
			 //阿克曼、差速、履带车显示电机B的目标速度和当前实际速度
			 OLED_ShowString(0,30,"R:");
			 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
			 else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
				
			 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

//			 if( Remoter_Ch1<0)	    OLED_ShowString(15,20,"-"),
//															OLED_ShowNumber(20,20,-Remoter_Ch1,5,12);
//			 else                 	OLED_ShowString(15,20,"+"),
//															OLED_ShowNumber(20,20, Remoter_Ch1,5,12);  
//			 if( Remoter_Ch2<0)	    OLED_ShowString(60,20,"-"),
//															OLED_ShowNumber(75,20,-Remoter_Ch2,5,12);
//			 else                 	OLED_ShowString(60,20,"+"),
//															OLED_ShowNumber(75,20, Remoter_Ch2,5,12);
//			 if( Remoter_Ch3<0)	    OLED_ShowString(15,30,"-"),
//															OLED_ShowNumber(20,30,-Remoter_Ch3,5,12);
//			 else                 	OLED_ShowString(15,30,"+"),
//															OLED_ShowNumber(20,30, Remoter_Ch3,5,12);  
//			 if( Remoter_Ch4<0)	    OLED_ShowString(60,30,"-"),
//															OLED_ShowNumber(75,30,-Remoter_Ch4,5,12);
//			 else                 	OLED_ShowString(60,30,"+"),
//															OLED_ShowNumber(75,30, Remoter_Ch4,5,12);
		 }
		 //Lines 3 and 4 of the display screen display content//
		 //显示屏第3、4行显示内容//
		 
		 //Line 5 of the display displays the content//
		 //显示屏第5行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
		 {
			  //Mec_Car Display the target speed and current actual speed of motor D
				//麦轮小车显示电机D的目标速度和当前实际速度
				OLED_ShowString(0,40,"D");
				if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
															OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
				else                 	OLED_ShowString(15,40,"+"),
															OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
				if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Omni_Car)
		 {
			  // The Omni_car shows Z-axis angular velocity (1000 times magnification) in rad/s
				//全向轮小车显示Z轴角速度(放大1000倍)，单位rad/s
				OLED_ShowString(0,40,"MOVE_Z"); 			
				if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																						OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
				else                              	OLED_ShowString(60,40,"+"),
																						OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
		 }
		 else if(Car_Mode==Akm_Car)
		 {
			  //Akm_Car displays the PWM value of the Servo
				//阿克曼小车显示舵机的PWM的数值
				OLED_ShowString(00,40,"SERVO:");
				if( Servo<0)		      OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(80,40,-Servo,4,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(80,40, Servo,4,12); 
		 }
		 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
			 //差速小车、履带车显示左右电机的PWM的数值
															 OLED_ShowString(00,40,"MA");
			 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
															 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(20,40,"+"),
															 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
															 OLED_ShowString(60,40,"MB");
			 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
															 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(80,40,"+"),
															 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
		 }
		 //Line 5 of the display displays the content//
		 //显示屏第5行显示内容//
			 
		 //Displays the current control mode //显示当前控制模式
		 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
		 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
		 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
		 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
		 else if ((Usart1_ON_Flag || Usart5_ON_Flag)==1) OLED_ShowString(0,50,"USART");
		 else                       OLED_ShowString(0,50,"ROS  ");
			
		 //Displays whether controls are allowed in the current car
		 //显示当前小车是否允许控制
		 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
		 else                      OLED_ShowString(45,50,"OFF"); 
			
																OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
			                          OLED_ShowString(88,50,".");
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			                          OLED_ShowString(110,50,"V");
		 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		}	
	 else if(Check==1)
	 {
		if(Proc_Flag==0)							//用户自检代码
		 {
			 OLED_ShowCHinese(00, 00, "请先阅读使用手册");
			 OLED_ShowCHinese(00, 16, "  单击用户按键  ");
			 OLED_ShowCHinese(00, 32, "    确认开始    ");
			 OLED_ShowString(104, 50, "0/8");
		 }
		 
		 if(Proc_Flag==1)					
		 {
			 OLED_Show_POT();
			 OLED_ShowCHinese12(8, 9, "请转动电位器：");
			 switch(Car_Mode_Show)
			 {
				case Mec_Car:       OLED_ShowString(92,10,"Mec "); break; 
				case Omni_Car:      OLED_ShowString(92,10,"Omni"); break; 
				case Akm_Car:       OLED_ShowString(92,10,"Akm "); break; 
				case Diff_Car:      OLED_ShowString(92,10,"Diff"); break; 
				case FourWheel_Car: OLED_ShowString(92,10,"4WD "); break; 
				case Tank_Car:      OLED_ShowString(92,10,"Tank"); break; 
			 }
			 OLED_ShowCHinese12(12, 22, "退出前需调回原车型");
			 OLED_ShowCHinese(00, 34, "  模式：电位器  ");
			 OLED_ShowString(104, 50, "1/8");
		 }
		 if(Proc_Flag==2)
		 {
			 OLED_ShowCHinese(24, 00, "架空车轮后");
			 OLED_ShowCHinese(00, 16, "单击开始高速转动");
			 OLED_ShowCHinese(24, 32, "模式：电机");
			 OLED_ShowString(104, 50, "2/8");
		 }
		 if(Proc_Flag==3)
		 {
				if( MOTOR_A.Encoder<0)	OLED_ShowString(00,5,"A-"),
																OLED_ShowNumber(20,5,-MOTOR_A.Encoder*1000,5,12);
				else                		OLED_ShowString(00,5,"A+"),
																OLED_ShowNumber(20,5, MOTOR_A.Encoder*1000,5,12);
				
				if( MOTOR_B.Encoder<0)	OLED_ShowString(70,5,"B-"),
																OLED_ShowNumber(90,5,-MOTOR_B.Encoder*1000,5,12);
				else                		OLED_ShowString(70,5,"B+"),
																OLED_ShowNumber(90,5, MOTOR_B.Encoder*1000,5,12);
				
				if( MOTOR_C.Encoder<0)	OLED_ShowString(00,20,"C-"),
																OLED_ShowNumber(20,20,-MOTOR_C.Encoder*1000,5,12);
				else                		OLED_ShowString(00,20,"C+"),
																OLED_ShowNumber(20,20, MOTOR_C.Encoder*1000,5,12);
				
				if( MOTOR_D.Encoder<0)	OLED_ShowString(70,20,"D-"),
																OLED_ShowNumber(90,20,-MOTOR_D.Encoder*1000,5,12);
				else                		OLED_ShowString(70,20,"D+"),
																OLED_ShowNumber(90,20, MOTOR_D.Encoder*1000,5,12);
				OLED_ShowCHinese(24, 32, "模式：电机");
				OLED_ShowString(104, 50, "2/8");
		 }
		 if(Proc_Flag==4)
		 {
				OLED_ShowCHinese(00, 00, "    单击进入    ");
				OLED_ShowCHinese(00, 32, "  模式：陀螺仪  ");
			  OLED_ShowString(104, 50, "3/8");
		 }
		 if(Proc_Flag==5)
		 {
			 OLED_ShowCHinese(00, 00, "晃动小车观察数据");
			 OLED_ShowString(00, 16, "G");
			 if(gyro[0]<0)							 OLED_ShowString(16, 16, "-"),
																	 OLED_ShowNumber(24, 16, -gyro[0], 5, 12);
			 else							 					 OLED_ShowString(16, 16, "+"),
																	 OLED_ShowNumber(24, 16, gyro[0], 5, 12);
			 
			 if(gyro[1]<0)							 OLED_ShowString(16, 26, "-"),
																	 OLED_ShowNumber(24, 26, -gyro[1], 5, 12);
			 else							 					 OLED_ShowString(16, 26, "+"),
																	 OLED_ShowNumber(24, 26, gyro[1], 5, 12);
			 
			 if(gyro[2]<0)	 						OLED_ShowString(16, 36, "-"),
																	OLED_ShowNumber(24, 36, -gyro[2], 5, 12);
			 else						 						OLED_ShowString(16, 36, "+"),
																	OLED_ShowNumber(24, 36, gyro[2], 5, 12);
			 
			 OLED_ShowString(64, 16, "|");
			 OLED_ShowString(64, 26, "|");
			 OLED_ShowString(64, 36, "|");
			 
			 OLED_ShowString(72, 16, "A");
			 if(accel[0]<0)							 OLED_ShowString(80, 16, "-"),
																	 OLED_ShowNumber(88, 16, -accel[0], 5, 12);
			 else							 					 OLED_ShowString(80, 16, "+"),
																	 OLED_ShowNumber(88, 16, accel[0], 5, 12);
			 
			 if(accel[1]<0)							 OLED_ShowString(80, 26, "-"),
																	 OLED_ShowNumber(88, 26, -accel[1], 5, 12);
			 else							 					 OLED_ShowString(80, 26, "+"),
																	 OLED_ShowNumber(88, 26, accel[1], 5, 12);
			 
			 if(accel[2]<0)							 OLED_ShowString(80, 36, "-"),
																	 OLED_ShowNumber(88, 36, -accel[2], 5, 12);
			 else							 					 OLED_ShowString(80, 36, "+"),
																	 OLED_ShowNumber(88, 36, accel[2], 5, 12);
			 
			 OLED_ShowString(104, 50, "3/8");
		 }
		 if(Proc_Flag==6)
		 {
			 OLED_ShowCHinese(00, 00, "    单击进入    ");
			 OLED_ShowCHinese(8, 32, "模式：六路舵机");
			 OLED_ShowString(104, 50, "4/8");
		 }
		 if(Proc_Flag==7 || Proc_Flag==8 || Proc_Flag==9 || Proc_Flag==10 || Proc_Flag==11 || Proc_Flag==12)
		 {
			 OLED_ShowString(00, 00, "1:"),OLED_ShowNumber(16, 00, Servo_Count[0], 5, 12);
			 OLED_ShowString(64, 00, "2:"),OLED_ShowNumber(80, 00, Servo_Count[1], 5, 12);
			 OLED_ShowString(00, 10, "3:"),OLED_ShowNumber(16, 10, Servo_Count[2], 5, 12);
			 OLED_ShowString(64, 10, "4:"),OLED_ShowNumber(80, 10, Servo_Count[3], 5, 12);
			 OLED_ShowString(00, 20, "5:"),OLED_ShowNumber(16, 20, Servo_Count[4], 5, 12);
			 OLED_ShowString(64, 20, "6:"),OLED_ShowNumber(80, 20, Servo_Count[5], 5, 12);
				OLED_ShowCHinese(8, 32, "模式：六路舵机");
			 OLED_ShowString(104, 50, "4/8");
		 }
		 if(Proc_Flag==13)
		 {
				OLED_ShowCHinese(00, 00, "    单击进入    ");
				OLED_ShowCHinese(00, 32, "  模式：蜂鸣器  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(Proc_Flag==14)
		 {
			 OLED_ShowCHinese(24, 16, "蜂鸣器：");
			 if(Buzzer==1)		OLED_ShowCHinese(88, 16, "开");
			 else							OLED_ShowCHinese(88, 16, "关");
			 OLED_ShowCHinese(00, 32, "  模式：蜂鸣器  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(Proc_Flag==15)
		 {
			 OLED_ShowCHinese(00, 00, "  拨动使能开关  ");
			 OLED_ShowCHinese(32, 16, "开关：");
			 if(EN==1)					OLED_ShowCHinese(80, 16, "开");
			 else								OLED_ShowCHinese(80, 16, "关");
			 OLED_ShowCHinese(8, 32, "模式：使能开关");
			 OLED_ShowString(104, 50, "6/8");
		 }
		 if(Proc_Flag==16)
		 {
			  OLED_ShowCHinese(00, 00, "确认蓝牙红灯常亮");
			  OLED_ShowCHinese(24, 16, "后单击进入");
				OLED_ShowCHinese(24, 32, "模式：蓝牙");
			  OLED_ShowString(104, 50, "7/8");
		 }
		 if(Proc_Flag==17)
		 {
			 OLED_ShowCHinese(8, 00, "小车行动状态：");
			 if(uart2_send_flag==1)					OLED_ShowCHinese(48, 16, "向前");
			 else if(uart2_send_flag==2)		OLED_ShowCHinese(48, 16, "向右");
			 else if(uart2_send_flag==3)		OLED_ShowCHinese(48, 16, "向后");
			 else if(uart2_send_flag==4)		OLED_ShowCHinese(48, 16, "向左");
			 else if(uart2_send_flag==5)		OLED_ShowCHinese(48, 16, "停止");
//			 OLED_ShowCHinese(00, 00, "单击发送");
//			 OLED_ShowString16(64, 00, "WHEELTEC");
//			 OLED_ShowCHinese(00, 16, "接收的");
//			 OLED_ShowString16(48, 16, "ASCII:");
//			 OLED_ShowNumber(96, 18, (USART2->DR), 4, 12);		//在APP发送的是字符，所以这里使用Ascii码显示
			 OLED_ShowCHinese(24, 32, "模式：蓝牙");
			 OLED_ShowString(104, 50, "7/8");
		 }
		 if(Proc_Flag==18)
		 {
				OLED_ShowCHinese(8, 00, "使用数据线连接");
			  OLED_ShowCHinese(20, 16, "中间的");
			  OLED_ShowString16(68, 16, "USB");
				OLED_ShowCHinese(92,16, "口");
				OLED_ShowCHinese(20, 32, "模式：串口");
			  OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(Proc_Flag==19)
		 {
			 OLED_ShowCHinese(00, 00, "发送");
			 OLED_ShowString16(36, 00, "#");
			 OLED_ShowCHinese(48, 00, "结尾的数据");
			 
			 OLED_ShowCHinese(8, 16, "波特率：");
			 OLED_ShowString16(72, 16, "115200");
//			 OLED_ShowCHinese(00, 00, "单击发送");
//			 OLED_ShowString16(64, 00, "WHEELTEC");
//			 OLED_ShowCHinese(00, 16, "接收的");
//			 OLED_ShowString16(48, 16, "ASCII:");
//			 OLED_ShowNumber(96, 18, (USART3->DR), 4, 12);		//在串口助手上发送的是字符，所以这里使用Ascii码显示
			 OLED_ShowCHinese(20, 32, "模式：串口");
			 OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(Proc_Flag==20)
		 {
				OLED_ShowCHinese(00, 00, "即将退出质检模式");
			  OLED_ShowCHinese(00, 16, "请转动电位器回到");
			  OLED_ShowCHinese(12, 32, "当前车型：");
			  switch(Car_Mode_Show)
			  {
					case Mec_Car:       OLED_ShowString16(92,32,"Mec "); break; 
					case Omni_Car:      OLED_ShowString16(92,32,"Omni"); break; 
					case Akm_Car:       OLED_ShowString16(92,32,"Akm "); break; 
					case Diff_Car:      OLED_ShowString16(92,32,"Diff"); break; 
					case FourWheel_Car: OLED_ShowString16(92,32,"4WD "); break; 
					case Tank_Car:      OLED_ShowString16(92,32,"Tank"); break; 
			  }
				OLED_ShowString(104, 50, "8/8");
		 }
		 OLED_ShowCHinese12(00, 50, "单击继续双击退出");
	 }
	 
		OLED_Refresh_Gram();
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //对电池电压处理成百分比形式
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //车轮速度单位转换为0.01m/s，方便在APP显示
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //用于交替打印APP数据和显示波形
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //发送参数到APP，APP在调试界面显示
		 printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0) 
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //发送参数到APP，APP在首页显示
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //发送参数到APP，APP在波形界面显示
		 printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
	 }
}


