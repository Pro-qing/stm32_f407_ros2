#include "balance.h"

int Time_count=0; //Time variable //计时变量
u32 Buzzer_count1 = 0;
// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 

short test_num;

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据

u8 command_lost_count=0; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
      //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //没有使用到
		}
		
		//Ackermann structure car
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Servo=SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;

			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //舵机PWM值限幅
		}
		
		//Differential car
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
		
		//FourWheel car
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
		
		//Tank Car
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(Time_count<3000)Time_count++;
			Buzzer_count1++;
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//获取编码器数据，即车轮实时速度，并转换位国际单位
			Get_Velocity_Form_Encoder();   
			//Click the user button to update the gyroscope zero
				//单击用户按键更新陀螺仪零点
				Key();
			if(Check==0) //If self-check mode is not enabled //如果没有启动自检模式
			{
//				command_lost_count++; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
//				if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0) //不是APP、PS2、航模遥控模式，就是CAN、串口1、串口3控制模式
//					Move_X=0, Move_Y=0, Move_Z=0;

				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令
				
				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN、串口1、串口3(ROS)、串口5控制直接得到三轴目标速度，无须额外处理
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
				 
				
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
				if(Turn_Off(Voltage)==0) 
				 { 			
			//Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
						 
					 Limit_Pwm(16700);
//					 CAN1_Send_All_Motors_With_Sync();
					 //Set different PWM control polarity according to different car models
					 //根据不同小车型号设置不同的PWM控制极性
					 switch(Car_Mode)
					 {
							case Mec_Car:       CAN1_Send_All_Motors_With_Sync();break;
						 //Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
							case Omni_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //全向轮小车
							case Akm_Car:       Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //阿克曼小车
							case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //两轮差速小车
							case FourWheel_Car: Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //四驱车 
							case Tank_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //履带车
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
				 else	Set_Pwm(0,0,0,0,0); 
			 }
				else
				{
//					if(Proc_Flag==3)						//自检电机
//					{
//						 if(check_time_count_motor_forward>0)
//						 {	 
//							 check_time_count_motor_forward--;
//							 Full_rotation=16799;
//						 }
//						 else if(check_time_count_motor_retreat>0) 
//						 {	 
//							 check_time_count_motor_retreat--;
//							 Full_rotation=-16799;
//						 }		

//						 switch(Car_Mode)
//						 {
//								case Mec_Car:       Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
//								case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Omni car                //全向轮小车
//								case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Ackermann structure car //阿克曼小车
//								case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Differential car        //两轮差速小车
//								case FourWheel_Car: Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //FourWheel car           //四驱车 
//								case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Tank Car                //履带车
//						 } 
//						 if(!(check_time_count_motor_retreat>0) && !(check_time_count_motor_forward>0))
//						 {	 
//							 Set_Pwm(0,0,0,0,0);		 
//						 }
//					}
				if(Proc_Flag==4)		Set_Pwm(0,0,0,0,0);
				if(Proc_Flag==6)		TIM8_SERVO_Init(9999,168-1);					//六路舵机
				if(Proc_Flag==7)																					//控制舵机
				{
					if(servo_direction[0]==0&&Servo_Count[0]<2500) Servo_Count[0]=Servo_Count[0]+5;
				 if(servo_direction[0]==0&&Servo_Count[0]>=2500) servo_direction[0]=1;
				 if(Servo_Count[0]>500&&servo_direction[0]==1)  Servo_Count[0]=Servo_Count[0]-5;
				 if(Servo_Count[0]<=500&&servo_direction[0]==1)  Servo_Count[0]=500,servo_direction[0] = 2;
				 TIM12->CCR2=Servo_Count[0];
				 
				}
				if(Proc_Flag==8)
				{
					if(servo_direction[0]!=2)					Servo_Count[0]=500,TIM12->CCR2=Servo_Count[0];
					if(servo_direction[1]==0&&Servo_Count[1]<2500) Servo_Count[1]=Servo_Count[1]+5;
				 if(servo_direction[1]==0&&Servo_Count[1]>=2500) servo_direction[1]=1;
				 if(Servo_Count[1]>500&&servo_direction[1]==1)  Servo_Count[1]=Servo_Count[1]-5;
				 if(Servo_Count[1]<=500&&servo_direction[1]==1)  Servo_Count[1]=500,servo_direction[1] = 2;
					TIM12->CCR1=Servo_Count[1];
				}
				if(Proc_Flag==9)
				{
					if(servo_direction[1]!=2)					Servo_Count[1]=500,TIM12->CCR1=Servo_Count[1];
					if(servo_direction[2]==0&&Servo_Count[2]<2500) Servo_Count[2]=Servo_Count[2]+5;
				 if(servo_direction[2]==0&&Servo_Count[2]>=2500) servo_direction[2]=1;
				 if(Servo_Count[2]>500&&servo_direction[2]==1)  Servo_Count[2]=Servo_Count[2]-5;
				 if(Servo_Count[2]<=500&&servo_direction[2]==1)  Servo_Count[2]=500,servo_direction[2] = 2;
					TIM8->CCR4=Servo_Count[2];
				}
				if(Proc_Flag==10)
				{
					if(servo_direction[2]!=2)					Servo_Count[2]=500,TIM8->CCR4=Servo_Count[2];
					if(servo_direction[3]==0&&Servo_Count[3]<2500) Servo_Count[3]=Servo_Count[3]+5;
				 if(servo_direction[3]==0&&Servo_Count[3]>=2500) servo_direction[3]=1;
				 if(Servo_Count[3]>500&&servo_direction[3]==1)  Servo_Count[3]=Servo_Count[3]-5;
				 if(Servo_Count[3]<=500&&servo_direction[3]==1)  Servo_Count[3]=500,servo_direction[3] = 2;
					TIM8->CCR3=Servo_Count[3];
				}
				if(Proc_Flag==11)
				{
					if(servo_direction[3]!=2)					Servo_Count[3]=500,TIM8->CCR3=Servo_Count[3];
					if(servo_direction[4]==0&&Servo_Count[4]<2500) Servo_Count[4]=Servo_Count[4]+5;
				 if(servo_direction[4]==0&&Servo_Count[4]>=2500) servo_direction[4]=1;
				 if(Servo_Count[4]>500&&servo_direction[4]==1)  Servo_Count[4]=Servo_Count[4]-5;
				 if(Servo_Count[4]<=500&&servo_direction[4]==1)  Servo_Count[4]=500,servo_direction[4] = 2;
					TIM8->CCR2=Servo_Count[4];
				}
				if(Proc_Flag==12)
				{
					if(servo_direction[4]!=2)					Servo_Count[4]=500,TIM8->CCR2=Servo_Count[4];
					if(servo_direction[5]==0&&Servo_Count[5]<2500) Servo_Count[5]=Servo_Count[5]+5;
				 if(servo_direction[5]==0&&Servo_Count[5]>=2500) servo_direction[5]=1;
				 if(Servo_Count[5]>500&&servo_direction[5]==1)  Servo_Count[5]=Servo_Count[5]-5;
				 if(Servo_Count[5]<=500&&servo_direction[5]==1)  Servo_Count[5]=500,servo_direction[5] = 2;
					TIM8->CCR1=Servo_Count[5];
				}
				
				if(Proc_Flag==13)																	//
				{
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					 TIM8->CCR1=Servo_Count[5];
					 TIM8->CCR2=Servo_Count[4];
					 TIM8->CCR3=Servo_Count[3];
					 TIM8->CCR4=Servo_Count[2];
					 TIM12->CCR1=Servo_Count[1];
					 TIM12->CCR2=Servo_Count[0];
				}
				if(Proc_Flag==14)																	//蜂鸣器间隔1s响一次
				{
					if((Buzzer_count1/100)%2)			Buzzer = 1;
					else													Buzzer = 0;
				}
				if(Proc_Flag==15)			Buzzer = 0;
//				if(Proc_Flag==17)																	//向APP发送WHEELTEC
//				{
//					if(uart2_send_flag==1)
//					{
//						USART2_Return();
//						uart2_send_flag = 0;
//						app_count = 0;
//					}
//				}
				if(Proc_Flag==19)
				{
					if(uart3_send_flag==1)
					{
						USART3_Return();
						uart3_send_flag = 0;
						message_count = 0;
					}
				}
				}
		 }  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(Check==1&&tmp==1)		
	{
		Proc_Flag++;
		if(Proc_Flag==21)			
		{
			Check = 0;
			Buzzer = 0;
			Proc_Flag = 0;
			check_time_count_motor_forward=300;
			check_time_count_motor_retreat=500;
			Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
			servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
			TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
		}
	}
	if(Check==0&&tmp==2)		memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
	if(Check==1&&tmp==2)		
	{
		Check = 0;
		Buzzer = 0;
		Proc_Flag = 0;
		check_time_count_motor_forward=300;
		check_time_count_motor_retreat=500;
		Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
		servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
		TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
	}
//	if(tmp==3)							
//	{
//		Check = 1;
////		Proc_Flag = 17;
//	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//如果连续6次接近满幅输出，判断为电机乱转，让电机失能	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}



/************	 分割线		************//************	 分割线		************//************	 分割线		************//************	 分割线		************//************	 分割线		************/

// 速度转换为编码的函数
// 建立线性映射关系：编码值 = (速度 / 1500) * 0x05DC
uint16_t speed_to_hex(float speed_rpm)
{
    if(speed_rpm == 0)
	{
        return 0x0000;  // 速度为0时返回0000
	}
    // 计算编码值
    uint16_t hex_value = (uint16_t)((speed_rpm / 1500.0f) * 0x05DC);
    // 限制范围，防止溢出
    if (hex_value > 0x05DC)
	{
        hex_value = 0x05DC;
    }
    return hex_value;
}

// 将四个轮子速度转换为编码
void convert_speeds_to_hex(uint16_t* motor_codes) {
    motor_codes[0] = speed_to_hex(MOTOR_A.Target);  // 电机A编码
    motor_codes[1] = speed_to_hex(MOTOR_B.Target);  // 电机B编码
    motor_codes[2] = speed_to_hex(MOTOR_C.Target);  // 电机C编码
    motor_codes[3] = speed_to_hex(MOTOR_D.Target);  // 电机D编码
}

//转换成can数据格式
// 将四个电机速度转换为CAN发送数据数组
void convert_speeds_to_can_data(u8 can_data_arrays[4][8], u8 acceleration, u8 sync_flag) {
    // 电机A的CAN数据
    u16 speed_a = (u16)fabs(MOTOR_A.Target);
    can_data_arrays[0][0] = MOTOR_1_ID;                    // 地址
    can_data_arrays[0][1] = CAN_COMMAND_SPEED_CONTROL;     // 命令 0xF6
    can_data_arrays[0][2] = (MOTOR_A.Target >= 0) ? DIRECTION_CW : DIRECTION_CCW; // 方向
    can_data_arrays[0][3] = (speed_a >> 8) & 0xFF;        // 速度高字节
    can_data_arrays[0][4] = speed_a & 0xFF;                // 速度低字节
    can_data_arrays[0][5] = acceleration;                  // 加速度
    can_data_arrays[0][6] = sync_flag;                     // 同步标志
    can_data_arrays[0][7] = CHECKSUM_FIXED;               // 校验字节 0x6B
    
    // 电机B的CAN数据
    u16 speed_b = (u16)fabs(MOTOR_B.Target);
    can_data_arrays[1][0] = MOTOR_2_ID;
    can_data_arrays[1][1] = CAN_COMMAND_SPEED_CONTROL;
    can_data_arrays[1][2] = (MOTOR_B.Target >= 0) ? DIRECTION_CW : DIRECTION_CCW;
    can_data_arrays[1][3] = (speed_b >> 8) & 0xFF;
    can_data_arrays[1][4] = speed_b & 0xFF;
    can_data_arrays[1][5] = acceleration;
    can_data_arrays[1][6] = sync_flag;
    can_data_arrays[1][7] = CHECKSUM_FIXED;
    
    // 电机C的CAN数据
    u16 speed_c = (u16)fabs(MOTOR_C.Target);
    can_data_arrays[2][0] = MOTOR_3_ID;
    can_data_arrays[2][1] = CAN_COMMAND_SPEED_CONTROL;
    can_data_arrays[2][2] = (MOTOR_C.Target >= 0) ? DIRECTION_CW : DIRECTION_CCW;
    can_data_arrays[2][3] = (speed_c >> 8) & 0xFF;
    can_data_arrays[2][4] = speed_c & 0xFF;
    can_data_arrays[2][5] = acceleration;
    can_data_arrays[2][6] = sync_flag;
    can_data_arrays[2][7] = CHECKSUM_FIXED;
    
    // 电机D的CAN数据
    u16 speed_d = (u16)fabs(MOTOR_D.Target);
    can_data_arrays[3][0] = MOTOR_4_ID;
    can_data_arrays[3][1] = CAN_COMMAND_SPEED_CONTROL;
    can_data_arrays[3][2] = (MOTOR_D.Target >= 0) ? DIRECTION_CW : DIRECTION_CCW;
    can_data_arrays[3][3] = (speed_d >> 8) & 0xFF;
    can_data_arrays[3][4] = speed_d & 0xFF;
    can_data_arrays[3][5] = acceleration;
    can_data_arrays[3][6] = sync_flag;
    can_data_arrays[3][7] = CHECKSUM_FIXED;
}

// CAN发送数据
// 发送所有电机的CAN数据



/**************************************************************************
Function: 发送单个电机控制数据
Input   : motor_id: 电机ID (MOTOR_1_ID, MOTOR_2_ID, MOTOR_3_ID, MOTOR_4_ID)
          motor_data: 8字节电机控制数据数组
Output  : 0-成功, 1-失败
**************************************************************************/
u8 CAN1_Send_Motor_Data(u8 motor_id, u8* motor_data)
{
    u8 mbox;
    u16 i = 0;
    
    // 发送电机控制数据，使用电机自己的ID作为CAN ID
    mbox = CAN1_Tx_Msg(motor_id, 0, 0, 8, motor_data);
    
    // 等待发送完成
    while((CAN1_Tx_Staus(mbox) != 0x07) && (i < 0xFFF)) {
        i++;
    }
    
    if(i >= 0xFFF) {
        return 1;  // 发送失败
    }
    return 0;  // 发送成功
}

/**************************************************************************
Function: 发送同步启动信号
Input   : 无
Output  : 0-成功, 1-失败
注意：同步启动信号格式为 00 FF 66 6B
**************************************************************************/
u8 CAN1_Send_Sync_Start(void)
{
    u8 mbox;
    u16 i = 0;
    u8 sync_data[8] = {
        SYNC_START_ID,        // 0x00 - 地址
        SYNC_START_CMD1,      // 0xFF - 同步启动命令1
        SYNC_START_CMD2,      // 0x66 - 同步启动命令2
        SYNC_START_CHECKSUM,  // 0x6B - 校验位
        0x00, 0x00, 0x00, 0x00  // 剩余字节填0
    };
    
    // 同步启动信号发送到CAN ID 0x00
    mbox = CAN1_Tx_Msg(SYNC_START_ID, 0, 0, 8, sync_data);
    
    // 等待发送完成
    while((CAN1_Tx_Staus(mbox) != 0x07) && (i < 0xFFF)) {
        i++;
    }
    
    if(i >= 0xFFF) {
        return 1;  // 发送失败
    }
    return 0;  // 发送成功
}

/**************************************************************************
Function: 发送所有电机控制数据并启动同步运动
Input   : 无（使用预定义的参数）
Output  : 0-全部成功, 1-有发送失败
**************************************************************************/
u8 CAN1_Send_All_Motors_With_Sync(void)
{
    u8 can_data_arrays[4][8];  // 四个电机的CAN数据
    u8 result = 0;  // 结果标志
    u8 motor_ids[4] = {MOTOR_1_ID, MOTOR_2_ID, MOTOR_3_ID, MOTOR_4_ID};
    
    // 1. 转换电机速度为CAN数据格式，使用你指定的参数
    convert_speeds_to_can_data(can_data_arrays, MOTOR_ACCELERATION, MOTOR_SYNC_FLAG);
    
    // 2. 依次发送四个电机的控制数据
    for(u8 i = 0; i < 4; i++) {
        if(CAN1_Send_Motor_Data(motor_ids[i], can_data_arrays[i]) != 0) {
            result = 1;  // 标记有发送失败
        }
        // 添加小延时确保数据发送间隔
        // delay_us(500);  // 根据需要调整延时
    }
    
    // 3. 发送同步启动信号（因为同步标志为0x01）
    if(CAN1_Send_Sync_Start() != 0) {
        result = 1;  // 标记同步信号发送失败
    }
    
    return result;
}


