
#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"

/*--------Motor_A control pins--------*/
#define PWM_PORTA1 GPIOB			 //PWMA
#define PWM_PIN_A1 GPIO_Pin_8 //PWMA
#define PWMA1 	  TIM10->CCR1	 //PWMA

#define PWM_PORTA2 GPIOB			 //PWMA
#define PWM_PIN_A2 GPIO_Pin_9 //PWMA
#define PWMA2 	  TIM11->CCR1	 //PWMA
/*------------------------------------*/

/*--------Motor_B control pins--------*/
#define PWM_PORTB1 GPIOE			 //PWMB
#define PWM_PIN_B1 GPIO_Pin_5 //PWMB
#define PWMB1 	  TIM9->CCR1	 //PWMB

#define PWM_PORTB2 GPIOE			 //PWMB
#define PWM_PIN_B2 GPIO_Pin_6 //PWMB
#define PWMB2 	  TIM9->CCR2	 //PWMB

/*------------------------------------*/

/*--------Motor_C control pins--------*/
#define PWM_PORTC1 GPIOE			 //PWMC
#define PWM_PIN_C1 GPIO_Pin_11 //PWMC
#define PWMC1 	  TIM1->CCR2	 //PWMC

#define PWM_PORTC2 GPIOE			 //PWMC
#define PWM_PIN_C2 GPIO_Pin_9 //PWMC
#define PWMC2 	  TIM1->CCR1	 //PWMC

/*------------------------------------*/

/*--------Motor_D control pins--------*/
#define PWM_PORTD1 GPIOE			 //PWMD
#define PWM_PIN_D1 GPIO_Pin_14 //PWMD
#define PWMD1 	  TIM1->CCR4	 //PWMD

#define PWM_PORTD2 GPIOE			 //PWMD
#define PWM_PIN_D2 GPIO_Pin_13 //PWMD
#define PWMD2 	  TIM1->CCR3	 //PWMD

/*------------------------------------*/
#define EN     PDin(3)  

#define Servo_PWM  TIM12->CCR2
#define SERVO_INIT 1500  //Servo zero point //舵机零点


/************	 分割线		************//************	 分割线		************//************	 分割线		************//************	 分割线		************/
/*42motor*/
// 电机ID定义
#define MOTOR_1_ID  2
#define MOTOR_2_ID  3
#define MOTOR_3_ID  4
#define MOTOR_4_ID  5

// CAN通讯命令定义
#define CAN_COMMAND_SPEED_CONTROL  0xF6

// 固定校验值
#define CHECKSUM_FIXED  0x6B

// 方向定义
#define DIRECTION_CW   0x00  // 00表示CW
#define DIRECTION_CCW  0x01  // 01表示CCW

// 同步标志定义
#define SYNC_DISABLE   0x00  // 00表示不启用多机同步
#define SYNC_ENABLE    0x01  // 01表示启用多机同步

// 你的控制参数定义
#define MOTOR_ACCELERATION    0x0A  // 加速度参数
#define MOTOR_SYNC_FLAG       0x01  // 同步标志

// 同步启动信号定义
#define SYNC_START_ID         0x00  // 同步启动信号的CAN ID
#define SYNC_START_CMD1       0xFF  // 同步启动命令1
#define SYNC_START_CMD2       0x66  // 同步启动命令2
#define SYNC_START_CHECKSUM   0x6B  // 同步启动校验位

// 电机控制数据结构
typedef struct {
    uint8_t motor_id;       // 电机ID
    uint8_t direction;      // 方向：0-CW, 1-CCW
    uint16_t speed;         // 速度 RPM
    uint8_t acceleration;   // 加速度档位 0-255
    uint8_t sync_flag;      // 多机同步标志：0-不启用, 1-启用
} MotorControlData_t;

// 电机控制命令帧结构
typedef struct {
    uint8_t address;        // 地址
    uint8_t command;        // 命令字 0xF6
    uint8_t direction;      // 方向
    uint8_t speed_high;     // 速度高字节
    uint8_t speed_low;      // 速度低字节
    uint8_t acceleration;   // 加速度
    uint8_t sync_flag;      // 同步标志
    uint8_t checksum;       // 校验字节
} MotorCommand_t;

//补充函数声明
u8 CAN1_Send_All_Motors_With_Sync(void);
u8 CAN1_Send_Sync_Start(void);
u8 CAN1_Send_Motor_Data(u8 motor_id, u8* motor_data);
void convert_speeds_to_can_data(u8 can_data_arrays[4][8], u8 acceleration, u8 sync_flag);
void convert_speeds_to_hex(uint16_t* motor_codes);

/************	 分割线		************//************	 分割线		************//************	 分割线		************/

void Enable_Pin(void);
void Servo_PWM_Init(u16 arr,u16 psc);
void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM9_PWM_Init(u16 arr,u16 psc);
void TIM10_PWM_Init(u16 arr,u16 psc);
void TIM11_PWM_Init(u16 arr,u16 psc);			
	


#endif
