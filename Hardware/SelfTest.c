#include "stm32f10x.h"                  // Device header
#include  "SelfTest.h"
#include  "DPWM.h"
#include  "Servo.h"
#include  "ADCServo.h"
#include "Delay.h"
#include  "MotorDriver.h"
#include "Motor.h"
#include <stdlib.h>
#include <math.h> 
#include "SelfTest.h"
/* 硬件测试参数 */
/*----------------------------------行走直流电机-----------------------------------------------------*/
#define MOTOR_TEST_SPEED     60    // 电机测试速度（0-100）
#define MOTOR_TEST_DURATION  800   // 单方向测试时间(ms）
/*---------------------------------升降直流电机--------------------------------------------------------*/
#define LIFT_TEST_SPEED     60    // 测试速度百分比
#define LIFT_TEST_DURATION  1200  // 单方向测试时间(ms）
ErrorCode systemError = ERR_NONE; 
uint8_t Get_Servo_Feedback(void);
int  Self_Check_Routine(void) //自检
{
    
    /* 阶段1：闭合位置检测 */
    Servo_SetAngle(0);       // 发送0度指令
	Servo2_SetAngle(0);  
    Delay_ms(800);           // 等待机械运动完成
    
	
    uint8_t currentAngle = Get_Servo_Feedback();
    // 实际角度>5度视为故障（考虑机械间隙）
    if(currentAngle > 5) {   
        systemError = ERR_SERVO_RANGE;
        goto CHECK_FAILED;
    }

    /* 阶段2：中间位置检测 */ 
    Servo_SetAngle(90);
	Servo2_SetAngle(90); 
    Delay_ms(500);           // 中间位置运动较快
    
    currentAngle = Get_Servo_Feedback();
    // 允许±10度偏差（考虑电位器线性度）
    if(abs((int8_t)currentAngle - 45) > 10) { 
        systemError = ERR_SERVO_RANGE;
        goto CHECK_FAILED;
    }

    /* 阶段3：张开位置检测 */
    Servo_SetAngle(160);
	Servo2_SetAngle(160); 
    Delay_ms(800);           // 确保完全张开
    
    currentAngle = Get_Servo_Feedback();
    // 完全张开应≥175度（防止机械过冲）
    if(currentAngle < 100) {  
        systemError = ERR_SERVO_RANGE;
        goto CHECK_FAILED;
    }
	    /*-------------------------------- 行走电机自检 --------------------------------*/
    // 左电机正反转测试
    Motor1_SetSpeed(MOTOR_TEST_SPEED);
    Delay_ms(MOTOR_TEST_DURATION);
    Motor1_SetSpeed(-MOTOR_TEST_SPEED);
    Delay_ms(MOTOR_TEST_DURATION);
    Motor1_SetSpeed(0);

    // 右电机正反转测试
    Motor2_SetSpeed(MOTOR_TEST_SPEED);
    Delay_ms(MOTOR_TEST_DURATION);
    Motor2_SetSpeed(-MOTOR_TEST_SPEED);
    Delay_ms(MOTOR_TEST_DURATION);
    Motor2_SetSpeed(0);
	/*------------------------------------升降电机-------------------------------*/
	/* 上升测试 */
    SMotor3_SetSpeed(LIFT_TEST_SPEED);  // 正向速度
    Delay_ms(LIFT_TEST_DURATION);
    
    /* 短暂暂停观察 */
    SMotor3_SetSpeed(0);
    Delay_ms(500);  // 暂停500ms确认位置
    
    /* 下降测试 */  
    SMotor3_SetSpeed(-LIFT_TEST_SPEED); // 反向速度
    Delay_ms(LIFT_TEST_DURATION);
    
    /* 结束归位 */
    SMotor3_SetSpeed(0);
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	Delay_s(3);
	GPIO_SetBits(GPIOB,GPIO_Pin_14);\
    // 测试通过
	
	Servo_SetAngle(180);
	
    return 1; 

CHECK_FAILED:
    /* 错误处理 */
    TIM_Cmd(TIM2, DISABLE);        // 关闭PWM输出
    GPIO_ResetBits( GPIOA , GPIO_Pin_1); // 强制拉低引脚
	MotorDriverFullStop();                 // 电机急停
	SMotor3_SetSpeed(0);   // 停止升降
	return 0;
    }
