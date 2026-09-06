#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "ZPWM.h"
#include  "Delay.h"
#include  "HCSR04.h"

/*----------------------------------行走————————————————-----------------*/
/* 运动控制参数宏定义 */
#define MAX_SPEED      100    // 最大速度限制（对应100% PWM占空比）
#define STEER_GAIN     0.6f   // 转向灵敏度调节系数（0.1-1.0）
#define SPIN_SPEED     80     // 原地旋转速度百分比（建议值：60-90）
#define TURN_STRENGTH  50     // 行进中转向强度（建议值：30-70）
/*---------------------------------超声波控制速度----------------------------*/
#define SAFE_DIST_FAR     40    // 全速前进距离阈值（单位：厘米，≥此值触发全速前进）
#define SAFE_DIST_NEAR    20    // 线性减速起始距离（单位：厘米，20cm-40cm区间开始减速）
#define SAFE_DIST_STOP    5     // 紧急停止绝对距离（单位：厘米，≤5cm立即刹停）
#define BASE_SPEED_FORWARD 80   // 正向基准速度（0-100对应PWM占空比，建议不超过电机额定值）
#define BASE_SPEED_BACKWARD 60  // 反向基准速度（绝对值，建议低于正向速度防止电流冲击）
/**
  * @brief  电机驱动模块1初始化（左）
  * @note   初始化电机1的PWM和方向控制GPIO
  */
void MotorDriver1_Init(void){
	ZPWM1_Init(); 		  //直流电机PWM初始化（直流电机1，轮子）
	Motor1_Init();		  //直流电机初始化（直流电机1，轮子）
}

/**
  * @brief  电机驱动模块2初始化
  * @note   初始化电机2的PWM和方向控制GPIO
  */
void MotorDriver2_Init(void){
	ZPWM2_Init();  		  //直流电机PWM初始化（直流电机2，轮子）
	Motor2_Init();		  //直流电机初始化（直流电机2，轮子）
}

// 全速前进
void MoveForward(void){
    Motor1_SetSpeed(100);  // 左电机正转100%
    Motor2_SetSpeed(100);  // 右电机正转100%
}
//全速后进
void MoveBackward(void){
    Motor1_SetSpeed(-100); // 左电机反转100%
    Motor2_SetSpeed(-100); // 右电机反转100%
}

//刹车停止
void  MotorDriverFullStop(void){
	 Motor1_SetSpeed(0); // 左电机
     Motor2_SetSpeed(0); // 右电机
}
/********************* 差速转向控制函数 *********************/
/**
  * @brief  智能差速转向控制算法
  * @param  base_speed : 基准速度（-100~100）
  * @param  steer      : 转向系数（-100左转 ~ 100右转）
  * @note   基于差速驱动模型：V_left = V_linear - Δ，V_right = V_linear + Δ
  */
void SmartSteering(int8_t base_speed, int8_t steer) {
    // 输入参数有效性校验
    base_speed = (base_speed < -MAX_SPEED) ? -MAX_SPEED : 
                ((base_speed > MAX_SPEED) ? MAX_SPEED : base_speed);//基准速度限幅
    steer = (steer < -100) ? -100 : ((steer > 100) ? 100 : steer);//转向系数限幅

     /* 差速计算 */
    float delta = steer * STEER_GAIN;         // 计算速度差量
    int8_t left_speed = base_speed - delta;   // 左轮速度 = 基准 - 差量
    int8_t right_speed = base_speed + delta;  // 右轮速度 = 基准 + 差量

    // 二次限幅保护
    left_speed = (left_speed < -MAX_SPEED) ? -MAX_SPEED : 
                ((left_speed > MAX_SPEED) ? MAX_SPEED : left_speed);
    right_speed = (right_speed < -MAX_SPEED) ? -MAX_SPEED : 
                 ((right_speed > MAX_SPEED) ? MAX_SPEED : right_speed);

    // 设置实际速度
    Motor1_SetSpeed(left_speed);   // 左电机
    Motor2_SetSpeed(right_speed);  // 右电机
}
 
// 原地左转（左轮后退，右轮前进）
void SpinLeft(void) {
    // 左轮反转，右轮正转（原地逆时针旋转）
    Motor1_SetSpeed(-SPIN_SPEED);  // PA4=0, PA5=1
    Motor2_SetSpeed(SPIN_SPEED);    // PA8=1, PA9=0
}

// 原地右转（右轮后退，左轮前进）
void SpinRight(void) {
    // 左轮正转，右轮反转（原地顺时针旋转）
    Motor1_SetSpeed(SPIN_SPEED);    // PA4=1, PA5=0
    Motor2_SetSpeed(-SPIN_SPEED);   // PA8=0, PA9=1
}

// 行进中左转（保持基准速度，添加左转差速）
void TurnLeftWhileMoving(int8_t base_speed) 
{
    SmartSteering(base_speed, -TURN_STRENGTH); 
}

// 行进中右转（保持基准速度，添加右转差速）
void TurnRightWhileMoving(int8_t base_speed) {
    SmartSteering(base_speed, TURN_STRENGTH);  
}






/*————-----------------------------------------超声波控制速度-------------------------------------------------------*/
/**


  * 控制逻辑详解：
  * 1. 距离分区管理：
  *    - [40cm, +∞)  全速前进
  *    - [20cm, 40cm) 线性减速前进
  *    - [5cm, 20cm)  线性加速后退
  *    - [0cm, 5cm)   紧急制动
  * 2. 速度曲线：
  *    前进方向速度由80%线性降至0%，后退方向速度由0%线性升至60%
  * 3. 特别处理：
  *    - 后退速度采用负值传递，需确保电机驱动函数能正确处理方向
  *    - 所有距离比较均为闭区间检查，防止临界值跳变
  */
int EnhancedUltrasonicControl(void) 
{
    /* 获取实时距离 */
    uint16_t dist = range(); // range()需返回厘米级距离值，最大不超过655cm（uint16_t限制）

    /********************** 紧急制动区域 **********************/
    if (dist < SAFE_DIST_STOP) {
        // 触发条件：距离≤5cm
        MotorDriverFullStop(); // 立即切断电机动力
        return 1; // 退出控制循环，避免后续操作影响制动
    }
    
    /********************** 避障后退区域 **********************/
    //else if (dist < SAFE_DIST_NEAR) {
        /* 
         * 计算后退速度（负速度表示反向）：
         * speed = -BASE_BACKWARD * (SAFE_DIST_NEAR - dist) / (SAFE_DIST_NEAR - SAFE_DIST_STOP)
         * 当dist=5cm时：speed = -60*(20-5)/(20-5) = -60 （全速后退）
         * 当dist=20cm时：speed = -60*(20-20)/(20-5) = 0   （停止）
         * 实现5cm→20cm区间速度从-60到0的线性变化
         */
        //int8_t speed = -(BASE_SPEED_BACKWARD * (SAFE_DIST_NEAR - dist) / 
                        //(SAFE_DIST_NEAR - SAFE_DIST_STOP));
        
        /* 直行后退（steer=0表示无转向差速） */
       // SmartSteering(speed, 0); // 依赖已实现的差速转向函数
   //}
    
    /********************** 线性减速前进区域 **********************/
    else if (dist < SAFE_DIST_FAR) {
        /* 
         * 计算前进速度：
         * speed = BASE_FORWARD * (dist - SAFE_DIST_NEAR) / (SAFE_DIST_FAR - SAFE_DIST_NEAR)
         * 当dist=20cm时：speed = 80*(20-20)/(40-20) = 0    （停止）
         * 当dist=40cm时：speed = 80*(40-20)/(40-20) = 80   （全速前进）
         * 实现20cm→40cm区间速度从0到80的线性变化
         */
        int8_t speed = BASE_SPEED_FORWARD * (dist - SAFE_DIST_NEAR) / 
                      (SAFE_DIST_FAR - SAFE_DIST_NEAR);
        
        /* 直行前进（steer=0表示无转向差速） */
        SmartSteering(speed, 0); 
    }
    
    /********************** 全速前进区域 **********************/
    else {
        // 安全区域直行全速前进
        MoveForward(); // 调用预定义的全速前进函数
    }
}
