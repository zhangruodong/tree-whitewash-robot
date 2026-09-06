#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "stm32f10x.h"
#include <stdlib.h>
#include <math.h> 
#include <stdio.h>
#include <stdbool.h>
#include "Delay.h"
#include "OLED.h"
#include "HCSR04.h"
#include "ADCServo.h"
#include "MotorDriver.h"
#include "USART.h"
#include "system.h"
#include "string.h"
#include "SelfTest.h"
#include  "PumpBuzzer.h"
#include  "Servo.h"
#include  "Motor.h"
/* 系统状态枚举 */
// 先声明枚举类型
typedef enum {
    STATE_WAIT = 'W',   // 等待指令
	STATE_NO_TREE='N',//没有找到树
    STATE_SEARCH_TREE = 'S',// 搜索树木
    STATE_APPROACH_TREE = 'U',//接近树木
    STATE_PAINT_PREP = 'P', // 喷涂准备
    STATE_RETREAT = 'R',    // 后撤
    STATE_RETREAT_BEEP = 'B',// 蜂鸣提示
	STATE_TING='T'
} SystemState;
typedef struct {
    SystemState state;           // 当前系统状态
    uint32_t state_timestamp;    // 状态进入时间戳（毫秒）
    uint8_t last_cmd;      // 待处理指令            // 最新接收的串口指令
  
} SystemCtrl;

/* 公共函数声明 */
void System_StateMachine(void);
uint32_t GetTick(void);
void Hardware_Init(void);
void TIM1_Init(void);
void GPIO15_Init(void);
void TIM1_UP_IRQHandler(void);
void Hardware_Init(void);
void System_StateMachine(void) ;

///* 在system.h中添加指令宏定义 */
//#define CMD_WAIT 'W'	//等待指令
//#define CMD_NO_TREE      'N'  // 未发现树木  
//#define CMD_SEARCH_TREE  'S'	//原地搜寻
////#define CMD_UNTREATED  	 'U'  // 发现未涂白树
//#define CMD_APPROACH_TREE  'U'	////接近树木
//#define CMD_PAINT_PREP 	'P' // 喷涂准备
//#define CMD_RETREAT  'R'   // 后撤
//#define CMD_RETREAT_BEEP  'B'// 蜂鸣提示
//#define CMD_TING         'T'  // 停止当前动作


#endif
