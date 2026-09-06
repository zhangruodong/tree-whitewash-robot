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
#include "MotorDriver.h"
#include "USART.h"
#include "string.h"
#include "SelfTest.h"
#include "PumpBuzzer.h"
#include "Servo.h"
#include "Motor.h"

/* 系统状态枚举（值就是串口指令字符） */
typedef enum {
    STATE_WAIT = 'W',           // 等待指令
    STATE_NO_TREE = 'N',        // 没找到树
    STATE_SEARCH_TREE = 'S',    // 原地寻找树
    STATE_APPROACH_TREE = 'U',  // 接近树木
    STATE_PAINT_PREP = 'P',     // 开始涂白准备
    STATE_RETREAT = 'R',        // 后退
    STATE_RETREAT_BEEP = 'B',   // 后退提示音
    STATE_LIFT_UP = 'A',        // 手动升
    STATE_LIFT_DOWN = 'X'       // 手动降
} SystemState;

/* 运行模式：自动 / 急停锁定 */
typedef enum {
    MODE_AUTO = 0,   // 自动：听摄像头流程
    MODE_STOP = 1    // 急停：锁住，只有 'G' 指令能恢复
} RunMode;

typedef struct {
    SystemState state;           // 当前系统状态
    uint32_t state_timestamp;    // 状态进入时间（毫秒）
} SystemCtrl;

/* 函数声明 */
void System_StateMachine(void);
uint32_t GetTick(void);
void Hardware_Init(void);
void TIM1_Init(void);
void GPIO15_Init(void);
void TIM1_UP_IRQHandler(void);

#endif
