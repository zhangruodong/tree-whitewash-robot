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
#include "system.h"
#include "string.h"
#include "SelfTest.h"
#include "PumpBuzzer.h"
#include "Servo.h"
#include "Motor.h"

// 升降电机方向（80=升、-80=降，已实机确认）
#define LIFT_UP_SPEED      80
#define LIFT_DOWN_SPEED   -80
// 升降全行程时间（毫秒，按实机校准）
#define LIFT_UP_TIME_MS    10000   // 低位→高位（10秒）
#define LIFT_DOWN_TIME_MS  10000   // 高位→低位（10秒）

// 全局变量：1ms 定时计数（TIM1 每 1ms 中断一次）
volatile uint32_t timer_counter = 0;
static SystemCtrl sys_ctrl = {STATE_WAIT, 0};
static RunMode run_mode = MODE_AUTO;   // 当前模式：自动 / 急停锁定
static bool paint_first_enter = true;  // 涂白状态首次进入标志
static uint8_t lift_is_up = 0;         // 升降当前位置：0=低位(下降位)，1=高位

// TIM1初始化函数（1ms 节拍）
void TIM1_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 使能TIM1时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 2. 配置定时器参数 */
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;     // 预分频值：72MHz / 7200 = 10KHz
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;          // 自动重装值：10KHz / 10 = 1ms 中断一次
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* 3. 使能定时器更新中断 */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    /* 4. 配置NVIC中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 5. 启动定时器 */
    TIM_Cmd(TIM1, ENABLE);
}

// GPIO初始化函数：PB15为输出
void GPIO15_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 配置PB15为推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


// TIM1更新中断服务函数
void TIM1_UP_IRQHandler(void) {
    static uint16_t heartbeat_div = 0;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除中断标志
        timer_counter++;                            // 每 1ms 加一

        // 心跳灯：每 500ms 翻转一次，保持 1Hz 可见闪烁（不随 1ms 节拍高频翻转）
        if (++heartbeat_div >= 500) {
            heartbeat_div = 0;
            GPIO_WriteBit(GPIOB, GPIO_Pin_15,
                         (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15)));
        }
    }
}

void Hardware_Init(void)  //硬件初始化
{
	Servo_Init();		  //舵机初始化
	Servo2_Init();		  //舵机2初始化
	ServoMotion_Init();	  //舵机平滑运动初始化
	MotorDriver1_Init();										//电机驱动模块1初始化（左）
	MotorDriver2_Init();										//电机驱动模块2初始化（右）
	SMotor3_Init();		  //直流电机初始化（直流电机3，升降用）
	BUMP_Init();		  //水泵初始化
	HC_SR04_Init();		  //超声波
	Timer_Init(); 		  //定时器
	FMQ_Init();			  //蜂鸣器初始化
	OLED_Init();
	Serial_Init();
	GPIO15_Init();
	TIM1_Init();
}

// 统一停止所有执行器（急停 / 进入等待时调用）
// 只用可恢复的停止方式：速度清零 + 关水泵/蜂鸣器；
// 不碰 TIM2 和 PA1，这样 'G' 恢复后电机还能正常转。
static void StopAll(void) {
    MotorDriverFullStop();   // 两个驱动轮停
    SMotor3_SetSpeed(0);     // 升降电机停
    BUMP_GUAN();             // 水泵关
    FMQ_GUAN();              // 蜂鸣器关
    Servo_StopAllMotions();  // 取消进行中的舵机平滑运动
    paint_first_enter = true;// 复位涂白首次进入标志
}

// 离开涂白状态时的收尾：关泵、停升降、关臂、复位首次进入标志
static void PaintCleanup(void) {
    BUMP_GUAN();
    SMotor3_SetSpeed(0);
    Servo_StartSmoothMotion(0, 80.0f, 40.0f, 2000);   // 关臂
    Servo_StartSmoothMotion(1, 50.0f, 90.0f, 2000);
    paint_first_enter = true;
}

// 处理串口指令：
//   'T' = 急停：停所有执行器 + 锁定（最高优先级，任何动作下都立即生效）
//   'G' = 恢复自动：回到等待状态，先停住等人
//   其余字符 = 流程指令，只在自动模式下生效
static void ProcessCommand(void) {
    if (Serial_GetRxFlag()) {
        uint8_t cmd = Serial_GetRxData();

        switch (cmd) {
        case 'T':                 // 急停：其他状态停所有+锁机；手动升/降时停+记位置+回等待（不锁）
            if (sys_ctrl.state == STATE_LIFT_UP) {
                lift_is_up = 1;               // 往上走时收到停止 → 默认在顶端
                SMotor3_SetSpeed(0);          // 停升降
                sys_ctrl.state = STATE_WAIT;  // 不锁机，直接回等待
            } else if (sys_ctrl.state == STATE_LIFT_DOWN) {
                lift_is_up = 0;               // 往下走时收到停止 → 默认在底端
                SMotor3_SetSpeed(0);
                sys_ctrl.state = STATE_WAIT;
            } else {
                StopAll();                    // 其他状态：急停 + 锁机
                run_mode = MODE_STOP;
            }
            break;
        case 'G':                 // 恢复自动：先停住等人
            run_mode = MODE_AUTO;
            sys_ctrl.state = STATE_WAIT;
            break;
        default:                  // 流程指令
            if (run_mode == MODE_AUTO) {
                switch (cmd) {
                case 'W': sys_ctrl.state = STATE_WAIT;           break;
                case 'N': sys_ctrl.state = STATE_NO_TREE;        break;
                case 'S': sys_ctrl.state = STATE_SEARCH_TREE;    break;
                case 'U': sys_ctrl.state = STATE_APPROACH_TREE;  break;
                case 'P': sys_ctrl.state = STATE_PAINT_PREP;     break;
                case 'R': sys_ctrl.state = STATE_RETREAT;        break;
                case 'B': sys_ctrl.state = STATE_RETREAT_BEEP;   break;
                case 'A': sys_ctrl.state = STATE_LIFT_UP;        break;
                case 'X': sys_ctrl.state = STATE_LIFT_DOWN;      break;
                }
            }
            break;
        }
    }
}

uint32_t GetTick(void) {
    return timer_counter;  // 返回毫秒级计时
}

void System_StateMachine(void) {
    uint32_t current_time = GetTick();
    ProcessCommand();               // 先处理指令（含急停/恢复）

    if (run_mode != MODE_AUTO) {    // 急停锁定时，不跑状态机
        return;
    }

    // 状态切换检测：状态变了就重置时间戳；离开涂白状态时收尾（关泵/关臂）
    static SystemState last_state = STATE_WAIT;
    if (sys_ctrl.state != last_state) {
        if (last_state == STATE_PAINT_PREP) {
            PaintCleanup();
        }
        last_state = sys_ctrl.state;
        sys_ctrl.state_timestamp = current_time;
    }

    Servo_UpdateAllMotions();       // 非阻塞：推进舵机平滑运动

    switch (sys_ctrl.state) {
    case STATE_WAIT:
        StopAll();                  // 等待时确保执行器停干净
        break;

    case STATE_NO_TREE:             // 没找到树状态：直接转原地寻找状态
        sys_ctrl.state = STATE_SEARCH_TREE;
        break;

    case STATE_SEARCH_TREE:
        Motor1_SetSpeed(90);        // 原地旋转寻找
        Motor2_SetSpeed(-90);
        // 超时没找到，就回到等待指令状态
        if (current_time - sys_ctrl.state_timestamp >= 30000) {
            OLED_Clear();
            OLED_ShowString(2, 1, "chaoshi");
            MotorDriverFullStop();
            sys_ctrl.state = STATE_WAIT;
        }
        break;

    case STATE_APPROACH_TREE:
        US_Update();                        // 非阻塞推进测距状态机
        // 距离够近（≤5cm）就开始涂白准备；超时就转回等待指令状态
        if (EnhancedUltrasonicControl()) {
            sys_ctrl.state = STATE_PAINT_PREP;
        } else if (current_time - sys_ctrl.state_timestamp >= 90000) {
            sys_ctrl.state = STATE_WAIT;
        }
        break;

    case STATE_PAINT_PREP: {        // 涂白状态：根据当前位置决定涂白方向
        if (paint_first_enter) {
            BUMP_KAI();
            Servo_StartSmoothMotion(0, 40.0f, 80.0f, 2000);   // 开臂（喷头伸出）
            Servo_StartSmoothMotion(1, 90.0f, 50.0f, 2000);
            // 低位向上涂、高位向下涂（边动边喷）
            if (lift_is_up) {
                SMotor3_SetSpeed(LIFT_DOWN_SPEED);   // 高位 → 向下涂
            } else {
                SMotor3_SetSpeed(LIFT_UP_SPEED);     // 低位 → 向上涂
            }
            paint_first_enter = false;
        }

        // 走完一个行程后结束涂白（方向不同时长不同）
        uint32_t travel_ms = lift_is_up ? LIFT_DOWN_TIME_MS : LIFT_UP_TIME_MS;
        if (current_time - sys_ctrl.state_timestamp >= travel_ms) {
            lift_is_up = !lift_is_up;   // 位置翻转
            sys_ctrl.state = STATE_RETREAT;
        }
    }
    break;

    case STATE_RETREAT:             // 后退状态：后退后转提示音状态
        SMotor3_SetSpeed(0);
        Motor1_SetSpeed(-80);
        Motor2_SetSpeed(-80);
        if (current_time - sys_ctrl.state_timestamp > 5000) {
            MotorDriverFullStop();
            sys_ctrl.state = STATE_RETREAT_BEEP;
        }
        break;

    case STATE_RETREAT_BEEP:        // 后退提示音：响完后到等待状态
        FMQ_KAI();
        if (current_time - sys_ctrl.state_timestamp > 2000) {
            FMQ_GUAN();
            sys_ctrl.state = STATE_WAIT;
        }
        break;

    case STATE_LIFT_UP:             // 手动升：走到头自动停
        if (lift_is_up) {           // 已经在高位，不用动
            sys_ctrl.state = STATE_WAIT;
            break;
        }
        SMotor3_SetSpeed(LIFT_UP_SPEED);
        if (current_time - sys_ctrl.state_timestamp >= LIFT_UP_TIME_MS) {
            SMotor3_SetSpeed(0);
            lift_is_up = 1;
            sys_ctrl.state = STATE_WAIT;
        }
        break;

    case STATE_LIFT_DOWN:           // 手动降：走到头自动停
        if (!lift_is_up) {          // 已经在低位，不用动
            sys_ctrl.state = STATE_WAIT;
            break;
        }
        SMotor3_SetSpeed(LIFT_DOWN_SPEED);
        if (current_time - sys_ctrl.state_timestamp >= LIFT_DOWN_TIME_MS) {
            SMotor3_SetSpeed(0);
            lift_is_up = 0;
            sys_ctrl.state = STATE_WAIT;
        }
        break;
    }
}
