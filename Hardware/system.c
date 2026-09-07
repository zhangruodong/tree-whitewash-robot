#include "stm32f10x.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_adc.h"
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

// 升降底端限位开关：PB7（输入上拉，开关另一端接 GND，压下=低电平）
#define LIFT_LIMIT_PORT    GPIOB
#define LIFT_LIMIT_PIN     GPIO_Pin_7

// 串口通信超时：非等待状态超过此时间没收到任何指令，自动回安全等待态。
// 需大于最长状态"接近树木"的 90 秒，避免误触发；按 AI 实际指令节奏可调小。
#define COMM_TIMEOUT_MS  120000

// ===== 电池电压检测 + 欠压保护 =====
// 12V 锂电池 = 3 串三元（3S）：满电 12.6V，标称 11.1V，欠压保护 9.0V。
// 分压：上臂 R1=100k 接电池+，下臂 R2=33k 接 GND，中点接 PA4（ADC_IN4）。
// 分压比 33/(100+33)=0.248，ADC 满量程 4095 对应 13.30V。
#define BAT_ADC_PORT       GPIOA
#define BAT_ADC_PIN        GPIO_Pin_4
#define BAT_ADC_CHANNEL    ADC_Channel_4
#define BAT_ADC_SAMPLE     ADC_SampleTime_239Cycles5   // 长采样时间匹配 100k 高源阻抗
#define BAT_FULL_SCALE_MV  13300   // ADC 满量程对应电池电压（mV）
#define BAT_UNDER_MV       9000    // 欠压保护阈值 9.0V（单节 3.0V）
#define BAT_RECOVER_MV     9600    // 恢复阈值 9.6V（迟滞 0.6V，防振荡）
#define BAT_LOW_CNT        3       // 连续越界 N 次才动作（去抖）
#define BAT_SAMPLE_MS      500     // 采样周期（ms）
#define BAT_AVG_N          8       // 滑动平均窗口

// 全局变量：1ms 定时计数（TIM1 每 1ms 中断一次）
volatile uint32_t timer_counter = 0;
static SystemCtrl sys_ctrl = {STATE_WAIT, 0};
static RunMode run_mode = MODE_AUTO;   // 当前模式：自动 / 急停锁定
static bool paint_first_enter = true;  // 涂白状态首次进入标志
static uint8_t lift_is_up = 0;         // 升降当前位置：0=低位(下降位)，1=高位
static uint32_t last_cmd_time = 0;     // 最后收到串口指令的时间戳（通信超时保护用）

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
	LiftSwitch_Init();	  //升降底端限位开关初始化（PB7，输入上拉）
	BUMP_Init();		  //水泵初始化
	HC_SR04_Init();		  //超声波
	Timer_Init(); 		  //定时器
	FMQ_Init();			  //蜂鸣器初始化
	OLED_Init();
	Serial_Init();
	GPIO15_Init();
	TIM1_Init();
}

// ===== 独立看门狗（IWDG）=====
// 主循环跑飞时硬件兜底复位。LSI 40kHz / 256 = 156.25Hz，
// 重装 399 → 超时约 2.5s（LSI 30~60kHz 偏差下实际 1.7~3.4s）。
// 注意：IWDG 一旦使能只能靠复位关闭，所以调试时用 DBGMCU 把它冻结，
// 否则断点暂停期间会被反复复位。
#define WDG_RELOAD  399

void WDG_Init(void)
{
    /* 调试模式下冻结 IWDG：CPU 被调试器暂停时不计数，避免打断调试 */
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);

    /* 使能写访问 → 设置预分频和重装值 → 立即装载 → 启动 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(WDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void WDG_Feed(void)
{
    IWDG_ReloadCounter();   // 每轮主循环喂一次
}

// ===== 升降底端限位开关 + 上电找零 =====
// 限位开关 PB7 输入上拉，另一端接 GND：压下时 PB7 读到低电平。
// 上电时让升降往下走到压上开关，就能确定"现在在底端"，
// 从而重建 lift_is_up，解决断电后位置标志丢失的问题。
void LiftSwitch_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;      // 输入上拉：压下=低电平
    GPIO_InitStructure.GPIO_Pin   = LIFT_LIMIT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LIFT_LIMIT_PORT, &GPIO_InitStructure);
}

// 读取限位开关：返回 1 = 开关被压下（升降在底端）
static uint8_t LiftLimit_Pressed(void)
{
    return (GPIO_ReadInputDataBit(LIFT_LIMIT_PORT, LIFT_LIMIT_PIN) == Bit_RESET);
}

// 找零失败：停升降，OLED 显示失败原因，返回 0
// （OLED 字体只支持 ASCII，故用英文提示）
static int LiftHome_Fail(char *reason)
{
    SMotor3_SetSpeed(0);
    OLED_Clear();
    OLED_ShowString(1, 1, "FindHome Fail");
    OLED_ShowString(2, 1, reason);
    return 0;
}

// 上电找零：往下走直到压上底端限位开关，重建 lift_is_up=0。
// 全程轮询 'T' 可急停；返回 1 = 成功，0 = 被急停或超时打断。
int Lift_FindHome(void)
{
    uint32_t start;

    /* 若上电时开关已被压下（升降停在底端），先往上抬直到开关释放，
       离开死区后再往下找，确保能重新压上、准确确定底端位置 */
    if (LiftLimit_Pressed()) {
        SMotor3_SetSpeed(LIFT_UP_SPEED);
        start = timer_counter;
        while (LiftLimit_Pressed()) {
            if (Serial_GetRxFlag() && Serial_GetRxData() == 'T') {
                SMotor3_SetSpeed(0);
                return 0;
            }
            if (timer_counter - start >= 2000) {   // 2s 还没释放：开关卡死
                return LiftHome_Fail("Switch Jammed");
            }
        }
        SMotor3_SetSpeed(0);
    }

    /* 往下走，直到压上底端限位开关（或超时） */
    SMotor3_SetSpeed(LIFT_DOWN_SPEED);
    start = timer_counter;
    while (!LiftLimit_Pressed()) {
        if (Serial_GetRxFlag() && Serial_GetRxData() == 'T') {   // 随时可急停
            SMotor3_SetSpeed(0);
            return 0;
        }
        if (timer_counter - start >= LIFT_DOWN_TIME_MS + 2000) { // 超时：开关异常
            return LiftHome_Fail("No Limit Hit");
        }
    }
    SMotor3_SetSpeed(0);

    /* 碰到底端：记录位置，再往上抬直到开关释放（离开死区，避免一直压着） */
    lift_is_up = 0;
    SMotor3_SetSpeed(LIFT_UP_SPEED);
    start = timer_counter;
    while (LiftLimit_Pressed()) {
        if (Serial_GetRxFlag() && Serial_GetRxData() == 'T') {
            SMotor3_SetSpeed(0);
            return 0;
        }
        if (timer_counter - start >= 2000) {
            return LiftHome_Fail("Switch Jammed");
        }
    }
    SMotor3_SetSpeed(0);

    return 1;
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
        last_cmd_time = timer_counter;   // 记录最后收到指令的时间
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

// 开机显示复位原因（排查"为什么重启了"）
// OLED 字库只支持 ASCII，故用英文；显示后清除复位标志，避免下次误判。
void ShowResetReason(void)
{
    OLED_ShowString(2, 1, "RST:");
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
        OLED_ShowString(3, 1, "Watchdog");     // 看门狗复位
    } else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET) {
        OLED_ShowString(3, 1, "Software");     // 软件复位
    } else if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET) {
        OLED_ShowString(3, 1, "PowerOn");      // 上电复位（POR 和 PIN 同时置位，先查 POR）
    } else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) {
        OLED_ShowString(3, 1, "NRST Pin");     // 仅复位脚复位
    } else {
        OLED_ShowString(3, 1, "Unknown");
    }
    RCC_ClearFlag();                            // 清除复位标志
}

// 电池电压检测相关状态
static uint16_t bat_buf[BAT_AVG_N];   // 滑动平均环形缓冲
static uint8_t  bat_idx = 0;          // 缓冲写指针
static uint32_t bat_sum = 0;          // 缓冲累加和
static uint8_t  bat_count = 0;        // 已填样本数（<N 时按实际数平均）
static uint16_t batt_mv = 0;          // 滤波后的电池电压（mV）
static uint8_t  batt_low = 0;         // 1 = 当前处于欠压保护状态
static uint8_t  batt_low_cnt = 0;     // 越界去抖计数（触发/恢复共用）
static uint32_t bat_last_sample = 0;  // 上次采样时间戳

void ADC_Battery_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);               // ADC 时钟 = 72MHz/6 = 12MHz

    /* PA4 = 模拟输入 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin   = BAT_ADC_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BAT_ADC_PORT, &GPIO_InitStructure);

    /* 单通道、软件触发、单次转换 */
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, BAT_ADC_CHANNEL, 1, BAT_ADC_SAMPLE);
    ADC_Cmd(ADC1, ENABLE);

    /* 上电校准 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
}

/* 单次采样：启动转换并等待完成（约 20us，带 2ms 超时兜底） */
static uint16_t BAT_ReadRaw(void)
{
    uint32_t t;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    t = timer_counter;
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if (timer_counter - t >= 2) return 0;   // 2ms 超时兜底
    }
    return ADC_GetConversionValue(ADC1);
}

/* 欠压触发：停所有执行器 + 锁定 + OLED 报警（蜂鸣在状态机里周期驱动） */
static void Battery_LowAction(void)
{
    StopAll();
    run_mode = MODE_STOP;              // 锁定（等同急停，'G' 可恢复）
    OLED_Clear();
    OLED_ShowString(1, 1, "LowBattery!");
    OLED_ShowString(2, 1, "Batt:");
    OLED_ShowNum(2, 6, batt_mv / 1000, 2);
    OLED_ShowChar(2, 8, '.');
    OLED_ShowNum(2, 9, (batt_mv % 1000) / 100, 1);
    OLED_ShowChar(2, 10, 'V');
    OLED_ShowString(3, 1, "Charge or swap");
}

/* 周期调用（内部 500ms 节流）：采样 + 滤波 + 欠压保护 + 等待态显示电压 */
static void Battery_Update(void)
{
    uint16_t raw;
    uint32_t avg;

    if (timer_counter - bat_last_sample < BAT_SAMPLE_MS) return;
    bat_last_sample = timer_counter;

    /* 滑动平均：先减旧再加新 */
    raw = BAT_ReadRaw();
    if (bat_count < BAT_AVG_N) {
        bat_buf[bat_count++] = raw;
        bat_sum += raw;
    } else {
        bat_sum -= bat_buf[bat_idx];
        bat_buf[bat_idx] = raw;
        bat_sum += raw;
        bat_idx = (bat_idx + 1) % BAT_AVG_N;
    }
    avg = bat_count ? bat_sum / bat_count : raw;

    /* ADC → 电压（mV）：V = avg * 13300 / 4095 */
    batt_mv = (uint16_t)(avg * BAT_FULL_SCALE_MV / 4095);

    /* 欠压保护 + 迟滞 */
    if (!batt_low) {
        if (batt_mv < BAT_UNDER_MV) {
            if (++batt_low_cnt >= BAT_LOW_CNT) {
                batt_low_cnt = 0;
                batt_low = 1;
                Battery_LowAction();
            }
        } else {
            batt_low_cnt = 0;
        }
    } else {
        if (batt_mv > BAT_RECOVER_MV) {
            if (++batt_low_cnt >= BAT_LOW_CNT) {
                batt_low_cnt = 0;
                batt_low = 0;
            }
        } else {
            batt_low_cnt = 0;
        }
    }

    /* 正常且等待态：刷新电压显示（第 4 行，格式如 "Batt:11.8V"） */
    if (!batt_low && sys_ctrl.state == STATE_WAIT) {
        OLED_ShowString(4, 1, "Batt:");
        OLED_ShowNum(4, 6, batt_mv / 1000, 2);
        OLED_ShowChar(4, 8, '.');
        OLED_ShowNum(4, 9, (batt_mv % 1000) / 100, 1);
        OLED_ShowChar(4, 10, 'V');
    }
}

void System_StateMachine(void) {
    uint32_t current_time = GetTick();
    ProcessCommand();               // 先处理指令（含急停/恢复）

    Battery_Update();               // 电池电压采样 + 欠压保护（任何模式都跑）

    if (run_mode != MODE_AUTO) {    // 急停/欠压锁定时，不跑状态机
        if (batt_low) {             // 欠压报警：1s 周期蜂鸣
            if ((current_time / 1000) & 1) FMQ_KAI();
            else FMQ_GUAN();
        }
        return;
    }

    // 串口通信超时保护：非等待状态长时间收不到指令，自动回安全等待态
    if (sys_ctrl.state != STATE_WAIT &&
        current_time - last_cmd_time >= COMM_TIMEOUT_MS) {
        StopAll();
        sys_ctrl.state = STATE_WAIT;
        last_cmd_time = current_time;
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
