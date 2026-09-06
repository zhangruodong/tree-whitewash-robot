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
// 全局变量用于记录定时器中断次数
volatile uint32_t timer_counter = 0;
static SystemCtrl sys_ctrl = {STATE_WAIT, 0, 0};

///* 指令到状态映射 */
//static const struct {
//    uint8_t cmd;
//    SystemState state;
//} cmd_state_map[] = {
//    {CMD_NO_TREE,    STATE_SEARCH_TREE},
//    {CMD_UNTREATED,  STATE_APPROACH_TREE},
//    {CMD_STOP,       STATE_WAIT_CMD}
//};

// TIM1初始化函数
void TIM1_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 使能TIM1时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 2. 配置定时器参数 */
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;     // 预分频值（72MHz / 7200 = 10KHz）
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;       // 自动重装载值（10KHz / 10000 = 1秒中断一次）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    // 高级定时器特有参数
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

// GPIO初始化（配置PB15为输出）
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
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除中断标志
        timer_counter++;
        GPIO_WriteBit(GPIOB, GPIO_Pin_15, 
                     (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15))); // 翻转PB15
    }
}

void Hardware_Init(void)  //外设初始化
{
	Servo_Init();		  //舵机初始化
	MotorDriver1_Init();										//电机驱动模块1初始化（左）
	MotorDriver2_Init();										//电机驱动模块2初始化右）
	SMotor3_Init();		  //直流电机初始化（直流电机3，升降杆）
	BUMP_Init();		  //水泵初始化
	HC_SR04_Init();		  //超声波
	Timer_Init(); 		  //定时器
	FMQ_Init();			  //蜂鸣器初始化
	ADC_GPIO_Init();		//adc初始化
	ADC_Init_Channel4();	//adc
	OLED_Init();
	FMQ_Init();
	Serial_Init();
	GPIO15_Init();
	TIM1_Init();
}



static void ProcessCommand(void) {
    if(Serial_GetRxFlag()) {
        uint8_t cmd = Serial_GetRxData();
        sys_ctrl.last_cmd = cmd;  // 存储最新指令
        
        switch(cmd) {
			
	case 'W'://等待指令  串口'W'
        sys_ctrl.state = STATE_WAIT;
        break;
    case 'N'://没有找到树 串口发送  ‘N’		跳转到原地搜索状态  也可以串口跳  ‘s’
        sys_ctrl.state = STATE_NO_TREE;
        break;
	case 'S':		//原地搜寻状态'S'
        sys_ctrl.state = STATE_SEARCH_TREE;		
        break;
    case 'U':		//接近树木	'U'
        sys_ctrl.state = STATE_APPROACH_TREE;
        break;
    case 'P':		 // 喷涂准备'P' 
        sys_ctrl.state = STATE_PAINT_PREP;
        break;
    case 'R':			//后撤	'R'
        sys_ctrl.state = STATE_RETREAT;
        break;
    case'B':	// 蜂鸣提示	'B'
        sys_ctrl.state = STATE_RETREAT_BEEP;
        break;
	case 'T':
        sys_ctrl.state = STATE_TING;
		break;

        
        }
    }
}
uint32_t GetTick(void) {
    return timer_counter;  // 提供具体实现
}
void System_StateMachine(void) {
   // static uint8_t lift_phase = 0;
    uint32_t current_time = GetTick();
    ProcessCommand();

    switch(sys_ctrl.state) {
    case STATE_WAIT:
        if(sys_ctrl.last_cmd == STATE_NO_TREE) {
            OLED_Clear();
            OLED_ShowString(2, 1, "NO_TREE");
            sys_ctrl.state = STATE_NO_TREE;//等待状态，如果接到没有找到树的指令就跳转到没有找到树的状态，找到了
            sys_ctrl.state_timestamp = current_time;
        }//else 
        break;
	case STATE_NO_TREE://没有找到树的状态，跳转到原地搜索树的状态
		 sys_ctrl.state = STATE_SEARCH_TREE;
         sys_ctrl.state_timestamp = current_time;
		break;
    case STATE_SEARCH_TREE:
        // 保持旋转搜索
        Motor1_SetSpeed(90);
        Motor2_SetSpeed(-90);
        
        //原地搜索状态，如果超过时间，就回到等待指令状态，如果接到靠近树的状态，就跳转到接近树木状态
        if((current_time - sys_ctrl.state_timestamp >= 30) ){
          // (sys_ctrl.last_cmd == STATE_APPROACH_TREE)) 
			OLED_Clear();
            OLED_ShowString(2, 1, "chaoshi");
            MotorDriverFullStop();
            sys_ctrl.state = STATE_WAIT;
            sys_ctrl.state_timestamp = current_time;
        }else if(sys_ctrl.last_cmd == STATE_APPROACH_TREE) {
			 sys_ctrl.state = STATE_APPROACH_TREE;
             sys_ctrl.state_timestamp = current_time;}
        break;

    case STATE_APPROACH_TREE:
        EnhancedUltrasonicControl();
        // 靠近树木状态，如果距离够了就开始涂白准备，如果超时了就调整转到等待指令状态
        if(range()==5){sys_ctrl.state = STATE_PAINT_PREP;
            sys_ctrl.state_timestamp = current_time;}
		else if(current_time - sys_ctrl.state_timestamp >= 90) {
            sys_ctrl.state = STATE_WAIT;
            sys_ctrl.state_timestamp = current_time;
        }
        break;

    case STATE_PAINT_PREP: {		//开始涂白状态，结束后跳转到后撤状态
		
        static bool first_enter = true;
        if(first_enter) {
            //Servo_SetAngle(0);
            BUMP_KAI();
            first_enter = false;
        }

        /* 升降控制 */
//        switch(lift_phase) {
//        case 0:  // 上升
            SMotor3_SetSpeed(80);
            if(current_time - sys_ctrl.state_timestamp > 17) {
//                lift_phase = 1;
                sys_ctrl.state_timestamp = current_time;
//            }
//            break;
//        case 1:  // 下降
            SMotor3_SetSpeed(-80);
            if(current_time - sys_ctrl.state_timestamp >13 ) {
                SMotor3_SetSpeed(0);
				BUMP_GUAN();
                sys_ctrl.state = STATE_RETREAT;
                first_enter = true;
            }
//            break;
        }
        } 
	break;

    case STATE_RETREAT://后撤状态，跳转到蜂鸣器状态
		SMotor3_SetSpeed(0);
        Motor1_SetSpeed(-80);
        Motor2_SetSpeed(-80);
        if(current_time - sys_ctrl.state_timestamp > 5) {
            MotorDriverFullStop();
            sys_ctrl.state = STATE_RETREAT_BEEP;
            sys_ctrl.state_timestamp = current_time;
        }
        break;

		case STATE_RETREAT_BEEP://蜂鸣器，结束后到等待状态
        FMQ_KAI();
        if(current_time - sys_ctrl.state_timestamp > 2) {
            FMQ_GUAN();
            sys_ctrl.state = STATE_WAIT;
        }
        break;
		
		
		case STATE_TING:
			 /* 错误处理 */
		TIM_Cmd(TIM2, DISABLE);        // 关闭PWM输出
		GPIO_ResetBits( GPIOA , GPIO_Pin_1); // 强制拉低引脚
		MotorDriverFullStop();                 // 电机急停
		SMotor3_SetSpeed(0);   // 停止升降
        break;
    }

}
