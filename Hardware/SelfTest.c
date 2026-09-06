#include "stm32f10x.h"
#include "SelfTest.h"
#include "DPWM.h"
#include "Servo.h"
#include "Delay.h"
#include "MotorDriver.h"
#include "Motor.h"
#include "USART.h"
#include <stdlib.h>
#include <math.h>

/* hardware self-test routine.
   The waits below poll USART2 for 'T' (emergency stop), so a Bluetooth
   stop command can abort the self-test at any point instead of blocking
   the main loop for the full ~9 s with no way to stop. */

#define MOTOR_TEST_SPEED     60    /* drive motor test speed (0-100) */
#define MOTOR_TEST_DURATION  800   /* drive motor test time (ms) */
#define LIFT_TEST_SPEED      60    /* lift motor test speed (%) */
#define LIFT_TEST_DURATION   1200  /* lift motor test time (ms) */

extern volatile uint32_t timer_counter;   /* 1 ms tick from system.c */

ErrorCode systemError = ERR_NONE;

/* Wait ms milliseconds, polling for 'T'. Returns 1 if the wait finished
   normally, 0 if emergency stop was requested (motors/servos stopped). */
static uint8_t SelfTest_Wait(uint32_t ms)
{
    uint32_t start = timer_counter;
    while (timer_counter - start < ms) {
        if (Serial_GetRxFlag() && Serial_GetRxData() == 'T') {
            MotorDriverFullStop();
            SMotor3_SetSpeed(0);
            Servo_StopAllMotions();
            return 0;
        }
    }
    return 1;
}

int Self_Check_Routine(void)
{
    /* stage 1: servo closed position */
    Servo_SetAngle(0);
    Servo2_SetAngle(0);
    if (!SelfTest_Wait(800)) return 0;

    /* stage 2: servo middle position */
    Servo_SetAngle(90);
    Servo2_SetAngle(90);
    if (!SelfTest_Wait(500)) return 0;

    /* stage 3: servo open position */
    Servo_SetAngle(160);
    Servo2_SetAngle(160);
    if (!SelfTest_Wait(800)) return 0;

    /* drive motor test: left */
    Motor1_SetSpeed(MOTOR_TEST_SPEED);
    if (!SelfTest_Wait(MOTOR_TEST_DURATION)) return 0;
    Motor1_SetSpeed(-MOTOR_TEST_SPEED);
    if (!SelfTest_Wait(MOTOR_TEST_DURATION)) return 0;
    Motor1_SetSpeed(0);

    /* drive motor test: right */
    Motor2_SetSpeed(MOTOR_TEST_SPEED);
    if (!SelfTest_Wait(MOTOR_TEST_DURATION)) return 0;
    Motor2_SetSpeed(-MOTOR_TEST_SPEED);
    if (!SelfTest_Wait(MOTOR_TEST_DURATION)) return 0;
    Motor2_SetSpeed(0);

    /* lift motor test: up */
    SMotor3_SetSpeed(LIFT_TEST_SPEED);
    if (!SelfTest_Wait(LIFT_TEST_DURATION)) return 0;

    SMotor3_SetSpeed(0);
    if (!SelfTest_Wait(500)) return 0;

    /* lift motor test: down */
    SMotor3_SetSpeed(-LIFT_TEST_SPEED);
    if (!SelfTest_Wait(LIFT_TEST_DURATION)) return 0;

    SMotor3_SetSpeed(0);

    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    if (!SelfTest_Wait(3000)) return 0;
    GPIO_SetBits(GPIOB, GPIO_Pin_14);

    Servo_SetAngle(180);

    return 1;
}
