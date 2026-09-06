#include "stm32f10x.h"
#include "Motor.h"
#include "ZPWM.h"
#include "Delay.h"
#include "HCSR04.h"

/* motion control parameters */
#define MAX_SPEED      100    /* max speed (100% PWM) */
#define STEER_GAIN     0.6f   /* steering diff gain (0.1-1.0) */
#define SPIN_SPEED     80     /* in-place spin speed (60-90) */
#define TURN_STRENGTH  50     /* turning strength while moving (30-70) */

#define SAFE_DIST_FAR      40 /* full forward above this (cm) */
#define SAFE_DIST_NEAR     20 /* start decel below this (cm) */
#define SAFE_DIST_STOP     5  /* hard stop below this (cm) */
#define BASE_SPEED_FORWARD 80 /* forward base speed */
#define BASE_SPEED_BACKWARD 60/* backward base speed */

void MotorDriver1_Init(void) {
    ZPWM1_Init();        /* DC motor 1 PWM */
    Motor1_Init();       /* DC motor 1 */
}

void MotorDriver2_Init(void) {
    ZPWM2_Init();        /* DC motor 2 PWM */
    Motor2_Init();       /* DC motor 2 */
}

void MoveForward(void) {
    Motor1_SetSpeed(100);
    Motor2_SetSpeed(100);
}

void MoveBackward(void) {
    Motor1_SetSpeed(-100);
    Motor2_SetSpeed(-100);
}

void MotorDriverFullStop(void) {
    Motor1_SetSpeed(0);
    Motor2_SetSpeed(0);
}

/* differential steering: V_left = base - delta, V_right = base + delta */
void SmartSteering(int8_t base_speed, int8_t steer) {
    base_speed = (base_speed < -MAX_SPEED) ? -MAX_SPEED :
                ((base_speed > MAX_SPEED) ? MAX_SPEED : base_speed);
    steer = (steer < -100) ? -100 : ((steer > 100) ? 100 : steer);

    float delta = steer * STEER_GAIN;
    int8_t left_speed = base_speed - delta;
    int8_t right_speed = base_speed + delta;

    left_speed = (left_speed < -MAX_SPEED) ? -MAX_SPEED :
                ((left_speed > MAX_SPEED) ? MAX_SPEED : left_speed);
    right_speed = (right_speed < -MAX_SPEED) ? -MAX_SPEED :
                 ((right_speed > MAX_SPEED) ? MAX_SPEED : right_speed);

    Motor1_SetSpeed(left_speed);
    Motor2_SetSpeed(right_speed);
}

void SpinLeft(void) {
    Motor1_SetSpeed(-SPIN_SPEED);
    Motor2_SetSpeed(SPIN_SPEED);
}

void SpinRight(void) {
    Motor1_SetSpeed(SPIN_SPEED);
    Motor2_SetSpeed(-SPIN_SPEED);
}

void TurnLeftWhileMoving(int8_t base_speed) {
    SmartSteering(base_speed, -TURN_STRENGTH);
}

void TurnRightWhileMoving(int8_t base_speed) {
    SmartSteering(base_speed, TURN_STRENGTH);
}

/* Ultrasonic distance-based approach control.
   Uses the last averaged non-blocking distance from US_GetDistance().
   Returns 1 when the stop distance is reached (dist <= SAFE_DIST_STOP),
   otherwise 0. */
int EnhancedUltrasonicControl(void) {
    uint16_t dist = US_GetDistance();   /* last averaged distance (cm) */

    if (dist <= SAFE_DIST_STOP) {
        MotorDriverFullStop();          /* too close: hard stop */
        return 1;
    }
    else if (dist < SAFE_DIST_FAR) {
        /* decelerate approaching: speed ramps 0 -> 80 as dist goes 20 -> 40 */
        int8_t speed = BASE_SPEED_FORWARD * (dist - SAFE_DIST_NEAR) /
                      (SAFE_DIST_FAR - SAFE_DIST_NEAR);
        SmartSteering(speed, 0);
    }
    else {
        MoveForward();                  /* far: full forward */
    }
    return 0;
}
