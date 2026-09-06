#ifndef __MOTORDRIVER_H
#define __MOTORDRIVER_H

void MotorDriver1_Init(void);
void MotorDriver2_Init(void);
void MoveForward(void);
void MoveBackward(void);
void MotorDriverFullStop(void);
void SmartSteering(int8_t base_speed, int8_t steer);
void SpinLeft(void);
void SpinRight(void);
void TurnLeftWhileMoving(int8_t base_speed);
void TurnRightWhileMoving(int8_t base_speed);

int EnhancedUltrasonicControl(void);   /* returns 1 when stop distance reached */

#endif
