#ifndef __MOTOR_H
#define __MOTOR_H


/*GPIOB pin12|pin13*/
void Motor1_Init(void);
void Motor1_SetSpeed(int8_t Speed);


/*GPIOA pin8 |pin9*/
void Motor2_Init(void);
void Motor2_SetSpeed(int8_t Speed);


/*GPIOB pin4|pin5*/
void SMotor3_Init(void);
void SMotor3_SetSpeed(int8_t Speed);
#endif
