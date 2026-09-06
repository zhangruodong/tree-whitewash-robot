#ifndef __MOTORDRIVER_H
#define __MOTORDRIVER_H
/*---------------------------------------------行走-------------------------------------------------------------------*/
void MotorDriver1_Init(void);										//电机驱动模块1初始化（左）
void MotorDriver2_Init(void);										//电机驱动模块2初始化右）
void MoveForward(void);      										//全速前进
void MoveBackward(void);     										//全速后退
void MotorDriverFullStop(void);       											//刹车停止
void SmartSteering(int8_t base_speed, int8_t steer);				//差速控制
void SpinLeft(void);        										// 原地左转（左轮后退，右轮前进）
void SpinRight(void);       										// 原地右转（右轮后退，左轮前进）
void TurnLeftWhileMoving(int8_t base_speed);        	 			// 行进中左转（保持基准速度，添加左转差速）
void TurnRightWhileMoving(int8_t base_speed);      	 				// 行进中右转（保持基准速度，添加右转差速）
/*-------------------------------------------升降--------------------------------------------------------------------*/


/*-------------------------------------------------超声波控制速度----------------------------------------------------*/
void EnhancedUltrasonicControl(void);
#endif
