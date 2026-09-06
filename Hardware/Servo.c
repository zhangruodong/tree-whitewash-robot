#include "stm32f10x.h"                  // Device header
#include "DPWM.h"
#include "Servo.h"

extern volatile uint32_t timer_counter;  // 1ms 定时计数（定义在 system.c）

// 舵机运动实例
static ServoMotion servo_instances[MAX_SERVOS] = {0};

// 底层：舵机1初始化
void Servo_Init(void)
{
	PWM_TIM2_Common_Init();
	DPWM_Init();
}

// 底层：舵机1直接设角度（0~180° 线性映射到 0.5~2.5ms）
void Servo_SetAngle(float Angle)
{
	DPWM_SetCompare3(Angle / 180 * 2000 + 500);
}

// 底层：舵机2初始化
void Servo2_Init(void)
{
	PWM_TIM2_Common_Init();
	DPWM2_Init();
}

// 底层：舵机2直接设角度
void Servo2_SetAngle(float Angle)
{
	DPWM2_SetCompare4(Angle / 180 * 2000 + 500);
}

// 平滑运动：初始化所有实例
void ServoMotion_Init(void)
{
	for (int i = 0; i < MAX_SERVOS; i++) {
		servo_instances[i].is_active = 0;
	}
}

// 平滑运动：启动一次运动（非阻塞，只登记并预计算系数）
void Servo_StartSmoothMotion(uint8_t servo_id, float start_angle,
                             float end_angle, uint32_t duration_ms)
{
	if (servo_id >= MAX_SERVOS || duration_ms == 0) return;

	float T = (float)duration_ms;
	float dy = end_angle - start_angle;

	servo_instances[servo_id].is_active = 1;
	servo_instances[servo_id].start_angle = start_angle;
	servo_instances[servo_id].end_angle = end_angle;
	servo_instances[servo_id].duration_ms = duration_ms;
	servo_instances[servo_id].start_time = timer_counter;

	// 预计算三次 Hermite（smoothstep）系数：两端速度为零
	servo_instances[servo_id].coeff.a = start_angle;
	servo_instances[servo_id].coeff.b = 0.0f;
	servo_instances[servo_id].coeff.c = 3.0f * dy / (T * T);
	servo_instances[servo_id].coeff.d = -2.0f * dy / (T * T * T);
}

// 平滑运动：每轮主循环调用，推进所有进行中的运动（非阻塞）
void Servo_UpdateAllMotions(void)
{
	for (uint8_t i = 0; i < MAX_SERVOS; i++) {
		if (!servo_instances[i].is_active) continue;

		uint32_t elapsed = timer_counter - servo_instances[i].start_time;
		float angle;

		if (elapsed >= servo_instances[i].duration_ms) {
			// 运动结束：钉住最终角度并停用
			angle = servo_instances[i].end_angle;
			servo_instances[i].is_active = 0;
		} else {
			SplineCoeff c = servo_instances[i].coeff;
			float t = (float)elapsed;
			angle = c.a + c.b * t + c.c * t * t + c.d * t * t * t;
		}

		if (i == 0) Servo_SetAngle(angle);
		else        Servo2_SetAngle(angle);
	}
}

// 取消所有进行中的运动（急停时调用）
void Servo_StopAllMotions(void)
{
	for (uint8_t i = 0; i < MAX_SERVOS; i++) {
		servo_instances[i].is_active = 0;
	}
}

// 查询某舵机是否还在运动
uint8_t Servo_IsMoving(uint8_t servo_id)
{
	if (servo_id >= MAX_SERVOS) return 0;
	return servo_instances[servo_id].is_active;
}
