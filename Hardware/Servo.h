#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

// 支持的最大舵机数量
#define MAX_SERVOS 2

// 三次多项式系数（缓起缓停曲线 a + b*t + c*t^2 + d*t^3）
typedef struct {
    float a, b, c, d;
} SplineCoeff;

// 舵机平滑运动状态
typedef struct {
    uint8_t is_active;      // 运动是否进行中
    float start_angle;      // 起始角度(°)
    float end_angle;        // 目标角度(°)
    uint32_t duration_ms;   // 运动时长(ms)
    uint32_t start_time;    // 开始时刻(ms)
    SplineCoeff coeff;      // 预计算的插值系数（启动时算一次）
} ServoMotion;

// 底层：直接设置角度（0~180° 线性映射到 0.5~2.5ms 脉宽）
void Servo_Init(void);
void Servo_SetAngle(float Angle);
void Servo2_Init(void);
void Servo2_SetAngle(float Angle);

// 平滑运动接口（非阻塞）
void ServoMotion_Init(void);
void Servo_StartSmoothMotion(uint8_t servo_id, float start_angle,
                             float end_angle, uint32_t duration_ms);
void Servo_UpdateAllMotions(void);   // 每轮主循环调用，非阻塞推进
void Servo_StopAllMotions(void);     // 取消所有进行中的运动（急停时调用）
uint8_t Servo_IsMoving(uint8_t servo_id);

#endif
