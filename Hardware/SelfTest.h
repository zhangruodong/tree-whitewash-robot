#ifndef __SELFTEST_H
#define __SELFTEST_H
typedef enum {
    ERR_NONE = 0,
    ERR_SERVO_RANGE,     // 舵机角度范围异常
    ERR_SERVO_FEEDBACK   // 反馈信号异常
} ErrorCode;

extern ErrorCode systemError; 
int Self_Check_Routine(void);
#endif
