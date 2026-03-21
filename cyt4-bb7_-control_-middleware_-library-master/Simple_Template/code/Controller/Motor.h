#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "D:\BaiduNetdiskDownload\2026feika\code\workplace\cyt4-bb7_-control_-middleware_-library-master\libraries\zf_common\zf_common_typedef.h"
#include "D:\BaiduNetdiskDownload\2026feika\code\workplace\cyt4-bb7_-control_-middleware_-library-master\Simple_Template\code\System\usr_uart.hpp"
#include "LQR.h"
#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_RADIUS 0.08f // 轮子半径 单位 m

// 速度 PID 默认参数
#define MOTOR_PID_KP 10.0f
#define MOTOR_PID_KI 1.0f
#define MOTOR_PID_KD 0.5f
#define MOTOR_PID_I_TERM_MAX 5.0f
#define MOTOR_PID_OUT_MAX 50.0f

// 速度 PID 相关（线速度闭环 -> 扭矩）
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float I_term;
  float prev_error;
  float I_term_Max;
  float Out_Max;
  float DeadZone;
} MotorPID_t;

extern MotorPID_t wheel_speed_pid;

void MotorPID_Init(float kp, float ki, float kd, float i_term_max, float out_max);
float MotorSpeedClosedLoopTorque(float target_linear_speed, float measured_wheel_speed);
float MotorComputeTotalTorque(float target_linear_speed, float measured_wheel_speed, float u_balance);

#ifdef __cplusplus
}
#endif

#endif