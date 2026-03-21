#ifndef _LQR_H_
#define _LQR_H_

#include "D:\BaiduNetdiskDownload\2026feika\code\workplace\cyt4-bb7_-control_-middleware_-library-master\libraries\zf_common\zf_common_typedef.h"
#include "D:\BaiduNetdiskDownload\2026feika\code\workplace\cyt4-bb7_-control_-middleware_-library-master\Simple_Template\code\System\usr_uart.hpp"
#ifdef __cplusplus
extern "C" {
#endif

#define LQR_m 6.25f              // 参与俯仰的重量
#define LQR_h 0.2156f            // 单位 m
#define LQR_I (LQR_m * LQR_h * LQR_h)  // 转动惯量
#define LQR_g 9.8f               // 重力加速度
#define LQR_r 0.07f              // 轮子半径 单位 m
#define LQR_M 3.7f               // 不参与俯仰的重量 单位 kg
#define LQR_D ((LQR_M + LQR_m) * (LQR_m * LQR_h * LQR_h + LQR_I) - LQR_m * LQR_m * LQR_h * LQR_h)

// A 矩阵 (4x4)
#define A_00 0.0f
#define A_01 1.0f
#define A_02 0.0f
#define A_03 0.0f

#define A_10 (((LQR_M + LQR_m) * LQR_m * LQR_g * LQR_h) / LQR_D)
#define A_11 0.0f
#define A_12 0.0f
#define A_13 0.0f

#define A_20 0.0f
#define A_21 0.0f
#define A_22 0.0f
#define A_23 1.0f

#define A_30 (-(LQR_m * LQR_m * LQR_g * LQR_h * LQR_h) / LQR_D)
#define A_31 0.0f
#define A_32 0.0f
#define A_33 0.0f

// B 矩阵 (4x1)
#define B_00 0.0f
#define B_10 (-(LQR_m * LQR_h) / (LQR_D * LQR_r))
#define B_20 0.0f
#define B_30 ((1.0f / ((LQR_M + LQR_m) * LQR_r)) - ((LQR_m * LQR_m * LQR_h * LQR_h) / ((LQR_M + LQR_m) * LQR_D * LQR_r)))

// Q / R 权重
#define Q_00 0.0f
#define Q_11 15.0f
#define Q_22 500.0f
#define Q_33 800.0f
#define R 10.0f

extern float K[4]; // LQR增益矩阵
extern float Wheel_position; // 电机位置
extern float Wheel_Speed; // 电机速度
extern float Wheel_acceleration; // 电机加速度

// 公开接口
void LQR_GetMatrices(float A[4][4], float B[4], float Q[4][4], float *R_out);
void LQR_ComputeK();
void process_uart2_fifo(void);
void sendTorqueToMotor(float torque);
#ifdef __cplusplus
}
#endif

#endif