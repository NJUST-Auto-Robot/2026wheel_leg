/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: servo.h
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Servo\servo.h
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
#include "CPP/tst.h"
#include "zf_driver_pwm.h" // pwm_channel_enum从这个文件中查询

#define SERVO_PWM_CHANNEL (TCPWM_CH03_P06_7)
#if 1 // 非自购车模
#define LEFT_MAX_ANG (130)
#define MIDDLE_ANG (100)
#define RIGHT_MAX_ANG (70)
// #define LEFT_MAX_ANG (120)
// #define MIDDLE_ANG (100)                                           
// #define RIGHT_MAX_ANG (80)
#else // 自购车模
#define LEFT_MAX_ANG (115)
#define RIGHT_MAX_ANG (55)
#define MIDDLE_ANG (85)
#endif

extern float g_servo_norm_ang;

#ifdef __cplusplus
extern "C" {
#endif
void setServoNormAng(pwm_channel_enum pwmch, float set_angle,
                     bool is_oppositive);
void setServoTrueAng(pwm_channel_enum pwmch, float set_angle);
void initServo(pwm_channel_enum pwmch, uint32_t init_angle);
void servoTask(void *argument);
#ifdef __cplusplus
}
#endif
