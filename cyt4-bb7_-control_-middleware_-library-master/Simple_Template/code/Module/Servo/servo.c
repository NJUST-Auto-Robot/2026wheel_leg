/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: servo.c
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Servo\servo.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Module/Servo/servo.h"

// servo库专用的全局变量->归一化角度值
float g_servo_norm_ang = 0;

// 测试写入移动硬盘数据

// 舵机测试模式
#define SERVO_TST_MODE (0)
#if SERVO_TST_MODE
uint8_t tst_ang = 90;
#endif

/**
 * @brief 舵机转向任务
 *
 * @param argument
 */
void servoTask(void *argument) {
  /* USER CODE BEGIN */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20;
  xLastWakeTime = xTaskGetTickCount();

  /* 客制化准备 */
  initServo(SERVO_PWM_CHANNEL, 90);

  /* Infinite loop */
  for (;;) {
#if SERVO_TST_MODE
    // 舵机测试角度
    setServoTrueAng(SERVO_PWM_CHANNEL, tst_ang);
#else
    // 设置舵机的归一化角度
    setServoNormAng(SERVO_PWM_CHANNEL, g_servo_norm_ang, true);
#endif
    // 任务调度堵塞
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END */
}

/**
 * @brief 归一化角度还原成真实角度
 *
 * @param _norm_angle_ 归一化后的角度值
 * @return uint32_t
 */
static uint32_t antiNormalize2TrueAng(float _norm_angle_) {
  uint32_t result = 0;

  if (_norm_angle_ < 0) {
    result =
        (uint32_t)(_norm_angle_ * (MIDDLE_ANG - RIGHT_MAX_ANG) + MIDDLE_ANG);
  } else {
    result =
        (uint32_t)(_norm_angle_ * (LEFT_MAX_ANG - MIDDLE_ANG) + MIDDLE_ANG);
  }
  return result;
}
/**
 * @brief 将真实角度转化成pwm比较值
 *
 */
static uint32_t servoTrueAng2Duty(float set_angle) {
  float res_duty = 0;

  if (set_angle >= 180)
    set_angle = 180;
  else if (set_angle <= 0)
    set_angle = 0;

  res_duty = 250 + set_angle * (1000 / 180.f);

  return (uint32_t)res_duty;
}

/**
 * @brief 设置舵机的真实角度
 *
 * @param pwmch pwm通道
 * @param set_angle 设置的真实角度
 *  setServoTrueAng(TCPWM_CH28_P10_0,90);
 */
void setServoTrueAng(pwm_channel_enum pwmch, float set_angle) {
  uint32_t set_duty = servoTrueAng2Duty(set_angle);
  pwm_set_duty(pwmch, set_duty);
}

/**
 * @brief 通过归一化角度设置舵机转向
 *
 * @param pwmch 舵机pwm通道
 * @param set_angle 要设置的归一化角度值
 * @param is_oppositive 归一化角度是否反向
 * setServoNormAng(TCPWM_CH28_P10_0, 0, false);
 */
void setServoNormAng(pwm_channel_enum pwmch, float set_angle,
                     bool is_oppositive) {
  if (is_oppositive) {
    set_angle = -set_angle;
  }
  uint32_t set_duty = servoTrueAng2Duty(antiNormalize2TrueAng(set_angle));
  pwm_set_duty(pwmch, set_duty);
}

/**
 * @brief 初始化舵机
 *
 * @param pwmch pwm通道
 * @param init_angle 初始化时需要的真实角度
 * initServo(TCPWM_CH28_P10_0,90);
 */
void initServo(pwm_channel_enum pwmch, uint32_t init_angle) {
  pwm_init(pwmch, 50,
           servoTrueAng2Duty(
               init_angle)); // 舵机需要发出20ms为周期的pwm，对应50hz频率
}