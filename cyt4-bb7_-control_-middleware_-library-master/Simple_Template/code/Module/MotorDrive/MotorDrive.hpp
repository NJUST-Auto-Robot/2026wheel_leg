/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: MotorDrive.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
#include "zf_common_headfile.h"

// 电压设置
#define BATTERY_MAX_VOL (11)               // 福特
#define MOTOR_MAX_VOL (10)                 // 福特
#define MOTOR_MID_VOL (MOTOR_MAX_VOL >> 1) // 取一半为中间值
#define MOTOR_MIN_VOL (0)                  // 福特

// 引脚设置
#define MOTOR_PWM_PIN (TCPWM_CH19_P08_0)
#define MOTOR_DIR_PIN (P08_1)

// 使用L298N的话
#define L298N_USED 0
#if L298N_USED
#define L298N_EXR_DIR_PIN (P10_0)
#endif

typedef enum {
  backward_dir = 0,
  forward_dir = 1,
  stop_dir = 2,
} motor_dir_t;

class MotorBase {
public:
  MotorBase(pwm_channel_enum _pwm_pin, gpio_pin_enum _dir_pin)
      : pwm_pin(_pwm_pin), dir_pin(_dir_pin){};
  ~MotorBase(){};
  void gpioInit(void);

  void gpioSet(void);

  void setDirVol(motor_dir_t _dir, float _vol);

  void setNormDirVol(float drv_arg);

  uint32_t getPwmDuty(void) { return pwm_duty; }

protected:
  pwm_channel_enum pwm_pin;
  gpio_pin_enum dir_pin;
  uint8_t dir;
  uint32_t pwm_duty;
  float norm_arg;
};

extern MotorBase motor;

#ifdef __cplusplus
extern "C" {
#endif
void motorTask(void *argument);

#ifdef __cplusplus
}
#endif