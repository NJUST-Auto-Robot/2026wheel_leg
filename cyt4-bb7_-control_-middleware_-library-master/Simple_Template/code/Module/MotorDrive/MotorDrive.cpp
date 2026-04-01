/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: MotorDrive.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Module/MotorDrive/MotorDrive.hpp"
#include "AAARobot/AAARobot.hpp"
// 实例化对象
MotorBase motor(MOTOR_PWM_PIN, MOTOR_DIR_PIN);

/**
 * @brief 电机的线程任务
 *
 * @param argument
 */
void motorTask(void *argument) {
  /* USER CODE BEGIN */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  xLastWakeTime = xTaskGetTickCount();

  /* 客制化准备 */

  // 初始化GPIO的设置
  motor.gpioInit();

  /* Infinite loop */
  for (;;) {

    // 通过方向和电压给电机赋值
    // motor.setDirVol(stop_dir, tst_arg);

    // 通过归一化给电机赋值
    motor.setNormDirVol(car.motor_norm_duty); // 没有断链保护
    // 有断链保护
    // if (car.pFSI6X_data->GetStatus() == ESTABLISHED)
    //   motor.setNormDirVol(car.motor_norm_duty);
    // else
    //   motor.setNormDirVol(0);

    // gpio下达
    motor.gpioSet();

    // 任务调度堵塞
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END */
}

/**
 * @brief 初始化GPIO的设置
 *
 */
void MotorBase::gpioInit(void) {

  // 初始化 PWM 通道 频率 100KHz 初始占空比 0% 最大占空比是10000
  pwm_init(this->pwm_pin, 10000, 0);

  // 初始化方向GPO引脚
  gpio_init(this->dir_pin, GPO, 0, GPO_PUSH_PULL);

#if L298N_USED
  // 初始化给L298N用的额外GPO
  gpio_init()
#endif
}

/**
 * @brief 下达GPIO
 *
 */
void MotorBase::gpioSet(void) {
  gpio_set_level(this->dir_pin, this->dir);
  pwm_set_duty(this->pwm_pin, this->pwm_duty);
}

/**
 * @brief 设置方向和电压
 *
 * @param _dir 方向变量
 * @param _vol 实际要给电机输出的电压
 */
void MotorBase::setDirVol(motor_dir_t _dir, float _vol) {

  // 设置GPIO的dir_pin的电平【临时表达式】
  auto setGPIODrvDir = [this](motor_dir_t _dir) { this->dir = (uint8_t)_dir; };

  // 设置GPIO的pwm_pin的pwm【临时表达式】
  auto setGPIODrvVol = [this](float _set_vol_) {
    _set_vol_ = func_limit_ab(_set_vol_, MOTOR_MIN_VOL, MOTOR_MAX_VOL);

    // 用一个临时归一化量来计算要下达的duty值
    float tem_norm_val = 0.0f;

    if (this->dir != 2) {
      // 【下达电压值】对于【电池限制电压】的比率 * 最大duty值
      tem_norm_val = _set_vol_ / BATTERY_MAX_VOL * PWM_DUTY_MAX;
    }

    // 若drv_dir为2，即停止，直接在这里设置drv为0
    this->pwm_duty = (uint32_t)tem_norm_val;
  };

  // 设置GPIO的dir_pin的电平
  setGPIODrvDir(_dir);

  // 设置GPIO的pwm_pin的pwm
  setGPIODrvVol(_vol);
}

/**
 * @brief 用归一化参数设置方向和电压
 *
 * @param drv_arg
 */
void MotorBase::setNormDirVol(float drv_arg) {
  drv_arg = func_limit_ab(drv_arg, -1, 1); // 先限幅

  if (drv_arg == 0) {
    this->dir = (uint8_t)stop_dir;
    this->pwm_duty = 0;
    return; // 直接结束即可
  }

  // 决定方向
  this->dir = drv_arg > 0 ? (uint8_t)forward_dir : (uint8_t)backward_dir;

  // 取绝对值
  drv_arg = func_abs(drv_arg);

  float set_vol = MOTOR_MAX_VOL * drv_arg;

  // 设置GPIO的pwm_pin的pwm【临时表达式】
  auto setGPIODrvVol = [this](float _set_vol_) {
    _set_vol_ = func_limit_ab(_set_vol_, MOTOR_MIN_VOL, MOTOR_MAX_VOL);
    float tem_norm_val = 0.0f; // 用一个临时归一化量来计算要下达的duty值

    if (this->dir != 2) {
      // 【下达电压值】对于【电池限制电压】的比率 * 最大duty值
      tem_norm_val = _set_vol_ / BATTERY_MAX_VOL * PWM_DUTY_MAX;
    }

    // 若drv_dir为2，即停止，直接在这里设置drv为0
    this->pwm_duty = (uint32_t)tem_norm_val;
  };

  // 设置GPIO的pwm_pin的pwm
  setGPIODrvVol(set_vol);
}