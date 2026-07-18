/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: pid.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>

#include "zf_common_function.h"
#include "zf_driver_timer.h"

#ifndef __FLT_MAX__
#define __FLT_MAX__ FLT_MAX
#endif

#ifndef __FLT_MIN__
#define __FLT_MIN__ FLT_MIN
#endif

#include <math.h>
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
class OpenLoop_t {
private:
  /* data */
  float Gain;
  float Target;
  float Current;
  float Out;

public:
  OpenLoop_t(/* args */) {}
  ~OpenLoop_t() {}
};

class PID_t {
private:
  /* data */
  float dt;
  uint32_t last_time;
  float Target, Current, Error;
  float Out;

  float Kp, Ki, Kd;

  /*<! I项限幅 */
  float I_Term_Max;

  /*<! 输出限幅 */
  float Out_Max;

  float I_Term; /* 积分器输出 */
  float P_Term; /* 比例器输出 */
  float D_Term; /* 微分器输出 */

  /*!<积分分离阈值，需为正数。std::abs(error)大于该阈值取消积分作用。*/
  float I_SeparateThresh;

  float VarSpeed_I_A; /*!< 变速积分 A，需为正数。*/

  /*!< 变速积分 B，需为正数， \n
    在 error<=B 的区间内，为普通积分效果， \n
    在 B<error<=A+B 的区间内，为变速积分效果， \n
    在 A+B<error 的区间内，不继续积分。*/
  float VarSpeed_I_B;

  /*!< 死区，需为整数，std::abs(error)小于DeadZone时，输出为0。 */
  float DeadZone;

  /*!< 启用微分先行，文献中Current多译作Process Variable(PV)。 */
  bool D_of_Current;
  float integral_e;
  float pre_error;
  float pre_current;

  // 定义获取系统时钟的函数指针
  uint32_t (*Get_SystemTick)(void);

  // 误差滤波器
  float (*ErrorFilter)(float data);

  // 差分滤波器
  float (*DiffFilter)(float data);

public:
  PID_t() { this->Get_SystemTick = GetSystemTimer; }
  PID_t(float _kp, float _ki, float _kd, float _I_Term_Max, float _Out_Max)
      : Kp(_kp), Ki(_ki), Kd(_kd), I_Term_Max(_I_Term_Max), Out_Max(_Out_Max) {
    this->Get_SystemTick = GetSystemTimer;
  }
  ~PID_t() {}

  // 更新时间戳函数
  uint8_t UpdateTimeStamp();

  // 设置pid参数
  void SetPIDParam(float _Kp, float _Ki, float _Kd, float _I_Term_Max,
                   float _Out_Max);
  // 加载当前值
  void currentUpdate(float get) { Current = get; }

  // 加载目标值
  void targetUpdate(float set) { Target = set; }

  // pid计算函数
  float Adjust();

  // PID清除输出
  void clean_out();

  // PID清除积分项函数
  void clean_integral();

  // 设置误差滤波器
  void setErrorFilter(float (*_set_error_filter)(float));

  // 设置误差滤波器
  void setDiffFilter(float (*_set_diff_filter)(float));

  float getOut() { return Out; }
  float getTarget() { return Target; }
  float getCurrent() { return Current; }
  float getError() { return Error; }
  void setOutMax(float _Out_Max) { Out_Max = _Out_Max; }
  void setDeadZone(float _DeadZone) { DeadZone = _DeadZone; }
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/