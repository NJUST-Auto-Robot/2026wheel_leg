/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: pid.cpp
 * @Description: 
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.   
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved. 
 */
#include "Algorithm/PID/pid.hpp"

const float numeric_limits_float_epsilon = 1.19209e-07;

/**
 * @brief 更新时间戳函数
 *
 * @return uint8_t
 */
uint8_t PID_t::UpdateTimeStamp() {

  uint32_t now_time;

  /*Check `Get_SystemTick` */
  if (this->Get_SystemTick != NULL) {
    /*Convert to system time*/
    if (this->last_time == 0) {
      this->last_time = this->Get_SystemTick();
      return 1;
    }
    now_time = this->Get_SystemTick();

    /*Overflow*/
    if (now_time < this->last_time) {
      if (this->last_time - now_time > INT32_MAX) // 真溢出
        this->dt = (float)(now_time + (0xFFFFFFFF - this->last_time));
      else {
        return (uint8_t)this->dt; // 假溢出返回上一次dt
      }
    } else
      this->dt = (float)(now_time - this->last_time);

    this->last_time = now_time;

    this->dt *= 0.000001f;

    return 0;
  } else {
    this->dt = 0;
    return 1;
  }
}

/**
 * @brief 设置pid参数
 *
 * @param _Kp kp
 * @param _Ki ki
 * @param _Kd kd
 * @param _I_Term_Max i项的输出限制
 * @param _Out_Max 总输出限制
 */
void PID_t::SetPIDParam(float _Kp, float _Ki, float _Kd, float _I_Term_Max,
                        float _Out_Max) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  I_Term_Max = _I_Term_Max;
  Out_Max = _Out_Max;
}

/**
 * @brief pid计算函数
 *
 * @return float
 */
float PID_t::Adjust() {

  /*Error time*/
  if (this->UpdateTimeStamp())
    return 0;

  this->Error = this->Target - this->Current;

  if (fabs(this->Error) < this->DeadZone) {
    this->Out = 0;
    return this->Out;
  }

  /* Using Low-Pass Filter to preprocess*/
  if (this->ErrorFilter != NULL)
    this->Error = this->ErrorFilter(this->Error);

  this->P_Term = this->Kp * this->Error;

  /* PID with Changing integration rate */
  float I_VarSpeedf = 0;
  if (fabs(this->Error) <= this->VarSpeed_I_B)
    I_VarSpeedf = 1;
  else if (fabs(this->Error) <=
           (double)(this->VarSpeed_I_A) + this->VarSpeed_I_B)
    I_VarSpeedf =
        (this->VarSpeed_I_A - (fabs(this->Error)) + this->VarSpeed_I_B) /
        this->VarSpeed_I_A;

  float integral_max = fabsf(this->I_Term_Max / this->Ki);
  if (fabs(this->Ki) > numeric_limits_float_epsilon) {
    this->integral_e += I_VarSpeedf * this->Error * this->dt;
    /*Constrain*/
    this->integral_e = func_limit(this->integral_e, integral_max);
  } else {
    this->integral_e = 0;
  }

  /* Using Integral separation */
  if (fabs(this->Error) < this->I_SeparateThresh) {
    this->I_Term = this->Ki * this->integral_e;
    this->I_Term = func_limit(this->I_Term, this->I_Term_Max);
  } else {
    /*Clear*/
    this->I_Term = 0;
  }

  float d_err = 0;
  if (this->D_of_Current)
    d_err = (this->Current - this->pre_current) / this->dt;
  else
    d_err = (this->Error - this->pre_error) / this->dt;

  if (this->DiffFilter != NULL)
    d_err = this->DiffFilter(d_err);
  this->D_Term = this->Kd * d_err;

  this->pre_error = this->Error;
  this->pre_current = this->Current;

  this->Out = this->P_Term + this->I_Term + this->D_Term;

  /*limiting*/
  this->Out = func_limit(this->Out, this->Out_Max);

  return this->Out;
}

void PID_t::clean_out() { this->Out = 0.f; }

/**
 * @brief PID清除积分项函数
 *
 */
void PID_t::clean_integral() { this->integral_e = 0; }

/**
 * @brief 设置误差滤波器
 *
 * @param _set_error_filter
 */
void PID_t::setErrorFilter(float (*_set_error_filter)(float)) {
  this->ErrorFilter = _set_error_filter;
}

/**
 * @brief 设置差分滤波器
 *
 * @param _set_diff_filter
 */
void PID_t::setDiffFilter(float (*_set_diff_filter)(float)) {
  this->DiffFilter = _set_diff_filter;
}