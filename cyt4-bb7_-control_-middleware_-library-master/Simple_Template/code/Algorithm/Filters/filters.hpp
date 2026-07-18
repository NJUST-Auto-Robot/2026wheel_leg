/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-05
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: \iard:\0xFF
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Algorithm\Filters\filters.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    filter.h
 * @author  buff buffdemail@163.com
 * @brief   Filter set in general signal process and analysis.
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _FILTER_H
#define _FILTER_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include "arm_math.h"
#include <algorithm>
#include <string.h>
/* Exported function declarations --------------------------------------------*/
/* LowPassFilter */
class LowPassFilter {
public:
  /**
    @brief trust (0,1)
   */
  LowPassFilter(float trust = 1) : Trust(trust) {
    now_num = 0;
    last_out = 0;
  }
  ~LowPassFilter(){};
  float Trust;
  void operator<<(const float &);
  void operator>>(float &);
  float f(float num);

protected:
  void in(float num);
  float out();

private:
  float now_num;
  float last_out;
};

/* MedianFilter	*/
template <int Length> class MedianFilter {
  /**
    @brief 滤波宽度(1,100)
   */
public:
  MedianFilter() {
    static_assert((Length > 0) && (Length < 101),
                  "MedianFilter Length [1,100]");
    flag = Length;
    where_num = 0;
  }
  ~MedianFilter(){};
  void operator>>(float &num) { num = out(); }
  void operator<<(const float &num) { in(num); }
  float f(float num) {
    in(num);
    return (out());
  }

protected:
  void in(float num) {
    now_num = num;
    /* flag=Length然后递减保证宽度内都是有效波值 */
    flag > 0 ? flag-- : 0;
    buffer_num[where_num++] = num;
    where_num %= Length;
  }

  float out() {
    if (flag > 0)
      return now_num;
    else {
      /* 准备排序 */
      memcpy(sort_num, buffer_num, sizeof(sort_num));
      std::sort(sort_num, sort_num + Length);
      return sort_num[int(Length / 2)];
    }
  }

private:
  float buffer_num[Length];
  float sort_num[Length];
  float now_num;
  int flag, where_num;
};

/* MeanFilter */
template <int Length> class MeanFilter {
public:
  /**
    @brief 滤波宽度(1,100)
   */
  MeanFilter() {
    static_assert((Length > 0) && (Length < 101),
                  "MedianFilter Length [1,100]");
    for (int x = 0; x < Length; x++)
      buffer_num[x] = 0;
    flag = Length;
    where_num = 0;
    sum = 0;
  }
  ~MeanFilter(){};
  void operator>>(float &num) { num = out(); }
  void operator<<(const float &num) { in(num); }
  float f(float num) {
    in(num);
    return (out());
  }

protected:
  void in(float num) {
    now_num = num;
    sum -= buffer_num[where_num]; /*<! sum减去旧值 */
    sum += num;                   /*<! sum加上新值 */
    buffer_num[where_num++] = num;
    flag > 0 ? flag-- : 0; /*<!flag=Length然后递减保证宽度内都是有效波值 */
    where_num %= Length;
  }

  float out() {
    if (flag > 0)
      return now_num;
    else
      return (sum / Length);
  }

private:
  float buffer_num[Length];
  float now_num;
  float sum; /*<! 宽度和数字和 */
  int flag, where_num;
};

/**
 * @brief 带通滤波类
 *
 * float32_t input[1000];
 * float32_t output[1000];
 *
 * // 创建带通滤波器实例
    BandPassFilter filter(1000, 100, 500);

      // 处理信号
    filter.process(input, output, 1000);

     // 输出结果
    for (int i = 0; i < 1000; i++) {
        printf("%f\n", output[i]);
    }
 *
 */
template <typename T> class BandPassFilter {
public:
  BandPassFilter(float samplingRate, float lowCutoff, float highCutoff,
                 float32_t *input_buffer, float32_t *output_buffer)
      : input_buffer(input_buffer), output_buffer(output_buffer) {
    calculateBandPassFilterCoefficients(samplingRate, lowCutoff, highCutoff);
    arm_biquad_cascade_df1_init_f32(&S, 1, b, a);
  }

  void process(T *input, T *output, uint32_t length) {
    // 由于arm_biquad_cascade_df1_f32只支持float32_t类型，我们需要将输入数据转换为float32_t
    for (uint32_t i = 0; i < length; i++) {
      input_buffer[i] = static_cast<float32_t>(input[i]);
    }

    arm_biquad_cascade_df1_f32(&S, input_buffer, output_buffer, length);

    for (uint32_t i = 0; i < length; i++) {
      output[i] = static_cast<T>(output_buffer[i]);
    }
  }

private:
  arm_biquad_casd_df1_inst_f32 S;
  float32_t b[5];
  float32_t a[5];
  float32_t *input_buffer;
  float32_t *output_buffer;

  void calculateBandPassFilterCoefficients(float samplingRate, float lowCutoff,
                                           float highCutoff) {
    float32_t omegaL = 2.0f * PI * lowCutoff / samplingRate;
    float32_t omegaH = 2.0f * PI * highCutoff / samplingRate;
    float32_t alpha =
        sinf(omegaH - omegaL) / (2.0f * cosf((omegaH + omegaL) / 2.0f));

    b[0] = alpha;
    b[1] = 0;
    b[2] = -alpha;
    a[0] = 1 + alpha;
    a[1] = -2 * cosf((omegaH + omegaL) / 2.0f);
    a[2] = 1 - alpha;
  }
};

#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
