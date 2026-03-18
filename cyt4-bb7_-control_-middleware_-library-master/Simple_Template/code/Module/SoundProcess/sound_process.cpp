/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-08
 * @FilePath: sound_process.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Module/SoundProcess/sound_process.hpp"
#include "arm_const_structs.h"
#include <cmath>

// arm库的atan2函数
extern "C" arm_status arm_atan2_f32(float32_t y, float32_t x,
                                    float32_t *result);

// 滤波器的float32_缓存区
float32_t filter_input_buffer[MIC_RAW_SIZE];
float32_t filter_output_buffer[MIC_RAW_SIZE];

// 带通滤波器
BandPassFilter<int16_t> band_pass_filter(SAMPLE_FRQ, 200, 2200,
                                         filter_input_buffer,
                                         filter_output_buffer);

// 每个麦克风的adc通道
// adc_channel_enum sound_adc_ch_s[MicPos_TotalNum] = {
//     /*MicPos_Front*/ ADC0_CH16_P07_0,
//     /*MicPos_Back*/ ADC0_CH17_P07_1,
//     /*MicPos_Right*/ ADC0_CH19_P07_3,
//     /*MicPos_Left*/ ADC0_CH18_P07_2
//     /**/};
adc_channel_enum sound_adc_ch_s[MicPos_TotalNum] = {
    /*MicPos_Front*/ ADC0_CH00_P06_0,
    /*MicPos_Back*/ ADC0_CH01_P06_1,
    /*MicPos_Right*/ ADC0_CH02_P06_2,
    /*MicPos_Left*/ ADC0_CH03_P06_3
    /**/};

// 各个麦克风的直流偏置值
// uint16_t mic_adc_offset_s[MicPos_TotalNum] = {
//     /*MicPos_Front*/ 1864,
//     /*MicPos_Back*/ 1856,
//     /*MicPos_Right*/ 1873,
//     /*MicPos_Left*/ 1877
//     /**/};
uint16_t mic_adc_offset_s[MicPos_TotalNum] = {
    /*MicPos_Front*/ 1874,
    /*MicPos_Back*/ 1876,
    /*MicPos_Right*/ 1870,
    /*MicPos_Left*/ 1865
    /**/};

// adc原始数据二维数组
int16_t mic_raw_data[MicPos_TotalNum][MIC_RAW_SIZE];

// adc带通滤波数据二维数组
int16_t mic_filter_data[MicPos_TotalNum][MIC_RAW_SIZE];

// 循环数组的索引值
uint16_t mic_raw_data_idx;
// DEBUG！！！
// uint16_t mic_raw_data_idx = MIC_RAW_SIZE - 1;
// DEBUG！！！
// 为了避免idx在中断后不断变化，用于记住当下处理时候的索引
uint16_t mic_raw_data_now_idx;

// 定义取出数据
float signal_to_fft[MicPos_TotalNum][FFT_SIZE * 2];
// 定义互相关后的数据,互相关后降维了
float signal_to_ifft[MicPos_TotalNum / 2][FFT_SIZE * 2];
// 输出的数据
float output_signal[MicPos_TotalNum / 2][FFT_SIZE];
// 最大幅值的索引
uint16_t max_mag_idx[MicPos_TotalNum / 2];
// 时差单位数组，0：纵向，1：横向
int8_t TimeDifferenceUnit[MicPos_TotalNum / 2];
// 声音方位角
float soundAzimuth = 0;

// 双曲线定位角度制表(单位：度)
uint8_t hyperbolic_pos_ang_s[] = {0,  4,  8,  12, 17, 21, 25, 30,
                                  34, 40, 46, 52, 59, 68, 90};
//  0    4.0960    8.2132   12.3736   16.6015   20.9248   25.3769   30.0000   34.8499
//  40.0052   45.5847   51.7868   58.9973   68.2132   90.0000

/**
 * @brief 从adc中读取信号值
 *
 * getMicDataFromADC(mic_raw_data);
 */
void getMicDataFromADC(int16_t _mic_raw_data[MicPos_TotalNum][MIC_RAW_SIZE]) {
  // // 采集前硅麦信号
  // _mic_raw_data[MicPos_Front][mic_raw_data_idx] =
  //     adc_convert(MIC_FRONT_ADC_CH) - MIC_FRONT_OFFSET;
  // // 采集后硅麦信号
  // _mic_raw_data[MicPos_Back][mic_raw_data_idx] =
  //     adc_convert(MIC_BACK_ADC_CH) - MIC_BACK_OFFSET;
  // // 采集后硅麦信号
  // _mic_raw_data[MicPos_Left][mic_raw_data_idx] =
  //     adc_convert(MIC_LEFT_ADC_CH) - MIC_LEFT_OFFSET;
  // // 采集后硅麦信号
  // _mic_raw_data[MicPos_Right][mic_raw_data_idx] =
  //     adc_convert(MIC_RIGHT_ADC_CH) - MIC_RIGHT_OFFSET;

  // 采集各通道信息，并做偏置
  for (uint8_t i = 0; i < MicPos_TotalNum; ++i) {
    _mic_raw_data[i][mic_raw_data_idx] =
        adc_convert(sound_adc_ch_s[i]) - mic_adc_offset_s[i];
  }

  // 进行索引循环
  mic_raw_data_idx = (mic_raw_data_idx + 1) % MIC_RAW_SIZE;
}

// DEBUG！！！
// 信号频率
// #define SIGNAL_FRQ (500)
// DEBUG！！！

/**
 * @brief 声音结算主任务
 *
 * @param arg
 */
void soundProcessTask(void *arg) {
  const TickType_t xFreq = pdMS_TO_TICKS(10);
  TickType_t xLastWake = xTaskGetTickCount();

  // // 定义FFT对象
  // arm_cfft_instance_f32 arm_cfft_sR_f32_len2048;
  // // 初始化fft对象
  // arm_cfft_init_f32(&arm_cfft_sR_f32_len2048, FFT_SIZE);

  uint8_t i = 0;
  uint16_t j = 0;

  // DEBUG！！！
  // 生成测试信号
  // float tst_signal[2 * MIC_RAW_SIZE];
  // for (j = 0; j < 2 * MIC_RAW_SIZE; ++j) {
  //   tst_signal[j] = arm_sin_f32(2 * PI * SIGNAL_FRQ * j / SAMPLE_FRQ);
  // }
  // // 生成带时差的虚拟采样信号
  // float virtual_sig[MicPos_TotalNum][MIC_RAW_SIZE];
  // for (j = 0; j < MIC_RAW_SIZE; ++j) {
  //   virtual_sig[MicPos_Front][j] = tst_signal[j + 5];
  //   virtual_sig[MicPos_Back][j] = tst_signal[j];
  //   virtual_sig[MicPos_Right][j] = tst_signal[j + 5];
  //   virtual_sig[MicPos_Left][j] = tst_signal[j];
  // }
  // DEBUG！！！

  for (;;) {
    // 获取复数信息
    getMicComplexDataFromRaw(signal_to_fft, mic_raw_data);
    // DEBUG！！！
    // getMicComplexDataFromRaw(signal_to_fft, virtual_sig);
    // DEBUG！！！

    // 进行FFT,得到频域信息
    for (i = 0; i < MicPos_TotalNum; i++) {
      arm_cfft_f32(&arm_cfft_sR_f32_len2048, signal_to_fft[i], 0, 1);
    }

    // 计算后面和左面的共轭复数
    for (i = 0; i < MicPos_TotalNum / 2; ++i) {
      for (j = 0; j < FFT_SIZE; ++j) {
        signal_to_fft[2 * i + 1][2 * j + 1] =
            -(signal_to_fft[2 * i + 1][2 * j + 1]);
      }
    }

    for (i = 0; i < MicPos_TotalNum / 2; ++i) {
      // 执行复数乘法(逐个元素相乘)
      arm_cmplx_mult_cmplx_f32(signal_to_fft[i << 1], signal_to_fft[i << 1 + 1],
                               signal_to_ifft[i], FFT_SIZE);

      // 对复数乘法结果进行IFFT
      arm_cfft_f32(&arm_cfft_sR_f32_len2048, signal_to_ifft[i], 1, 1);

      // 求得时域信号的幅值
      arm_cmplx_mag_f32(signal_to_ifft[i], output_signal[i], FFT_SIZE);
    }

    max_mag_idx[0] = max_mag_idx[1] = 0;
    for (i = 0; i < MicPos_TotalNum / 2; ++i) {
      // 找到最大幅值的索引
      for (j = 0; j < FFT_SIZE; ++j) {
        if (output_signal[i][j] > output_signal[i][max_mag_idx[i]]) {
          max_mag_idx[i] = j;
        }
      }
      // 根据情况计算出时差单位
      TimeDifferenceUnit[i] = max_mag_idx[i] < FFT_SIZE / 2
                                  ? -(max_mag_idx[i])
                                  : FFT_SIZE - max_mag_idx[i];
    }

    // 计算方位角(-pi~+pi -> -180~+180)
    soundAzimuth =
        RAD_TO_DEG(atan2f(TimeDifferenceUnit[1], TimeDifferenceUnit[0]));

    // 延时以阻塞
    vTaskDelayUntil(&xLastWake, xFreq);
  }
}

/**
 *
 */
float getAzimuthFromYTimeDiff(int8_t time_diff, int8_t other_side_time_diff) {
  uint8_t abs_time_diff = abs(time_diff);
  int8_t other_side_sign_flag = other_side_time_diff > 0 ? 1 : -1;
  if (abs_time_diff > 14)
    return 0;

  if (time_diff < 0) {
    return other_side_sign_flag * (90 + hyperbolic_pos_ang_s[abs_time_diff]);
  } else
    return other_side_sign_flag * (90 - hyperbolic_pos_ang_s[abs_time_diff]);
}

float getAzimuthFromXTimeDiff(int8_t time_diff, int8_t other_side_time_diff) {
  uint8_t abs_time_diff = abs(time_diff);
  int8_t other_side_sign_flag = other_side_time_diff > 0 ? 1 : -1;
  int8_t sign_flag = time_diff > 0 ? 1 : -1;
  if (abs_time_diff > 14)
    return 0;
  if (other_side_sign_flag > 0) {
    return sign_flag * hyperbolic_pos_ang_s[abs_time_diff];
  } else {
    return sign_flag * (180 - hyperbolic_pos_ang_s[abs_time_diff]);
  }
}
/**
 * @brief 声音结算函数
 *
 * @param arg
 */
void soundProcessLoop(bool is_filter_used, SoundProcessAlgorithm_t algorithm) {
  uint8_t i = 0;
  uint16_t j = 0;

  // 进行带通滤波
  if (is_filter_used) {
    // 进行低通滤波
    for (i = 0; i < MicPos_TotalNum; ++i) {
      band_pass_filter.process(mic_raw_data[i], mic_filter_data[i],
                               MIC_RAW_SIZE);
    }
  }

  // DEBUG！！！
  // 生成测试信号
  // float tst_signal[2 * MIC_RAW_SIZE];
  // for (j = 0; j < 2 * MIC_RAW_SIZE; ++j) {
  //   tst_signal[j] = arm_sin_f32(2 * PI * SIGNAL_FRQ * j / SAMPLE_FRQ);
  // }
  // // 生成带时差的虚拟采样信号
  // float virtual_sig[MicPos_TotalNum][MIC_RAW_SIZE];
  // for (j = 0; j < MIC_RAW_SIZE; ++j) {
  //   virtual_sig[MicPos_Front][j] = tst_signal[j + 1];
  //   virtual_sig[MicPos_Back][j] = tst_signal[j];
  //   virtual_sig[MicPos_Right][j] = tst_signal[j];
  //   virtual_sig[MicPos_Left][j] = tst_signal[j + 5];
  // }
  // DEBUG！！！

  // 获取复数信息
  if (is_filter_used == false) // 不使用滤波，则原始数据
    getMicComplexDataFromRaw(signal_to_fft, mic_raw_data);
  else // 使用滤波，则滤波后数据
    getMicComplexDataFromRaw(signal_to_fft, mic_filter_data);
  // DEBUG！！！
  // DEBUG！！！
  // getMicComplexDataFromRaw(signal_to_fft, virtual_sig);
  // for (j = 0; j < 2 * FFT_SIZE; ++j) {
  //   signal_to_fft_back[j] = signal_to_fft[MicPos_Back][j];
  // }
  // DEBUG！！！

  // 进行FFT,得到频域信息
  for (i = 0; i < MicPos_TotalNum; i++) {
    arm_cfft_f32(&arm_cfft_sR_f32_len2048, signal_to_fft[i], 0, 1);
  }

  // 计算后面和左面的共轭复数
  for (i = 0; i < MicPos_TotalNum / 2; ++i) {
    for (j = 0; j < FFT_SIZE; ++j) {
      signal_to_fft[2 * i + 1][2 * j + 1] =
          -(signal_to_fft[2 * i + 1][2 * j + 1]);
    }
  }

  for (i = 0; i < MicPos_TotalNum / 2; ++i) {
    // 执行复数乘法(逐个元素相乘)
    arm_cmplx_mult_cmplx_f32(signal_to_fft[2 * i], signal_to_fft[i * 2 + 1],
                             signal_to_ifft[i], FFT_SIZE);

    // 对复数乘法结果进行IFFT
    arm_cfft_f32(&arm_cfft_sR_f32_len2048, signal_to_ifft[i], 1, 1);

    // 求得时域信号的幅值
    arm_cmplx_mag_f32(signal_to_ifft[i], output_signal[i], FFT_SIZE);
  }

  max_mag_idx[0] = max_mag_idx[1] = 0;
  for (i = 0; i < MicPos_TotalNum / 2; ++i) {
    // 找到最大幅值的索引
    for (j = 0; j < FFT_SIZE; ++j) {
      if (output_signal[i][j] > output_signal[i][max_mag_idx[i]]) {
        max_mag_idx[i] = j;
      }
    }
    // 根据情况计算出时差单位
    TimeDifferenceUnit[i] = max_mag_idx[i] < FFT_SIZE / 2
                                ? -(max_mag_idx[i])
                                : FFT_SIZE - max_mag_idx[i];
  }

  // 计算方位角
  switch (algorithm) {
  case AverageBilateralPositioning: // 平均双曲线渐行线定位
    soundAzimuth =
        (getAzimuthFromYTimeDiff(TimeDifferenceUnit[0], TimeDifferenceUnit[1]) +
         getAzimuthFromXTimeDiff(TimeDifferenceUnit[1],
                                 TimeDifferenceUnit[0])) /
        2;
    break;
  case HorizontalBilateralPositioning: // 水平双曲线定位
    soundAzimuth =
        getAzimuthFromXTimeDiff(TimeDifferenceUnit[1], TimeDifferenceUnit[0]);
    break;
  case VerticalBilateralPositioning: // 垂直双曲线定位
    soundAzimuth =
        getAzimuthFromYTimeDiff(TimeDifferenceUnit[0], TimeDifferenceUnit[1]);
    break;
  case CoordinatesTriangularPositioning: // 三角坐标定位
    arm_atan2_f32(TimeDifferenceUnit[1], TimeDifferenceUnit[0], &soundAzimuth);
    soundAzimuth = RAD_TO_DEG(soundAzimuth);
    break;
  default:
    break;
  }
}
