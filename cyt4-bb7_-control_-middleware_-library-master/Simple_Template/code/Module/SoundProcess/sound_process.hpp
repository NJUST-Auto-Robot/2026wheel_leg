/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: \iard:\0xFF
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Module\SoundProcess\sound_process.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Module\SoundProcess\sound_process.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Module\SoundProcess\sound_process.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Module\SoundProcess\sound_process.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Module\SoundProcess\sound_process.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Module\SoundProcess\sound_process.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
#if __cplusplus
#include "Algorithm/Filters/filters.hpp"
#endif // __cpluspluss
#include "arm_math.h"
#include "zf_common_headfile.h"

// 使能音频处理功能
// #define ENABLE_SOUND_PROCESS

// 用于FFT计算的数据长度
#define FFT_SIZE (2048)
// 用于采集硅麦的循环数组的长度（大于FFT的数据量）
#define MIC_RAW_SIZE (2500)
// 采样频率
#define SAMPLE_FRQ (10000)

typedef enum {
  MicPos_Front = 0,
  MicPos_Back,
  MicPos_Right,
  MicPos_Left,
  MicPos_TotalNum
} MicPos_enum;

typedef enum {
  CoordinatesTriangularPositioning = 0,
  HorizontalBilateralPositioning,
  VerticalBilateralPositioning,
  AverageBilateralPositioning,
} SoundProcessAlgorithm_t;

// 每个麦克风的adc通道
extern adc_channel_enum sound_adc_ch_s[MicPos_TotalNum];

// adc原始数据二维数组
extern int16_t mic_raw_data[MicPos_TotalNum][MIC_RAW_SIZE];

// adc带通滤波数据二维数组
extern int16_t mic_filter_data[MicPos_TotalNum][MIC_RAW_SIZE];

// 循环数组的索引值
extern uint16_t mic_raw_data_idx;

// 为了避免idx在中断后不断变化，用于记住当下处理时候的索引
extern uint16_t mic_raw_data_now_idx;

// 声音结算得到的方位角
extern float soundAzimuth;

// 时差单位数组，0：纵向，1：横向
extern int8_t TimeDifferenceUnit[MicPos_TotalNum / 2];

#ifdef __cplusplus
/**
 * @brief 从原始数据中，读取得到复数信息（但虚部为0）
 *
 * @tparam T
 * @param _signal_to_fft
 * @param _mic_raw_data
 */
template <typename T>
void getMicComplexDataFromRaw(
    float _signal_to_fft[MicPos_TotalNum][FFT_SIZE * 2],
    T _mic_raw_data[MicPos_TotalNum][MIC_RAW_SIZE]) {
  uint16_t mic_raw_data_idx_tmp = 0;

  // 保存当前的索引，防止此处被中断打断，索引发生了改变
  mic_raw_data_now_idx = mic_raw_data_idx;

  // 如果当前采集位置小于所需要复制的长度，则两端分别复制数据
  if (mic_raw_data_now_idx < FFT_SIZE) {
    // 首先时间更靠前的数据正处于数组末端
    // 然后时间更靠后的数据正在数组头部
    // 找到环形数据剪开的索引点
    mic_raw_data_idx_tmp = MIC_RAW_SIZE - (FFT_SIZE - mic_raw_data_now_idx);
  } else {
    // 此时时间靠前的在前面的数组内部
    // 时间靠后的在mic_raw_data_now_idx索引的这一侧
    mic_raw_data_idx_tmp = 0;
  }

  for (uint16_t fft_data_idx = 0; fft_data_idx < FFT_SIZE; ++fft_data_idx) {
    // 保存用于FFT的数据，实部赋值
    _signal_to_fft[MicPos_Front][fft_data_idx * 2] =
        _mic_raw_data[MicPos_Front][mic_raw_data_idx_tmp];
    _signal_to_fft[MicPos_Back][fft_data_idx * 2] =
        _mic_raw_data[MicPos_Back][mic_raw_data_idx_tmp];
    _signal_to_fft[MicPos_Left][fft_data_idx * 2] =
        _mic_raw_data[MicPos_Left][mic_raw_data_idx_tmp];
    _signal_to_fft[MicPos_Right][fft_data_idx * 2] =
        _mic_raw_data[MicPos_Right][mic_raw_data_idx_tmp];
    // 保存用于FFT的数据，虚部赋值（虚部为0）
    _signal_to_fft[MicPos_Front][fft_data_idx * 2 + 1] = 0;
    _signal_to_fft[MicPos_Back][fft_data_idx * 2 + 1] = 0;
    _signal_to_fft[MicPos_Left][fft_data_idx * 2 + 1] = 0;
    _signal_to_fft[MicPos_Right][fft_data_idx * 2 + 1] = 0;
    // 索引++，这句代码经过处理后，可以应对从数组尾部到头部的跨越
    mic_raw_data_idx_tmp = (mic_raw_data_idx_tmp + 1) % MIC_RAW_SIZE;
  }
}
#endif

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
void getMicDataFromADC(int16_t _mic_raw_data[MicPos_TotalNum][MIC_RAW_SIZE]);
void soundProcessTask(void *arg);
void soundProcessLoop(bool is_filter_used, SoundProcessAlgorithm_t algorithm);
float getAzimuthFromYTimeDiff(int8_t time_diff, int8_t other_side_time_diff);
float getAzimuthFromXTimeDiff(int8_t time_diff, int8_t other_side_time_diff);
#ifdef __cplusplus
}
#endif // __cplusplus
