/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: \iard:\0xFF
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
#include "zf_common_headfile.h"
#ifndef VOFAMONITOR_MAX_DATA_NUM
#define VOFAMONITOR_MAX_DATA_NUM 10
#endif

#ifdef __cplusplus

/**
 * @brief USART message data type (Communication Object).
 */
typedef struct {
  uint8_t port_num;
  uint16_t len;
  uint8_t *address;
} USART_COB;

// vofa的freertos运行任务
void vofaTask(void *argument);

namespace VofaMonitor {
union Type_change_t {
  uint8_t change_u8[4]; // 8位无符号整型【1个字节】
  float change_float;   // 32位浮点型数据【4个字节】
};
const uint8_t Max_Data_Num = VOFAMONITOR_MAX_DATA_NUM;
extern Type_change_t SendDataPool[]; // 传输数据池
extern uint8_t DataNum;              // 记录当前需要传输的数据量

void send(uint8_t uart_id);

/**
 * @brief
 * @warning firstindex最小可以从0开始
 * @tparam T
 * @param first_index 传入的数据中，第一个数据放在第几条曲线
 * @param dataArg 需要发送的数据
 */
template <typename T> void setData(uint8_t index, T data) {
  if (index >= Max_Data_Num)
    return;
  SendDataPool[index].change_float = data;
  DataNum = index > DataNum ? index : DataNum;
}

template <typename T, typename... Ts>
void setData(uint8_t first_index, T data, Ts... dataArgs) {
  static_assert(sizeof...(dataArgs) < Max_Data_Num,
                "The sent variable exceeded Max_Data_Num of VofaMonitor");

  setData(first_index, data);
  first_index++;
  setData(first_index, dataArgs...);
}
} // namespace VofaMonitor

#endif /* __cplusplus */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
