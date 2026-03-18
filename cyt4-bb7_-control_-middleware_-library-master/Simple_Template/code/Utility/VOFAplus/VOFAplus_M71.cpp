/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\Utility\VOFAplus\VOFAplus_M71.cpp
 * @Description: 给M71核使用的vofa+
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 * _______________#########_______________________
 * ______________############_____________________
 * ______________#############____________________
 * _____________##__###########___________________
 * ____________###__######_#####__________________
 * ____________###_#######___####_________________
 * ___________###__##########_####________________
 * __________####__###########_####_______________
 * ________#####___###########__#####_____________
 * _______######___###_########___#####___________
 * _______#####___###___########___######_________
 * ______######___###__###########___######_______
 * _____######___####_##############__######______
 * ____#######__#####################_#######_____
 * ____#######__##############################____
 * ___#######__######_#################_#######___
 * ___#######__######_######_#########___######___
 * ___#######____##__######___######_____######___
 * ___#######________######____#####_____#####____
 * ____######________#####_____#####_____####_____
 * _____#####________####______#####_____###______
 * ______#####______;###________###______#________
 * ________##_______####________####______________
 */

/* Includes ------------------------------------------------------------------*/
#include "Algorithm/Filters/filters.hpp"
#include "Module/SoundProcess/sound_process.hpp"
#include "System/usr_uart.hpp"
#include "Utility/VOFAplus/VOFAplus.hpp"
#include "zf_driver_adc.h"
#include "zf_driver_uart.h"
#include <cmath>

/* Configurations ------------------------------------------------------------*/
using namespace VofaMonitor;

// extern uint16_t tst_idx;
// extern int16_t input[1000];
// extern int16_t output[1000];
/* Messages ------------------------------------------------------------------*/
#if VOFAMONITOR_MAX_DATA_NUM < 0
#error "The maximum amount of "Max_Sent_Data_Num" must be greater than 0."
#else
// #warning "Please pay attention to the baud rate of the serial port
//(UART/USART) to avoid data loss. "
#endif
/* Private variables ---------------------------------------------------------*/

namespace VofaMonitor {
uint8_t DataNum = 0; // 记录当前需要传输的数据量
Type_change_t SendDataPool[VOFAMONITOR_MAX_DATA_NUM + 1]; // 传输数据池
} // namespace VofaMonitor
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * @brief vofa+设置发送数据并发送
 *
 */
static void vofaSetDataAndSend(void) {
  uint16_t data_idx_save = mic_raw_data_idx;
  setData(0, // 从index 0开始
          /*0*/ soundAzimuth,
          //  /*1*/ adc_convert(sound_adc_ch_s[MicPos_Front]),
          //  /*2*/ adc_convert(sound_adc_ch_s[MicPos_Back]),
          //  /*3*/ adc_convert(sound_adc_ch_s[MicPos_Right]),
          //  /*4*/ adc_convert(sound_adc_ch_s[MicPos_Left])
          //  /*1*/ input[tst_idx],
          //  /*2*/ output[tst_idx]
          /*1*/ TimeDifferenceUnit[0],
          /*2*/ TimeDifferenceUnit[1],
          /*3*/ mic_raw_data[MicPos_Front][data_idx_save],
          /*4*/ mic_raw_data[MicPos_Back][data_idx_save],
          /*5*/ mic_raw_data[MicPos_Right][data_idx_save],
          /*6*/ mic_raw_data[MicPos_Left][data_idx_save],
          // /*3*/ mic_filter_data[MicPos_Front][data_idx_save],
          // /*4*/ mic_filter_data[MicPos_Back][data_idx_save],
          // /*5*/ mic_filter_data[MicPos_Right][data_idx_save],
          // /*6*/ mic_filter_data[MicPos_Left][data_idx_save],
          /*7*/
          getAzimuthFromYTimeDiff(TimeDifferenceUnit[0], TimeDifferenceUnit[1]),
          /*8*/
          getAzimuthFromXTimeDiff(TimeDifferenceUnit[1], TimeDifferenceUnit[0])
          /*9*/
          /*占据空行*/);
  send(0);
}

/**
 * @brief vofa+的裸机运行任务
 *
 */
extern "C" void vofaTaskForBareMetal(void) { vofaSetDataAndSend(); }

/**
 * @brief vofa的freertos运行任务
 *
 * @param argument NULL
 */
void vofaTask(void *argument) {
  // 任务执行初始化
  const TickType_t xFreq = pdMS_TO_TICKS(40);
  TickType_t xLastWake = 0;
  xLastWake = xTaskGetTickCount();

  // 客制化初始化

  for (;;) {
    // setData(0, tst_1, tst_2, tst_cnt, tst_sin);

    vofaSetDataAndSend();

    // tst_idx++;
    // if (tst_idx >= 1000)
    //   tst_idx = 0;

    // 堵塞以任务切换
    vTaskDelayUntil(&xLastWake, xFreq);
  }
}

bool uart1_Send_DMA(uint8_t *pData, uint16_t Size) // DMA发送
{
  // if (TXingFlag == true)
  //   return false; // 串口发送忙,放弃发送该帧数据
  // 在发送中断中已经失能发送流
  // // LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7); //
  //  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7,Size);
  //  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)pData);
  // // 在客制化初始化已经设置好了
  // LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)&USART1->TDR);

  // 在客制化初始化已经设置好了
  // // LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);

  // LL_DMA_EnableStream(
  //     DMA2,
  //     LL_DMA_STREAM_7); // 发送流使能就应该在每一次发送打开，发送成功后关闭
  // TXingFlag = true;
  return true;
}

/**
 * @brief vofa的发送函数
 *
 * @param pUSART_COB 串口发送处理包
 * @return true
 * @return false
 */
bool Vofa_UART_Transmit(USART_COB *pUSART_COB) {

  // 假如空指针，退出
  if (SendFucArray[pUSART_COB->port_num] == nullptr)
    return false;

  // 不是空指针，则运行发送函数
  return SendFucArray[pUSART_COB->port_num](pUSART_COB->address,
                                            pUSART_COB->len);
}

/**
 * @brief 向Vofa上位机发送数据
 *
 * @param uart_id 发送的串口ID号
 */
void VofaMonitor::send(uint8_t uart_id) {
  static USART_COB TxPack = {1, 0, (uint8_t *)SendDataPool};

  SendDataPool[DataNum + 1].change_u8[0] = 0x00;
  SendDataPool[DataNum + 1].change_u8[1] = 0x00;
  SendDataPool[DataNum + 1].change_u8[2] = 0x80;
  SendDataPool[DataNum + 1].change_u8[3] = 0x7f;

  TxPack.len = sizeof(Type_change_t) * (DataNum + 2);
  TxPack.port_num = uart_id;
  Vofa_UART_Transmit(&TxPack);
  DataNum = 0;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
