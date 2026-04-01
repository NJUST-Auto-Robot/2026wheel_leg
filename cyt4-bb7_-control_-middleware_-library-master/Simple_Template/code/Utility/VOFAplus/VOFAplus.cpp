/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: VOFAplus.cpp
 * @Description:
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
#include "Utility/VOFAplus/VOFAplus.hpp"
#include "AAARobot/AAARobot.hpp"
#include "Module/FS_I6X/FS_I6X.hpp"
#include "Module/MotorDrive/MotorDrive.hpp"
#include "Module/RefereSystem/RefereSystem.hpp"
#include "Module/UBX/ubx_decoder.h"
#include "System/usr_uart.hpp"
#include "zf_driver_uart.h"
#include <cmath>

/* Configurations ------------------------------------------------------------*/
using namespace VofaMonitor;
/* Messages ------------------------------------------------------------------*/
#if VOFAMONITOR_MAX_DATA_NUM < 0
#error "The maximum amount of "Max_Sent_Data_Num" must be greater than 0."
#else
// #warning "Please pay attention to the baud rate of the serial port
//(UART/USART) to avoid data loss. "
#endif
/* Private variables ---------------------------------------------------------*/
extern uint8_t tst_monitor_data0;
extern uint8_t tst_monitor_data10;
extern uint8_t tst_monitor_data100;

namespace VofaMonitor {
uint8_t DataNum = 0; // 记录当前需要传输的数据量
Type_change_t SendDataPool[VOFAMONITOR_MAX_DATA_NUM + 1]; // 传输数据池
} // namespace VofaMonitor
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief vofa的freertos运行任务
 *
 * @param argument NULL
 */
void vofaTask(void *argument) {
  // 任务执行初始化
  const TickType_t xFreq = 40;
  TickType_t xLastWake = 0;
  xLastWake = xTaskGetTickCount();

  // 客制化初始化

  for (;;) {
    // setData(0, tst_1, tst_2, tst_cnt, tst_sin);

    if (car.pFSI6X_data->GetStatus() == ESTABLISHED) /**/
    {
      setData(0, // 从index 0开始
              /*0*/ car.is_reversing,
              // /*1*/ car.switch_straight_cnt,
              /*0*/ car.gnss_data.speed,
              // /*1*/ car.esc_data.power,
              // car.motor_norm_duty,
              // /*2*/ car.where_goal_max,
              // /*3*/ car.gnss_data.positionError,
              // /*4*/ getGNSSFushionedAuz(),
              // car.closed_spd_set,
              // /*5*/ car.steer_pid.getOut(),
              // /*6*/ car.move_mode,
              /*7*/ car.how_far_goal,
              /*8*/ car.target_azimuth,
              // /*9*/ car.gnss_data.speed
              // *car.pServo_norm_duty,      /**/
              // car.steer_pid.getCurrent(), /**/
              // car.steer_pid.getTarget(),  /**/
              // car.steer_pid.getOut(),     /**/
              // car.steer_pid.getError(),   /**/
              // car.is_reversing, /**/
              *car.pServo_norm_duty,
              // car.pre_is_reversing,              /**/
              car.move_mode,                     /**/
              car.pReferee_inf->link_status,     /**/
              car.handle_snd_azimuth,            /**/
              car.pReferee_inf->ctrl_board_index /**/
              /*占据空行*/);
      send(0);
    }

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
