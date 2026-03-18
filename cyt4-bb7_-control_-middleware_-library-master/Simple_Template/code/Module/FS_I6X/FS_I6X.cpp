/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: FS_I6X.cpp
 * @Description: 用于富斯I6X遥控器
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Module/FS_I6X/FS_I6X.hpp"
#include "AAARobot/AAARobot.hpp"
#include "Module/RefereSystem/RefereSystem.hpp"

FS_I6X_Classdef remote;

/**
 * @brief FS_I6X的执行任务
 *
 * @param arg
 */
void tskFS_I6X(void *arg) {
  TickType_t xLastWakeTime = 0, xNowTaskTickCnt = 0;
  const TickType_t xFreq = pdMS_TO_TICKS(100);
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFreq);
    // if (ulTaskNotifyTake(pdTRUE, 100) != 0) {
    //   remote.DataProcess(); // 数据处理
    // }
    xNowTaskTickCnt = xTaskGetTickCount();

    /* 检测遥控器连接 */
    remote.Check_Link(xNowTaskTickCnt);

    // 监测裁判系统的链接
    vRefereeSystemCheckLink(xNowTaskTickCnt);

    // 监测电调数据的连接
    vEscDataCheckLink(xNowTaskTickCnt);

    // /*  判断是否连接   */
    // if (remote.GetStatus() != ESTABLISHED) {
    //   // todo
    //   continue;
    // }
  }
}

/**
 * @brief 富斯I6X遥控器串口回调函数
 *
 * @param Recv_Data
 * @param ReceiveLen
 * @return uint32_t
 */
uint32_t FS_I6X_RxCpltCallback(uint8_t *Recv_Data, uint32_t ReceiveLen) {
  if (ReceiveLen >= 25) {
    // BaseType_t xHigherPriorityTaskWoken;
    remote.DataCapture(Recv_Data);
    //   vTaskNotifyGiveFromISR((TaskHandle_t)remoteCtrlUnitTaskHandle,
    //                          &xHigherPriorityTaskWoken);

    //   /* 如果xHigherPriorityTaskWoken现在设置为pdTRUE，则进行上下文切换
    // 应该执行以确保中断直接返回到最高
    // 优先级的任务。用于此目的的宏取决于中的端口
    // 使用和可以称为portEND_SWITCHING_ISR()。*/
    //   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    remote.DataProcess(); // 数据处理
  }
  return 0;
}

/**
 * @brief 将拨杆通道值处理成拨杆状态
 *
 * @param ch_val 拨杆通道值
 * @return SW_Status_Typedef 拨杆状态
 */
SW_Status_Typedef FS_I6X_Classdef::SW_Process(uint16_t ch_val) {
  if (ch_val < 300)
    return SW_UP;
  else if (ch_val > 1750)
    return SW_DOWN;
  else
    return SW_MID;
}

/**
 * @brief 连接检测函数
 *
 * @param current_check_time 当前系统时间，单位为ms
 */
void FS_I6X_Classdef::Check_Link(uint32_t current_check_time) {
  /*开始检测*/
  if (DataPack.HEAD == 0x0f && DataPack.END == 0) {
    Status = ESTABLISHED;
    last_recv_time = current_check_time;
  } else if (current_check_time - last_recv_time > LOST_THRESHOLD) {
    Status = LOST;
  }
  DataPack.HEAD = DataPack.END = 0xA5;
}

/**
 * @brief 数据处理函数
 *
 */
void FS_I6X_Classdef::DataProcess() {
  // 要摇杆映射为-1.1至1.1，然后限幅为-1至1，避免部分摇杆无法到达1的情况
  RX_Norm = DeadZone_Process((DataPack.ch1 - 1024) / 784.0f * 1.1f, -DeadZone,
                             DeadZone, 0);
  RY_Norm = DeadZone_Process((DataPack.ch2 - 1024) / 784.0f * 1.1f, -DeadZone,
                             DeadZone, 0);
  LY_Norm = DeadZone_Process((DataPack.ch3 - 1024) / 784.0f * 1.1f, -DeadZone,
                             DeadZone, 0);
  LX_Norm = DeadZone_Process((DataPack.ch4 - 1024) / 784.0f * 1.1f, -DeadZone,
                             DeadZone, 0);
  VRA_Norm = DeadZone_Process((DataPack.ch5 - 1024) / 784.0f * 1.1f, -DeadZone,
                              DeadZone, 0);
  VRB_Norm = DeadZone_Process((DataPack.ch6 - 1024) / 784.0f * 1.1f, -DeadZone,
                              DeadZone, 0);

  RX_Norm = func_limit_ab(RX_Norm, -1.f, 1.f);
  RY_Norm = func_limit_ab(RY_Norm, -1.f, 1.f);
  LY_Norm = func_limit_ab(LY_Norm, -1.f, 1.f);
  LX_Norm = func_limit_ab(LX_Norm, -1.f, 1.f);
  VRA_Norm = func_limit_ab(VRA_Norm, -1.f, 1.f);
  VRB_Norm = func_limit_ab(VRB_Norm, -1.f, 1.f);

  preSWA = SWA;
  SWA = SW_Process(DataPack.ch7);
  preSWB = SWB;
  SWB = SW_Process(DataPack.ch8);
  preSWC = SWC;
  SWC = SW_Process(DataPack.ch9);
  preSWD = SWD;
  SWD = SW_Process(DataPack.ch10);
}