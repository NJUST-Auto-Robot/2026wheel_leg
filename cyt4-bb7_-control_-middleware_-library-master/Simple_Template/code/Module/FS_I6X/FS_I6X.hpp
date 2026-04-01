/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: FS_I6X.hpp
 * @Description: 用于富斯I6X遥控器
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "System/usr_system.hpp"
#include "zf_common_headfile.h"

/* Private macros ------------------------------------------------------------*/
#define USE_FS_I6X (1)

#ifdef USE_FS_I6X
/* Private type --------------------------------------------------------------*/
// 拨杆类型
enum SW_Status_Typedef {
  SW_NONE = 0,
  SW_UP = 1,
  SW_MID = 3,
  SW_DOWN = 2,
};

/**
  @brief SBUS数据包内容
*/
#pragma pack(1)
struct SBUS_DataPack_Typedef {
  uint8_t HEAD;
  uint64_t ch1 : 11;
  uint64_t ch2 : 11;
  uint64_t ch3 : 11;
  uint64_t ch4 : 11;
  uint64_t ch5 : 11;
  uint64_t ch6 : 11;
  uint64_t ch7 : 11;
  uint64_t ch8 : 11;
  uint64_t ch9 : 11;
  uint64_t ch10 : 11;
  uint64_t ch11 : 11;
  uint64_t ch12 : 11;
  uint64_t ch13 : 11;
  uint64_t ch14 : 11;
  uint64_t ch15 : 11;
  uint64_t ch16 : 11;
  uint8_t END;
};
#pragma pack()
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#ifdef __cplusplus
class FS_I6X_Classdef {
private:
  static const uint16_t LOST_THRESHOLD = 100;
  uint32_t last_recv_time;             /*<! 上一次在线检测时间*/
  LinkageStatus_Typedef Status;        /*<! 连接状态 */
  SBUS_DataPack_Typedef DataPack = {}; /*<! 数据包*/
  float RX_Norm, RY_Norm, LX_Norm, LY_Norm;
  float VRA_Norm, VRB_Norm;
  SW_Status_Typedef SWA, SWB, SWC, SWD, preSWA, preSWB, preSWC, preSWD;

  SW_Status_Typedef SW_Process(uint16_t ch_val);

public:
  float DeadZone = 0.05f; // 摇杆死区

  /* 数据接收、处理函数 */
  inline void DataCapture(void *addr) { memcpy(&DataPack, addr, 25); }
  void DataProcess();

  /* 连接状态相关操作 */
  void Check_Link(uint32_t current_check_time);
  LinkageStatus_Typedef GetStatus() { return Status; }

  /* 获取遥控信息 */
  float Get_RX_Norm(void) { return RX_Norm; }     // 摇杆值（归一化）
  float Get_RY_Norm(void) { return RY_Norm; }     // 摇杆值（归一化）
  float Get_LX_Norm(void) { return LX_Norm; }     // 摇杆值（归一化）
  float Get_LY_Norm(void) { return LY_Norm; }     // 摇杆值（归一化）
  float Get_VRA_Norm(void) { return VRA_Norm; }   // 旋钮值（归一化）
  float Get_VRB_Norm(void) { return VRB_Norm; }   // 旋钮值（归一化）
  SW_Status_Typedef Get_SWA(void) { return SWA; } // 拨杆状态
  SW_Status_Typedef Get_SWB(void) { return SWB; } // 拨杆状态
  SW_Status_Typedef Get_SWC(void) { return SWC; } // 拨杆状态
  SW_Status_Typedef Get_SWD(void) { return SWD; } // 拨杆状态
  SW_Status_Typedef Get_preSWA(void) { return preSWA; } // 拨杆状态
  SW_Status_Typedef Get_preSWB(void) { return preSWB; } // 拨杆状态
  SW_Status_Typedef Get_preSWC(void) { return preSWC; } // 拨杆状态
  SW_Status_Typedef Get_preSWD(void) { return preSWD; } // 拨杆状态
  const SBUS_DataPack_Typedef &Get_Raw_SBUS_Data() {
    return DataPack;
  } // SBUS原始数据
};

extern FS_I6X_Classdef remote;
#endif // ! __cplusplus
/* Exported function declarations --------------------------------------------*/
#if __cplusplus
// 串口中断回调函数
extern "C" {
#endif
uint32_t FS_I6X_RxCpltCallback(uint8_t *Recv_Data, uint32_t ReceiveLen);
void tskFS_I6X(void *arg);
#if __cplusplus
}
#endif // __cplusplus
#endif // USE_FS_I6X