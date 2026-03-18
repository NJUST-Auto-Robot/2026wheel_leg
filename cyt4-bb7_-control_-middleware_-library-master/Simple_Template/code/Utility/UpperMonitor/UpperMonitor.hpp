/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-05
 * @FilePath: UpperMonitor.hpp
 * @Description: 
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.   
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved. 
 */
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "zf_common_headfile.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
typedef union type_change // 数据传输共用体结构
{
  uint8_t change_u8[4]; // 8位无符号整型【1个字节】
  float change_float;   // 32位浮点型数据【4个字节】
  int change_int;       // 32位有符号整型【4个字节】
  short int change_u16; // 16位有符号整型【2个字节】
} type_change_t;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
void Sent_Contorl(uart_index_enum uartn);
uint32_t RecHandle(uint8_t *data_buf, uint32_t length);
void UpperMonitorTask(void *arg);

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/