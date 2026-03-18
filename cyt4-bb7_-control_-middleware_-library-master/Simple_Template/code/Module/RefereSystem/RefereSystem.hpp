/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: RefereSystem.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
#include "System/usr_system.hpp"
#include "projdefs.h"
#include "zf_common_headfile.h"

#pragma pack(1)
typedef struct __RefereeInf_t {
  uint8_t head; // 0x66
  uint8_t ctrl_board_index;
  uint8_t check_byte; // ctrl_board_index ^ 0xff
  uint8_t tail;       // 0x88
  LinkageStatus_Typedef link_status = LOST;
  uint8_t pre_ctrl_board_index;
  uint8_t ctrl_is_switch; // 0x01: change, 0x00: no change
} RefereeInf_t;
#pragma pack()

// 接收到的裁判系统信息
extern RefereeInf_t referee_inf;

#ifdef __cplusplus
extern "C" {
#endif

// 解析裁判系统信息
uint32_t refereeSystemUnpack(uint8_t *buf, uint32_t len);

// 检查连接
void vRefereeSystemCheckLink(uint32_t current_check_time);
#ifdef __cplusplus
}
#endif
