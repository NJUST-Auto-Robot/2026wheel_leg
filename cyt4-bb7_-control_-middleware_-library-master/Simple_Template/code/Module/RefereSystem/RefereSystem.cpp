/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: RefereSystem.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Module/RefereSystem/RefereSystem.hpp"
#include "task.h"

RefereeInf_t referee_inf;
static uint32_t last_recv_time = 0;
static const uint16_t LOST_THRESHOLD = 500;

/**
 * @brief 裁判系统解包中断回调函数
 *
 * @param buf
 * @param len
 * @return uint32_t pdFALSE：失败 psPASS：成功
 */
uint32_t refereeSystemUnpack(uint8_t *buf, uint32_t len) {
  RefereeInf_t tem_ref_info;
  if (len != 4)
    return pdFALSE;

  memcpy(&tem_ref_info, buf, len);

  if (tem_ref_info.head != 0x66 || tem_ref_info.tail != 0x88 ||
      (tem_ref_info.check_byte != (tem_ref_info.ctrl_board_index ^ 0xff)))
    return pdFALSE;

  memcpy(&referee_inf, buf, len);
  return pdPASS;
}

/**
 * @brief 连接检测函数
 *
 * @param current_check_time 当前系统时间，单位为ms
 */
void vRefereeSystemCheckLink(uint32_t current_check_time) {
  /*开始检测*/
  if (referee_inf.head == 0x66 && referee_inf.tail == 0x88) {
    referee_inf.link_status = ESTABLISHED;
    last_recv_time = current_check_time;
  } else if (current_check_time - last_recv_time > LOST_THRESHOLD) {
    referee_inf.link_status = LOST;
  }
  referee_inf.head = referee_inf.tail = LOST_FILL_BYTE;
}

/**
 * @brief Referee System Get Link Status
 *
 * @return LinkageStatus_Typedef
 */
LinkageStatus_Typedef xRefereeSystemGetLinkStatus(void) {
  return referee_inf.link_status;
}