/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: ipc.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Module/IPC/ipc.hpp"

// 接收的方位角范围是0-360
uint32_t ipc_get_data = 0;

// 接收错误计数
uint32_t ipc_error_count = 0;

/**
 * @brief ipc中断回调函数
 *
 * @param receive_data 接收到的方位角数据
 * @return uint32_t 接收成功返回1，接收失败返回0
 */
void my_ipc_callback(uint32 receive_data) {
  if (receive_data > 360) {
    ++ipc_error_count;
    return;
  }

  ipc_get_data = receive_data;
}