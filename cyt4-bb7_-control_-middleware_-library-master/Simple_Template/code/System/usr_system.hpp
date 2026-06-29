/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: usr_system.hpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#pragma once
#include "cmsis_os.h"

#define Tiny_Stack_Size (64)
#define Small_Stack_Size (128)
#define Normal_Stack_Size (256)
#define Large_Stack_Size (512)
#define Huge_Stack_Size (1024)

#define PriorityVeryLow (1)
#define PriorityLow (2)
#define PriorityBelowNormal (3)
#define PriorityNormal (4)
#define PriorityAboveNormal (5)
#define PriorityHigh (6)
#define PrioritySuperHigh (7)
#define PriorityRealtime (8)

/**
  @brief 连接状态
*/
enum LinkageStatus_Typedef {
  LOST = 0U,
  ESTABLISHED,
};
#define LOST_FILL_BYTE (0xa5U)

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// 用户系统初始化函数
void usrSystemInit(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#ifdef __cplusplus
class USR_SYSTEM {
public:
  USR_SYSTEM() {}
  ~USR_SYSTEM() {}

  // 系统外设初始化函数
  void peripheralInit();

  // 系统任务初始化函数
  void TaskCreate();

private:
};
#endif