/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-08
 * @FilePath: usr_system_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system_M71.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system_M71.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: usr_system.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "usr_system.hpp"
#include "usr_uart.hpp"
#include "zf_common_headfile.h"

// 任务所需包含头文件
#include "Algorithm/Filters/filters.hpp"
#include "Utility/VOFAplus/VOFAplus.hpp"
#include "zf_device_oled.h"
#include "zf_driver_adc.h"
// 外设宏定义
#define LED1 (P19_0)
#define KEY1 (P11_0)
#define KEY2 (P11_1)
#define KEY3 (P11_2)
#define SWITCH1 (P21_5)
#define SWITCH2 (P21_6)

// 自定义类变量
USR_SYSTEM usr_sys;

// 任务专属变量
// 启动任务
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
void defaultTask(void *argument);

/**
 * @brief 用户系统初始化
 *
 */
void usrSystemInit(void) {
  // 外设初始化
  usr_sys.peripheralInit();

  // 任务初始化
  usr_sys.TaskCreate();
}

/**
 * @brief 用户外设初始化
 *
 */
void USR_SYSTEM::peripheralInit(void) {


}

/**
 * @brief 用户任务创建
 *
 */
void USR_SYSTEM::TaskCreate() {
  // 创建默认任务
  defaultTaskHandle = osThreadNew(defaultTask, NULL, &defaultTask_attributes);
}

volatile uint8_t debug_cnt = 0;
void defaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;) {
    ++debug_cnt;
    vTaskDelay(1000);
  }
  /* USER CODE END 5 */
}
