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
#include "AAARobot/AAARobot.hpp"
#include "usr_system.hpp"
#include "usr_uart.hpp"
#include "zf_common_headfile.h"

// 任务所需包含头文件
#include "Algorithm/Filters/filters.hpp"
#include "Module/SoundProcess/sound_process.hpp"
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

// 上位机任务
// osThreadId_t upperMonitorTaskHandle; // 1.修改句柄名称
// const osThreadAttr_t upperMonitorTask_attributes = {
//     // 2.修改参数集合名称
//     .name = "upperMonitorTask",                 // 3.修改字符名称
//     .stack_size = 128 * 4,                      // 4.修改大小
//     .priority = (osPriority_t)osPriorityNormal, // 5.修改优先级
// };
// 6.修改任务函数名称
// 7. 在启动函数中创建任务
// 8. 实现任务内容
#if ADC_DATA_VIEW
// vofaTask
osThreadId_t vofaTaskHandle; // 1.修改句柄名称
const osThreadAttr_t vofaTask_attributes = {
    // 2.修改参数集合名称
    .name = "vofaTask",                         // 3.修改字符名称
    .stack_size = 128 * 4,                      // 4.修改大小
    .priority = (osPriority_t)osPriorityNormal, // 5.修改优先级
};
// 6.修改任务函数名称
// 7. 在启动函数中创建任务
// 8. 实现任务内容
#endif

/**
 * @brief 硬件中断处理函数
 *
 */
void Cy_SysLib_ProcessingFault(void) {
  interrupt_global_disable(); // 关闭所有中断
  NVIC_SystemReset();         // 复位
}

/**
 * @brief IPC通信回调函数
 *
 * @param data
 */
void my_ipc_callback(uint32_t data) {
  //将 data 转换为 void 类型，向编译器表明我们有意忽略这个参数，从而消除警告
  (void)data;
}

// uint16_t tst_idx = 0;
// int16_t input[1000];
// int16_t output[1000];

/**
 * @brief 用户系统初始化
 *
 */
void usrSystemInit(void) {
  // 外设初始化
  usr_sys.peripheralInit();

  // 任务初始化
  usr_sys.TaskCreate();

  // // 初始化输入信号
  // for (int i = 0; i < 1000; i++) {
  //   input[i] = static_cast<int16_t>(1000 * sinf(2 * PI * 10 * i / 1000) +
  //                                   1000 * sinf(2 * PI * 10 * i / 1000));
  // }
  // // 预分配缓冲区
  // float32_t input_buffer[1000];
  // float32_t output_buffer[1000];

  // // 创建带通滤波器实例
  // BandPassFilter<int16_t> filter(1000, 5, 100, input_buffer, output_buffer);

  // // 处理信号
  // filter.process(input, output, 1000);
}

/**
 * @brief 用户外设初始化
 *
 */
void USR_SYSTEM::peripheralInit(void) {
  // 关闭DCache
  SCB_DisableDCache();

  // 初始化IPC模块 选择端口2 填写中断回调函数
  ipc_communicate_init(IPC_PORT_2, my_ipc_callback);

#if ADC_DATA_VIEW
  // 串口0：8 N 1
  uartSingleInit(0, 115200, false, 8, CY_SCB_UART_PARITY_NONE,
                 CY_SCB_UART_STOP_BITS_1);
#endif
#ifdef ENABLE_SOUND_PROCESS
  // ADC初始化
  for (uint8_t i = 0; i < MicPos_TotalNum; i++) {
    adc_init(sound_adc_ch_s[i], ADC_12BIT);
  }
#endif

  // adc采集的定时器-10KHz (0.1ms = 100us)
  pit_us_init(PIT_CH2, 100);
}

/**
 * @brief 用户任务创建
 *
 */
void USR_SYSTEM::TaskCreate() {

  // 创建默认任务
  defaultTaskHandle = osThreadNew(defaultTask, NULL, &defaultTask_attributes);

  // 创建upperMonitor任务
  // upperMonitorTaskHandle =
  //     osThreadNew(upperMonitorTask, NULL, &upperMonitorTask_attributes);

  // 创建vofa任务
  // vofaTaskHandle = osThreadNew(vofaTask, NULL, &vofaTask_attributes);

  // 创建声音结算任务
  // soundProcessTaskHandle =
  //     osThreadNew(soundProcessTask, NULL, &soundProcessTask_attributes);
}

volatile uint8_t debug_cnt = 0;
void defaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  //  gpio_init(P19_0, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 初始化 LED1
  //  输出 默认高电平 推挽输出模式

  /* Infinite loop */
  for (;;) {
    ++debug_cnt;
    vTaskDelay(1000);
  }
  /* USER CODE END 5 */
}
