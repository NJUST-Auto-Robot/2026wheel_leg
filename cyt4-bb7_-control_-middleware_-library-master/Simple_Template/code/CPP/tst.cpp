/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-04
 * @FilePath: \iard:\0xFF JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\CPP\tst.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "CPP/tst.h"
#include "zf_driver_gpio.h"

// 任务
osThreadId_t tstTaskHandle; // 1.修改句柄名称
const osThreadAttr_t tstTask_attributes = {
    // 2.修改参数集合名称
    .name = "tstTask",                          // 3.修改字符名称
    .stack_size = 128 * 4,                      // 4.修改大小
    .priority = (osPriority_t)osPriorityNormal, // 5.修改优先级
};
void tstTask(void *argument); // 6.修改任务函数名称
// 7. 在启动函数中创建任务
// 8. 实现任务内容

tst_class tstClass;

/**
 * @brief 创建tst任务
 *
 */
void tst_func(void)
{
    tstClass.tst_class_func();
}

void tstTask(void *argument)
{
    const TickType_t xFreq = 1;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t cnt = 0;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFreq);
        gpio_toggle_level(P19_0);
        vTaskDelay(250);
        cnt++;
    }
}

void tst_class::tst_class_func(void)
{
    this->tst_class_flag = 1;
    tstTaskHandle = osThreadNew(tstTask, NULL, &tstTask_attributes); // 创建cpp_tst任务
}