/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: usr_system.cpp
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
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system.cpp
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\System\usr_system.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "System/usr_system.hpp"
#include "System/usr_uart.hpp"
#include "zf_common_headfile.h"
// 任务所需包含头文件
#include "algorithm/mahony/mahony.h"
// #include "Drivers/Key/key.hpp"
// #include "Drivers/OLED/drv-u8g2.hpp"
// #include "Module/FS_I6X/FS_I6X.hpp"
// #include "Module/GUI/GUI.hpp"
// #include "Module/IPC/ipc.hpp"
// #include "Module/MPU6050/mpu6050.h"
#include "Module/IMU660RB/imu_data.h"
// #include "Module/MotorDrive/MotorDrive.hpp"
// #include "Module/PS2/ps2.h"
// #include "Module/RefereSystem/RefereSystem.hpp"
// #include "Module/Servo/servo.h"
// #include "Module/UBX/ubx_decoder.h"
// #include "Utility/UpperMonitor/UpperMonitor.hpp"
// #include "Utility/VOFAplus/VOFAplus.hpp"
// #include "zf_device_oled.h"

// extern uint8_t tst_monitor_data0;
// extern uint8_t tst_monitor_data10;
// extern uint8_t tst_monitor_data100;

// 外设宏定义
#define TEST_LED (P19_0)
// #define KEY1 (P11_0)
// #define KEY2 (P11_1)
// #define KEY3 (P11_2)
// #define SWITCH1 (P21_5)
// #define SWITCH2 (P21_6)

// 自定义类变量
USR_SYSTEM usr_sys;

// 任务专属变量
// 启动任务
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
void defaultTask(void *argument);

// 启动任务
osThreadId_t IMU660RBTaskHandle;
const osThreadAttr_t IMU660RBTask_attributes = {
    .name = "IMU660RBTask ",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
void IMU660RBTask(void *argument);
// key任务
// osThreadId_t keyTaskHandle;
// const osThreadAttr_t keyTask_attributes = {
//     .name = "keyTask",
//     .stack_size = 64,
//     .priority = (osPriority_t)osPriorityNormal,
// };
// void keyTask(void *argument);

// void mpu6050Task(void *argument); // 6.修改任务函数名称
uint16 delay_time = 0;
uint8 led_state = 0;

// 显示屏任务
// osThreadId_t oledTaskHandle; // 1.修改句柄名称
// const osThreadAttr_t oledTask_attributes = {
//     // 2.修改参数集合名称
//     .name = "oledTask",                         // 3.修改字符名称
//     .stack_size = 128 * 8,                      // 4.修改大小
//     .priority = (osPriority_t)osPriorityNormal, // 5.修改优先级
// };
// void oledTask(void *argument); // 6.修改任务函数名称

// 遥控器任务
// osThreadId_t remoteCtrlUnitTaskHandle; // 1.修改句柄名称
// const osThreadAttr_t remoteCtrlUnitTask_attributes = {
//     // 2.修改参数集合名称
//     .name = "remoteCtrlUnitTask",                 // 3.修改字符名称
//     .stack_size = 128 * 4,                        // 4.修改大小
//     .priority = (osPriority_t)osPriorityRealtime, // 5.修改优先级
// };
// 6.修改任务函数名称
// 7. 在默认任务中创建任务
// 8. 实现任务内容

// 电机驱动任务
// osThreadId_t motorTaskHandle; // 1.修改句柄名称
// const osThreadAttr_t motorTask_attributes = {
//     // 2.修改参数集合名称
//     .name = "drvTask",                            // 3.修改字符名称
//     .stack_size = 128 * 4,                        // 4.修改大小
//     .priority = (osPriority_t)osPriorityRealtime, // 5.修改优先级
// };
// 7. 在启动函数中创建任务
// 8. 实现任务内容

// 舵机转向任务
// float servo_set_ang = 0;
// osThreadId_t servoTaskHandle; // 1.修改句柄名称
// const osThreadAttr_t servoTask_attributes = {
//     // 2.修改参数集合名称
//     .name = "servoTask",                      // 3.修改字符名称
//     .stack_size = 128 * 4,                    // 4.修改大小
//     .priority = (osPriority_t)osPriorityHigh, // 5.修改优先级
// };
// 6.修改任务函数名称
// 7. 在启动函数中创建任务
// 8. 实现任务内容

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

// vofaTask
// osThreadId_t vofaTaskHandle; // 1.修改句柄名称
// const osThreadAttr_t vofaTask_attributes = {
//     // 2.修改参数集合名称
//     .name = "vofaTask",                         // 3.修改字符名称
//     .stack_size = 128 * 4,                      // 4.修改大小
//     .priority = (osPriority_t)osPriorityNormal, // 5.修改优先级
// };
// 6.修改任务函数名称
// 7. 在启动函数中创建任务
// 8. 实现任务内容

/**
 * @brief 硬件中断处理函数
 *
 */
// void Cy_SysLib_ProcessingFault(void) {
//   interrupt_global_disable(); // 关闭所有中断
//   NVIC_SystemReset();         // 复位
// }

/**
 * @brief 用户系统初始化
 *
 */
void usrSystemInit(void) {
  
  // 外设初始化
  usr_sys.peripheralInit();

  // 任务初始化
  usr_sys.TaskCreate();
  // 车子实例初始化
  // car.init();
}

/**
 * @brief 用户外设初始化
 *
 */
void USR_SYSTEM::peripheralInit(void) {
  // 关闭DCache
  // SCB_DisableDCache();

  // 初始化IPC模块 选择端口1 填写中断回调函数
  // ipc_communicate_init(IPC_PORT_1, my_ipc_callback);

  // 初始化定时器，用于pid
  // timer_init(TC_TIME2_CH1, TIMER_US);

  // 开启定时器，用于pid
  // timer_start(TC_TIME2_CH1);

  // 初始化所有串口以及fifo，并使能接收中断(除了串口2)
  usrUartInit();

  // 设置周期中断1ms，用于串口空闲中断判断
  pit_us_init(PIT_CH0, 1000);

  // 初始化 TEST_LED 输出 默认高电平 推挽输出模式
  gpio_init(TEST_LED, GPO, GPIO_LOW, GPO_PUSH_PULL);

  // exti_init(P11_0, EXTI_TRIGGER_RISING); // 初始化按键1
  // exti_init(P11_1, EXTI_TRIGGER_RISING); // 初始化按键2
  // exti_init(P11_2, EXTI_TRIGGER_RISING); // 初始化按键3
  // U8G2PeripheralInit();                  // 初始化OLED显示屏

  // 初始化IMU660RB
  imu_init(&imu660rb);                       // 初始化IMU实例
  
}   

/**
 * @brief 用户任务创建
 *
 */
void USR_SYSTEM::TaskCreate() {

  // 创建默认任务
  defaultTaskHandle = osThreadNew(defaultTask, NULL, &defaultTask_attributes);

  // 创建IMU惯导任务
  IMU660RBTaskHandle = osThreadNew(IMU660RBTask, NULL, &IMU660RBTask_attributes);

  // 创建北斗导航任务
  // keyTaskHandle = osThreadNew(keyTask, NULL, &keyTask_attributes);

  // 创建显示任务
  // oledTaskHandle = osThreadNew(oledTask, NULL, &oledTask_attributes);

#if USE_FS_I6X
  // 创建FS_I6X任务
  remoteCtrlUnitTaskHandle =
      osThreadNew(tskFS_I6X, NULL, &remoteCtrlUnitTask_attributes);
#else
  // 创建ps2任务
  // remoteCtrlUnitTaskHandle =
  //     osThreadNew(ps2Task, NULL, &remoteCtrlUnitTask_attributes);
#endif

  // 创建电机驱动任务
  // motorTaskHandle = osThreadNew(motorTask, NULL, &motorTask_attributes);

  // 创建舵机转向任务
  // servoTaskHandle = osThreadNew(servoTask, NULL, &servoTask_attributes);

// 创建upperMonitor任务
// upperMonitorTaskHandle =
//     osThreadNew(upperMonitorTask, NULL, &upperMonitorTask_attributes);

// 创建vofa任务
#if !ADC_DATA_VIEW
  // vofaTaskHandle = osThreadNew(vofaTask, NULL, &vofaTask_attributes);
#endif
}

/**
 * @brief 启动默认任务
 *
 * @param argument 传参
 */
void defaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  { 
    gpio_toggle_level(P19_0);
    
    vTaskDelay(500);
  /* USER CODE END 5 */
  }
}

/**
 * @brief 启动IMU660RB任务
 *
 * @param argument 传参
 */
void IMU660RBTask(void *argument) {
  /* USER CODE BEGIN 5 */
  while(1)
  { 
     if(imu660rb.imu_data_ready)
     {  
        imu_mahony_update(&imu660rb.raw_data, imu660rb.dt, &imu660rb.angles);
        if(imu660rb.imu_data_true >= 0) {
          imu_data_check(&imu660rb);           // 检查数据有效性
        }  
        if(imu660rb.imu_data_true == -1) {
          imu_cordinate_convert(&imu660rb); // 坐标系转换
          imu_zero_calibration(&imu660rb);  // 零偏校准
          imu660rb.imu_data_true = -2;      // 数据处理完成，设置状态为-2，等待发送
        }
        if(imu660rb.imu_data_true == -2) {
          imu_tx_data(&imu660rb);           // 发送IMU数据
          imu660rb.imu_data_true = -1;
        }
        imu660rb.imu_data_ready = false;    // 读取数据后，重置数据就绪标志
     }
     
    vTaskDelay(10);
  }
  /* USER CODE END 5 */
}
/**
 * @brief oled负责任务
 *
 * @param argument
 */
// void oledTask(void *argument) {
//   /* USER CODE BEGIN */
//   TickType_t xLastWakeTime;
//   const TickType_t xFrequency = 10;
//   xLastWakeTime = xTaskGetTickCount();
//   // char ch[100] = {0};

//   /* 客制化准备 */
//   oled_init();
//   // char ch[100];
//   u8g2Init(&u8g2);
//   Key_Init();

//   draw(&u8g2);
//   Menu_Init();
//   /* Infinite loop */
//   for (;;) {
//     Switch_Update();
//     Menu_Task_Run();

//     // 任务调度堵塞
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
//   /* USER CODE END */
// }
// /**
//  * @brief 北斗模块初始化及解包任务
//  *
//  * @param argument
//  */
// void keyTask(void *argument) {
//   /* USER CODE BEGIN */
//   TickType_t xLastWakeTime;
//   const TickType_t xFrequency = 10;

//   /* Infinite loop */
//   for (;;) {

//     vTaskDelay(xFrequency);
//   }
//   /* USER CODE END */
// }
