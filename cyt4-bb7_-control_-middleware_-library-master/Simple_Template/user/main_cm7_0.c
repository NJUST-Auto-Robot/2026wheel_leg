/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-05
 * @FilePath: main_cm7_0.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *            佛祖保佑     永不宕机     永无BUG
 */

#include "cmsis_os.h"
#include "System/usr_uart.hpp"
#include "System/usr_system.hpp"
#include "zf_common_headfile.h"

// 定义数据接收回调函数 如果另外一个核心发送信息 此核心会触发中断并且可以在回调函数读取数据

// **************************** 代码区域 ****************************
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P00_0


int main(void) {
  clock_init(SYSTEM_CLOCK_250M);  // 时钟配置及系统初始化<务必保留>
  debug_init();                   // 调试串口信息初始化
  
  //uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN, 8, NULL, 1);             // 初始化串口
  //uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断

  usrSystemInit();                // 用户系统初始化 包括外设和任务创建
  osKernelInitialize();           // 初始化FreeRTOS内核
  osKernelStart();                // 开启FreeRTOS内核调 
  
  while (true) {
    /*假如FreeRTOS调度成功，那么不会运行这里面的代码*/
  }
}
