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

uint8_t ipc_init_over = 0;
uint32 send_data_test = 0;
// 定义数据接收回调函数 如果另外一个核心发送信息 此核心会触发中断并且可以在回调函数读取数据
void my_ipc_callback(uint32 receive_data)
{
    ipc_init_over = 1;
    printf("receive M7_1 data:%d\r\n", receive_data);        // 将接收到的数据打印到串口     
}

// **************************** 代码区域 ****************************

int main(void) {
  clock_init(SYSTEM_CLOCK_250M);  // 时钟配置及系统初始化<务必保留>
  debug_init();                   // 调试串口信息初始化
  
  SCB_DisableDCache(); // 关闭DCache
  
  ipc_communicate_init(IPC_PORT_1, my_ipc_callback);          // 初始化IPC模块 选择端口1 填写中断回调函数
    
  usrSystemInit();                // 用户系统初始化 包括外设和任务创建
  osKernelInitialize();           // 初始化FreeRTOS内核
  osKernelStart();                // 开启FreeRTOS内核调度
  
  while (true) {
    if(ipc_init_over == 1)
    {
      ipc_init_over = 2;
      
      
    }
    /*假如FreeRTOS调度成功，那么不会运行这里面的代码*/
  }
}

// **************************** 任务将在system.cpp里运行 ****************************//
