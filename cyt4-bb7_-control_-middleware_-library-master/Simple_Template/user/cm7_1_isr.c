/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: \iard:\0xFF JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\user\cm7_1_isr.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK
 *接口的第三方开源库 Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即
 *GNU通用公共许可证）的条款 即 GPL 的第3版（即
 *GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt
 *文件中 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          isr_arm_7_0
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-9      pudding            first version
 ********************************************************************************************************************/

#include "System/usr_uart.hpp"
#include "Utility/UpperMonitor/UpperMonitor.hpp"
#include "zf_common_headfile.h"
static void handleWhenIdle(uint8_t uart_id);
static bool isUartIdle(fifo_struct *pfifo);

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr() { pit_isr_flag_clear(PIT_CH0); }

void pit0_ch1_isr() {
  // 清除定时中断标志位
  pit_isr_flag_clear(PIT_CH1);
  
}

void pit0_ch2_isr() { 
  pit_isr_flag_clear(PIT_CH2); 
  // 从adc读取麦克风数值到循环数组
  // DEBUG！！！
  #ifdef ENABLE_SOUND_PROCESS
  getMicDataFromADC(mic_raw_data);
  #endif
  // DEBUG！！！
  }
// **************************** PIT中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr() {}

void gpio_1_exti_isr() {
  if (exti_flag_get(P01_0)) // 示例P1_0端口外部中断判断
  {
  }
  if (exti_flag_get(P01_1)) {
  }
}

void gpio_2_exti_isr() {
  if (exti_flag_get(P02_0)) {
  }
  if (exti_flag_get(P02_4)) {
  }
}

void gpio_3_exti_isr() {}

void gpio_4_exti_isr() {}

void gpio_5_exti_isr() {}

void gpio_6_exti_isr() {}

void gpio_7_exti_isr() {}

void gpio_8_exti_isr() {}

void gpio_9_exti_isr() {}

void gpio_10_exti_isr() {}

void gpio_11_exti_isr() {
  if (exti_flag_get(P11_0)) {
    if (gpio_get_level(P11_0)) {
      printf("key 1 is pressed!\r\n");
    }
  }
  if (exti_flag_get(P11_1)) {
    if (gpio_get_level(P11_1)) {
      printf("key 2 is pressed!\r\n");
    }
  }
  if (exti_flag_get(P11_2)) {
    if (gpio_get_level(P11_2)) {
      printf("key 3 is pressed!\r\n");
    }
  }
}

void gpio_12_exti_isr() {}

void gpio_13_exti_isr() {}

void gpio_14_exti_isr() {}

void gpio_15_exti_isr() {}

void gpio_16_exti_isr() {}

void gpio_17_exti_isr() {}

void gpio_18_exti_isr() {}

void gpio_19_exti_isr() {}

void gpio_20_exti_isr() {}

void gpio_21_exti_isr() {}

void gpio_22_exti_isr() {}

void gpio_23_exti_isr() {}
// **************************** 外部中断函数 ****************************

//// **************************** DMA中断函数 ****************************
// void dma_event_callback(void* callback_arg, cyhal_dma_event_t event)
//{
//     CY_UNUSED_PARAMETER(event);
//
//
//
//
// }
//  **************************** DMA中断函数 ****************************

// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr(void) {
  if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_0)) &
      CY_SCB_UART_RX_NOT_EMPTY) // 串口0接收中断
  {
    Cy_SCB_ClearRxInterrupt(get_scb_module(UART_0),
                            CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位

    uart0_read_byte(); // 读取一个字节到对应的串口fifo
  } else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_0)) &
             CY_SCB_UART_TX_DONE) // 串口0发送中断
  {
    Cy_SCB_ClearTxInterrupt(get_scb_module(UART_0),
                            CY_SCB_UART_TX_DONE); // 清除接收中断标志位
  }
}

void uart1_isr(void) {
  if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_1)) &
      CY_SCB_UART_RX_NOT_EMPTY) // 串口1接收中断
  {
    Cy_SCB_ClearRxInterrupt(get_scb_module(UART_1),
                            CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位
    uart1_read_byte(); // 读取一个字节到对应的串口fifo

  } else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_1)) &
             CY_SCB_UART_TX_DONE) // 串口1发送中断
  {
    Cy_SCB_ClearTxInterrupt(get_scb_module(UART_1),
                            CY_SCB_UART_TX_DONE); // 清除接收中断标志位
  }
}

void uart2_isr(void) {
  if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_2)) &
      CY_SCB_UART_RX_NOT_EMPTY) // 串口2接收中断
  {
    Cy_SCB_ClearRxInterrupt(get_scb_module(UART_2),
                            CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位
    uart2_read_byte(); // 读取一个字节到对应的串口fifo

  } else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_2)) &
             CY_SCB_UART_TX_DONE) // 串口2发送中断
  {
    Cy_SCB_ClearTxInterrupt(get_scb_module(UART_2),
                            CY_SCB_UART_TX_DONE); // 清除接收中断标志位
  }
}

void uart3_isr(void) {
  if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_3)) &
      CY_SCB_UART_RX_NOT_EMPTY) // 串口3接收中断
  {
    Cy_SCB_ClearRxInterrupt(get_scb_module(UART_3),
                            CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位
    uart3_read_byte(); // 读取一个字节到对应的串口fifo

  } else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_3)) &
             CY_SCB_UART_TX_DONE) // 串口3发送中断
  {
    Cy_SCB_ClearTxInterrupt(get_scb_module(UART_3),
                            CY_SCB_UART_TX_DONE); // 清除接收中断标志位
  }
}

void uart4_isr(void) {
  if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_4)) &
      CY_SCB_UART_RX_NOT_EMPTY) // 串口4接收中断
  {
    Cy_SCB_ClearRxInterrupt(get_scb_module(UART_4),
                            CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位
    uart4_read_byte(); // 读取一个字节到对应的串口fifo

  } else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_4)) &
             CY_SCB_UART_TX_DONE) // 串口4发送中断
  {
    Cy_SCB_ClearTxInterrupt(get_scb_module(UART_4),
                            CY_SCB_UART_TX_DONE); // 清除接收中断标志位
  }
}
// **************************** 串口中断函数 ****************************
/**
 * @brief 串口空闲状态判断
 *
 * @param pfifo
 * @return true
 * @return false
 */
static bool isUartIdle(fifo_struct *pfifo) {
  if (pfifo->head == pfifo->pre_head) {
    if (pfifo->head == 0) {
      // 如果没有数据，退出
      return false;
    } else
      // 如果空闲了
      return true;
  }

  if (pfifo->head < pfifo->pre_head) {
    // 假如超过了数组长度
    return true;
  }

  // 如果在接收状态，也就是head变了，则更新pre_head
  pfifo->pre_head = pfifo->head;
  return false;
}

/**
 * @brief debug串口空闲中断处理函数
 *
 * @param pfifo 要被判断空闲状态的串口fifo
 * @param handle_func 空闲状态下要执行的处理函数
 */
static void handleWhenIdle(uint8_t uart_id) {
  if (isUartIdle(puart_fifo_s[uart_id])) {
    // 如果空闲，执行处理函数
    if (uart_handle_callback_s[uart_id] != NULL) {
      uart_handle_callback_s[uart_id]((uint8_t *)puart_fifo_s[uart_id]->buffer,
                                      puart_fifo_s[uart_id]->max -
                                          puart_fifo_s[uart_id]->size);
    }
    // clear fifo
    fifo_clear(puart_fifo_s[uart_id]);
  }
}