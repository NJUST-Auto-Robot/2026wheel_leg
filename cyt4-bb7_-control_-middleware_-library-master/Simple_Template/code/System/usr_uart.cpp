/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: usr_uart.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/**
 * @file usr_uart.cpp
 * @author Jae Frank (thissfk@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "System/usr_uart.hpp"
#include "zf_common_headfile.h"
#include "zf_driver_uart.h"

/**
 * @brief 测试用的中断回调函数，用于应答
 *
 * @param buf
 * @param len
 * @return uint32_t
 */
uint32_t uart_callback(uint8_t *buf, uint32_t len) {
  printf("REC: %s\tLEN: %u\r\n", buf, len);
  return 1;
}

// 串口的fifo
fifo_struct uart1_fifo, uart2_fifo, uart3_fifo, uart4_fifo;
// 串口fifo集合
fifo_struct *puart_fifo_s[5] = {&debug_uart_fifo, &uart1_fifo, &uart2_fifo,
                                &uart3_fifo, &uart4_fifo};
// 串口fifo的buffer
uint8_t uart0_buffer[UART0_BUFSIZE], uart1_buffer[UART1_BUFSIZE],
        uart2_buffer[UART2_BUFSIZE], uart3_buffer[UART3_BUFSIZE],
        uart4_buffer[UART4_BUFSIZE];
// 串口fifo的buffer集合
uint8_t *uart_buffer_s[5] = {uart0_buffer, uart1_buffer, uart2_buffer,
                             uart3_buffer, uart4_buffer};
// 串口fifo的buffer的长度集合
uint32_t uart_buffer_len_s[5] = {UART0_BUFSIZE, UART1_BUFSIZE, UART2_BUFSIZE,
                                 UART3_BUFSIZE, UART4_BUFSIZE};
 
// 串口中断回调函数集合
uart_handle_callback_t uart_handle_callback_s[5] = {
    (uart_handle_callback_t)debug_interrupt_handler,    // 串口0中断回调函数
    NULL,    // 串口1中断回调函数
    (uart_handle_callback_t)process_uart2_fifo,    // 串口2中断回调函数
    NULL,    // 串口3中断回调函数
    NULL     // 串口4中断回调函数
};

template <uint8_t uart_id> bool uartSend(uint8_t *pData, uint16_t Size) {
  uart_write_buffer((uart_index_enum)uart_id, pData, Size);

  // 仅为了消除警告
  return true;
}
Send_DMA_Fun_t uart0_Send = uartSend<0>;
Send_DMA_Fun_t uart1_Send = uartSend<1>;
Send_DMA_Fun_t uart2_Send = uartSend<2>;
Send_DMA_Fun_t uart3_Send = uartSend<3>;
Send_DMA_Fun_t uart4_Send = uartSend<4>;
// 发送函数的合集
Send_DMA_Fun_t UartSendArray[5] = {uart0_Send, uart1_Send, uart2_Send,
                                  uart3_Send, uart4_Send};

/**
 * @brief 初始化单个串口，注册fifo，并打开接收中断
 *
 * @tparam uart_id
 */

/**
 * @brief 初始化单个串口，注册fifo，并打开接收中断
 *
 * @param uart_id
 * @param baudrate
 * @param is_sbus 是否为sbus（时钟分频会不同）
 * @param data_bit_num 数据位个数
 * @param verification_type
 * 校验方式[cy_scb_uart.h->group_scb_uart_macro_parity]
 * @param stop_bit_num 停止位数[cy_scb_uart.h->group_scb_uart_macro_stop_bits]
 * uartSingleInit(0, 115200, false, 8, CY_SCB_UART_PARITY_NONE,
 * CY_SCB_UART_STOP_BITS_1);
 *
 * uartSingleInit(2, 100000, true, 9,
 * CY_SCB_UART_PARITY_EVEN, CY_SCB_UART_STOP_BITS_2);
 */
void uartSingleInit(uint8_t uart_id, uint32_t baudrate, bool is_sbus,
                    uint8_t data_bit_num, uint8_t verification_type,
                    uint8_t stop_bit_num) {
  // 初始化引脚和波特率
  if (is_sbus)
    uart_sbus_init((uart_index_enum)uart_id, baudrate,
                   (uart_tx_pin_enum)uart_id, (uart_rx_pin_enum)uart_id,
                   data_bit_num, verification_type, stop_bit_num);
  else
    uart_init((uart_index_enum)uart_id, baudrate, (uart_tx_pin_enum)uart_id,
              (uart_rx_pin_enum)uart_id, data_bit_num, verification_type,
              stop_bit_num);

  // 初始化串口的fifo
  fifo_init(puart_fifo_s[uart_id], FIFO_DATA_8BIT, uart_buffer_s[uart_id],
            uart_buffer_len_s[uart_id]);
  // 使能对应串口接收中断
  uart_rx_interrupt((uart_index_enum)uart_id, 1);
}

/**
 * @brief 初始化每个串口，注册fifo，并打开接收中断
 *
 */
void usrUartInit(void) {
  // 串口1：8 N 1
  uartSingleInit(1, 115200, false, 8, CY_SCB_UART_PARITY_NONE,
                 CY_SCB_UART_STOP_BITS_1);
  // 串口2：8 N 1
  uartSingleInit(1, 115200, false, 8, CY_SCB_UART_PARITY_EVEN,
                 CY_SCB_UART_STOP_BITS_2);
  // 串口3：8 N 1
  uartSingleInit(3, 115200, false, 8, CY_SCB_UART_PARITY_NONE,
                 CY_SCB_UART_STOP_BITS_1);
  // 串口4：8 N 1
  uartSingleInit(4, 115200, false, 8, CY_SCB_UART_PARITY_NONE,
                 CY_SCB_UART_STOP_BITS_1);
}

/**
 * @brief 在中断时，读一个字节进入fifo
 *
 * @tparam uart_id
 */
template <uint8_t uart_id> void uart_read_byte(void) {
  uint8_t rec_sta = 0;
  if (puart_fifo_s[uart_id]->init_state == true) {
    uint8_t get_byte;
    rec_sta = uart_read_self_quit((uart_index_enum)uart_id, &get_byte, 1000);
    if (rec_sta) {
      // 如果接收成功
      fifo_write_buffer(puart_fifo_s[uart_id], &get_byte, 1); // 存入 FIFO
    }
  }
}

// 初始化一些中断读字节的函数指针以供isr.c文件使用
void (*uart0_read_byte)(void) = uart_read_byte<0>;
void (*uart1_read_byte)(void) = uart_read_byte<1>;
void (*uart2_read_byte)(void) = uart_read_byte<2>;
void (*uart3_read_byte)(void) = uart_read_byte<3>;
void (*uart4_read_byte)(void) = uart_read_byte<4>;