/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-06
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: usr_uart.hpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/**
 * @file usr_uart.hpp
 * @author Jae Frank (thissfk@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "zf_common_headfile.h"
#include "D:\BaiduNetdiskDownload\2026feika\code\workplace\cyt4-bb7_-control_-middleware_-library-master\Simple_Template\code\Controller\LQR.h"
#define UART0_BUFSIZE 256
#define UART1_BUFSIZE 256
#define UART2_BUFSIZE 256
#define UART3_BUFSIZE 256
#define UART4_BUFSIZE 256

typedef uint32_t (*uart_handle_callback_t)(uint8_t *buf, uint32_t len);
typedef bool (*Send_DMA_Fun_t)(uint8_t *pData, uint16_t Size);

// 串口fifo指针集合
extern fifo_struct *puart_fifo_s[];

// 串口fifo的buffer集合
extern uint8_t* uart_buffer_s[];

// 串口fifo的buffer的长度集合
extern uint32_t uart_buffer_len_s[];

// 串口中断回调函数集合
extern uart_handle_callback_t uart_handle_callback_s[5];

// 初始化一些中断读字节的函数指针以供isr.c文件使用
extern void (*uart0_read_byte)(void);
extern void (*uart1_read_byte)(void);
extern void (*uart2_read_byte)(void);
extern void (*uart3_read_byte)(void);
extern void (*uart4_read_byte)(void);

// 初始化函数指针数组，以供c调用
extern Send_DMA_Fun_t UartSendArray[5];

#ifdef __cplusplus
extern "C" {
#endif
uint32_t uart_callback(uint8_t *buf, uint32_t len);
void usrUartInit(void);
void uartSingleInit(uint8_t uart_id, uint32_t baudrate, bool is_sbus,
                    uint8_t data_bit_num, uint8_t verification_type,
                    uint8_t stop_bit_num);

#ifdef __cplusplus
}
#endif