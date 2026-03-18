/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-04
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-05
 * @FilePath: UpperMonitor.cpp
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "Utility/UpperMonitor/UpperMonitor.hpp"

/***********************上位机调参使用***********************/
/* 在这里extern需要使用的变量和需要包含的头文件 */
uint8_t tst_monitor_data0 = 0;
uint8_t tst_monitor_data10 = 10;
uint8_t tst_monitor_data100 = 100;

/***********************上位机调参使用***********************/

/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#define Sent_Data_Num 9
uint8_t On_Off_flag;
type_change_t Sent_data_type[Sent_Data_Num + 2]; // 传输数据共用体
uint8_t USART0_Sent_Choose_Data[9] = {0, 1, 2, 3, 4,
                                      5, 6, 7, 8}; // 串口选择发送的数据标志

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void UpperMonitor_Sent_Set(float *data);
void UpperMonitor_Sent_Choose(float *data);
float PARAMETER_Change_float(uint8_t *PARAMETER);
void PARAMETER_MODIFICATION(uint8_t *PARAMETER);
void MODE_MODIFICATION(uint8_t *PARAMETER);
/* function prototypes -------------------------------------------------------*/

/**
 * @brief 上位机发送主任务
 *
 * @param arg
 */
void upperMonitorTask(void *arg) {
  const TickType_t xFreq = 4;
  TickType_t xLastWake = xTaskGetTickCount();

  // 客制化准备

  for (;;) {
    // 阻塞以进行系统调度
    vTaskDelayUntil(&xLastWake, xFreq);

    // 数据处理
    tst_monitor_data10 += 20;
    tst_monitor_data100 += 20;

    // 上位机发送
    // Sent_Contorl(UART_1);
  }
}

/**
 * @brief  发送数据函数
 * @param  None
 * @return None
 */
void Sent_Contorl(uart_index_enum uartn) {
  float temp[Sent_Data_Num];
  UpperMonitor_Sent_Choose(temp); // 选择要传输的数据
  UpperMonitor_Sent_Set(temp);    // 发送数据转换格式
  uart_write_buffer(uartn, (uint8_t *)Sent_data_type + 3, 39);
}

/**
 * @brief  串口发送参数选择函数(要观看的曲线),用于选择需要传输的数据
 * @param  data:需要传输的数组指针
 * @return None.
 *  case 0:
       data[i] = infantry.gimbal.yawMotor.Out;
    break;
 */
void UpperMonitor_Sent_Choose(float *data) {
  uint8_t i;
  for (i = 0; i < Sent_Data_Num; i++) {
    switch (USART0_Sent_Choose_Data[i]) {
      /* 以下部分用于观察参数曲线 */
    case 0:
      data[i] = tst_monitor_data0;
      break;
    case 1:
      data[i] = tst_monitor_data10;
      break;
    case 2:
      data[i] = tst_monitor_data100;
      break;
    default:
      break;
      /* 以上部分用于观察参数曲线 */
    }
  }
}

/**
 * @brief  上位机参数修改函数（要调的参数）
 * @param  PARAMETER：指令数组指针，用于读取指令
 * @return None.
 *   case 0x00:
      steer_pid.Kp = PARAMETER_Change_float(PARAMETER + 1);
      break;
 */
void PARAMETER_MODIFICATION(uint8_t *PARAMETER) {
  switch (PARAMETER[0]) {
  /* 以下部分用于修改参数内容 */
  case 0x00:
    tst_monitor_data0 = (uint8_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  default:
    break;
    /* 以上部分用于修改参数内容 */
  }
}

/**
 * @brief  串口发送设置函数,用于设置DMA串口的数据
 * @param  data:需要传输的数组指针
 * @return None.
 */
void UpperMonitor_Sent_Set(float *data) {
  uint8_t j;
  Sent_data_type[0].change_u8[3] = 0xfd;  // 发送数据头
  for (j = 1; j < Sent_Data_Num + 1; j++) // 数据体
  {
    Sent_data_type[j].change_float = data[j - 1];
  }
  Sent_data_type[Sent_Data_Num + 1].change_u8[0] = Sent_Data_Num; // 数据尾
  Sent_data_type[Sent_Data_Num + 1].change_u8[1] = 0xfe;          // 校验位
}

/**
 * @brief  上位机参数转变成浮点数函数
 * @param  PARAMETER：指令数组指针，用于读取指令
 * @return None.
 */
float PARAMETER_Change_float(uint8_t *PARAMETER) {
  uint8_t i = 0;
  union type_change Sent_data_temp; // 传输数据共用体
  for (i = 0; i < 4; i++) {
    Sent_data_temp.change_u8[i] = PARAMETER[3 - i]; // 转换成共用体数据类型
  }
  return Sent_data_temp.change_float; // 返回共用体转化后的数据
}

/**
 * @brief  上位机参数修改函数
 * @param  PARAMETER： 指令数组指针，用于读取指令
 * @return None.
 */
void MODE_MODIFICATION(uint8_t *PARAMETER) {
  switch (PARAMETER[0]) {
  case 0x00:
    USART0_Sent_Choose_Data[0] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x01:
    USART0_Sent_Choose_Data[1] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x02:
    USART0_Sent_Choose_Data[2] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x03:
    USART0_Sent_Choose_Data[3] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x04:
    USART0_Sent_Choose_Data[4] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x05:
    USART0_Sent_Choose_Data[5] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x06:
    USART0_Sent_Choose_Data[6] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x07:
    USART0_Sent_Choose_Data[7] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x08:
    USART0_Sent_Choose_Data[8] =
        (uint64_t)PARAMETER_Change_float(PARAMETER + 1);
    break;
  default:
    break;
  }
}

uint8_t USART_Interrupt_flag = 0xff; // 串口中断标志位
uint8_t USART_Get_Num_Flag = 0;      // 串口数据获取标志
uint8_t USART_receive[5] = {0};      // 串口接收缓存数组
int len = 0;
/**
* @brief  串口接收解析函数
* @param  data_buf：接收到的数据指针
          length  ：数据长度
* @return No meaning.
*/
uint32_t RecHandle(uint8_t *data_buf, uint32_t length) {
  uint8_t Temp = 0;
  len = length;
  for (int i = 0; i < length; i++) {
    Temp = data_buf[i];
    switch (USART_Interrupt_flag) {
    case 0xff: // USART0_Interrupt_flag==0xff时为等待模式，等待指令头输入
      if (Temp == 0xf0) // 指令头，识别上位机发送了修改指令
        USART_Interrupt_flag = 0xf0; // 下一个指令将进入模式选择模式
      break;
    case 0xf0:          // 进入模式选择
      if (Temp == 0x00) // 修改参数
      {
        USART_Interrupt_flag = 0x00; // 进入参数修改模式
        USART_Get_Num_Flag = 0;
      } else if (Temp == 0x01) // 修改模式
      {
        USART_Interrupt_flag = 0x01; // 进入模式修改模式
        USART_Get_Num_Flag = 0;
      } else if (Temp == 0x02) {
        USART_Interrupt_flag = 0x02; // 进入模式修改模式
        USART_Get_Num_Flag = 0;
      }
      break;
    case 0x00:
      USART_receive[USART_Get_Num_Flag] = Temp;
      USART_Get_Num_Flag++;
      if (USART_Get_Num_Flag > 4) // 参数处理
      {
        PARAMETER_MODIFICATION(USART_receive);
        USART_Interrupt_flag = 0xff; // 回到等待模式
      }
      break;
    case 0x01:
      USART_receive[USART_Get_Num_Flag] = Temp;
      USART_Get_Num_Flag++;
      if (USART_Get_Num_Flag > 4) // 参数处理
      {
        MODE_MODIFICATION(USART_receive);
        USART_Interrupt_flag = 0xff; // 回到等待模式
      }
      break;
    case 0x02:
      USART_receive[USART_Get_Num_Flag] = Temp;
      USART_Get_Num_Flag++;
      if (USART_Get_Num_Flag > 4) // 参数处理
      {
        if (USART_receive[0] == 0x0a) {
          for (int j = 1; j < 5; j++) {
            if (USART_receive[j] != 0x0a)
              USART_Interrupt_flag = 0xff; // 回到等待模式
          }
          if (USART_Interrupt_flag == 0x02) {
            On_Off_flag = 1;
            USART_Interrupt_flag = 0xff; // 回到等待模式
          }
        } else if (USART_receive[0] == 0xb0) {
          for (int j = 1; j < 5; j++) {
            if (USART_receive[j] != 0xb0)
              USART_Interrupt_flag = 0xff; // 回到等待模式
          }
          if (USART_Interrupt_flag == 0x02) {
            On_Off_flag = 0;
            USART_Interrupt_flag = 0xff; // 回到等待模式
          }
        } else
          USART_Interrupt_flag = 0xff; // 回到等待模式
      }
      break;

    default:
      USART_Interrupt_flag = 0xff; // 回到等待模式
      break;
    }
  }
  return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/