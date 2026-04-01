/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-06
 * @FilePath: ps2.h
 * JaeFrank\ForFun\SmartCar\SchoolGame\cyt4887_-free-rtos_-template\Simple_Template\code\PS2\ps2.h
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#ifndef __PSTWO_H__
#define __PSTWO_H__
#include "zf_driver_gpio.h"
#include <stdint.h>

/**
 * @brief PS2引脚定义
 *
 */
// DAT端口
#define PS2_DI P07_0
// CMD端口
#define PS2_DO P07_1
// CS端口
#define PS2_CS P07_2
// CLK端口
#define PS2_CLK P07_3

typedef struct _PS2_data_t {
  bool is_ps2_ok;
  uint8_t PS_Key;
  uint8_t PS_LX;
  uint8_t PS_LY;
  uint8_t PS_RX;
  uint8_t PS_RY;
  float norm_PS_LX;
  float norm_PS_LY;
  float norm_PS_RX;
  float norm_PS_RY;
} ps2_data_t;

typedef enum {
  mode_key = 0,
  select_key = 1,
  left_button_key = 2,
  right_button_key = 3,
  start_key = 4,
  up_arrow_key = 5,
  right_arrow_key = 6,
  down_arrow_key = 7,
  left_arrow_key = 8,
  left_down_pad = 9,
  right_down_pad,
  left_up_pad,
  right_up_pad,
  y_key,
  b_key,
  a_key,
  x_key,
} ps2_key_map_t;

#define DI gpio_get_level(PS2_DI)        // 读DI电平
#define DO_H gpio_set_level(PS2_DO, 1)   // 命令位高
#define DO_L gpio_set_level(PS2_DO, 0)   // 命令位低
#define CS_H gpio_set_level(PS2_CS, 1)   // CS拉高
#define CS_L gpio_set_level(PS2_CS, 0)   // CS拉低
#define CLK_H gpio_set_level(PS2_CLK, 1) // 时钟拉高
#define CLK_L gpio_set_level(PS2_CLK, 0) // 时钟拉低

#define PSB_SELECT 1
#define PSB_L3 2
#define PSB_R3 3
#define PSB_START 4
#define PSB_PAD_UP 5
#define PSB_PAD_RIGHT 6
#define PSB_PAD_DOWN 7
#define PSB_PAD_LEFT 8
#define PSB_L2 9
#define PSB_R2 10
#define PSB_L1 11
#define PSB_R1 12
#define PSB_GREEN 13
#define PSB_RED 14
#define PSB_BLUE 15
#define PSB_PINK 16
#define PSB_TRIANGLE 13
#define PSB_CIRCLE 14
#define PSB_CROSS 15
#define PSB_SQUARE 16

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

extern ps2_data_t ps2_data;

// These are stick values
#define PSS_RX 5 // 右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
void ps2Task(void *argument);
void PS2_Cmd(uint8_t Cmd);
uint8_t PS2_RedLight(void);
void PS2_ReadData(void);
uint8_t PS2_DataKey(void);
void PS2_ClearData(void);
void PS2_Vibration(uint8_t motor1, uint8_t motor2);
void PS2_ShortPoll(void);
void PS2_EnterConfing(void);
void PS2_TurnOnAnalogMode(void);
void PS2_VibrationMode(void);
uint8_t PS2_AnologData(uint8_t button);
void PS2_ExitConfing(void);
void PS2_SetInit(void);
#ifdef __cplusplus
}
#endif // __cplusplus
#endif
