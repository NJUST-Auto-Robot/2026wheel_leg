/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-05
 * @FilePath: ps2.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│
 *  ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\ │`~ ││
 *  │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│
 *  ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││
 *  │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│
 *  ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  ││
 *  │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│
 *  ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││
 *  │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│
 *  │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │
 *  │      └───┴─────┴───────────────────────┴─────┴───┘          │
 *  └─────────────────────────────────────────────────────────────┘
 */

#include "ps2.h"
#include "cmsis_os.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"

// void delay_us(uint16_t delay)
// {
// 	system_delay_us(delay);
// }
#define delay_us(x) system_delay_us(x)

uint16_t Handkey;
uint16_t ps2_test;

uint8_t Comd[2] = {0x01, 0x42};
uint8_t Data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t MASK[] = {
    PSB_SELECT,   PSB_L3,       PSB_R3,   PSB_START, PSB_PAD_UP, PSB_PAD_RIGHT,
    PSB_PAD_DOWN, PSB_PAD_LEFT, PSB_L2,   PSB_R2,    PSB_L1,     PSB_R1,
    PSB_GREEN,    PSB_RED,      PSB_BLUE, PSB_PINK}; // 按键值与按键名

// ps2数据
ps2_data_t ps2_data;

/**
 * @brief ps2负责任务
 *
 * @param argument
 */
void ps2Task(void *argument) {
  /* USER CODE BEGIN */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();

  /* 客制化准备 */
  PS2_SetInit();

  /* Infinite loop */
  for (;;) {
    // 任务调度堵塞
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    ps2_data.PS_Key = PS2_DataKey();
    ps2_data.PS_LX = PS2_AnologData(PSS_LX);
    ps2_data.PS_LY = PS2_AnologData(PSS_LY);
    ps2_data.PS_RX = PS2_AnologData(PSS_RX);
    ps2_data.PS_RY = PS2_AnologData(PSS_RY);

    if (!((ps2_data.PS_LX == ps2_data.PS_LY &&
           ps2_data.PS_LY == ps2_data.PS_RX &&
           ps2_data.PS_RX == ps2_data.PS_RY) &&
          (0xFF == ps2_data.PS_LX || 0x80 == ps2_data.PS_LX ||
           0x00 == ps2_data.PS_LX))) // 如果ps2数据异常
    {
      // 数据正常时才会归一化
      ps2_data.is_ps2_ok = true;
      ps2_data.norm_PS_LX = normalizeData(0, 127, 255, ps2_data.PS_LX, true);
      ps2_data.norm_PS_LY = normalizeData(0, 128, 255, ps2_data.PS_LY, true);
      ps2_data.norm_PS_RX = normalizeData(0, 127, 255, ps2_data.PS_RX, false);
      ps2_data.norm_PS_RY = normalizeData(0, 128, 255, ps2_data.PS_RY, true);

    } else {
      ps2_data.is_ps2_ok = false;
      ps2_data.norm_PS_LX = 0;
      ps2_data.norm_PS_LY = 0;
      ps2_data.norm_PS_RX = 0;
      ps2_data.norm_PS_RX = 0;
    }
  }
  /* USER CODE END */
}

/**
 * @brief 向手柄发送命令
 *
 * @param Cmd 命令
 */
void PS2_Cmd(uint8_t Cmd) {
  volatile uint16_t ref = 0x01;
  Data[1] = 0;
  for (ref = 0x01; ref < 0x0100; ref <<= 1) {
    if (ref & Cmd) {
      DO_H;
    } else
      DO_L;

    CLK_H;
    delay_us(10);
    CLK_L;
    delay_us(10);
    CLK_H;
    if (DI)
      Data[1] = ref | Data[1];
  }
  //	delay_us (16);
}

// 判断是否为红灯模式
// 返回值；0，红灯模式
//		  其他，其他模式
/**
 * @brief 判断是否为红灯模式
 *
 * @return uint8_t 0，红灯模式,其他，其他模式
 */
uint8_t PS2_RedLight(void) {
  CS_L;             // 拉低开始通讯
  PS2_Cmd(Comd[0]); // 开始命令
  PS2_Cmd(Comd[1]); // 请求数据
  CS_H;
  if (Data[1] == 0X73)
    return 0; // 0x73开红灯模式
  else
    return 1;
}

// 读取手柄数据
/**
 * @brief 读取手柄数据
 *
 */
void PS2_ReadData(void) {
  volatile uint8_t byte = 0;
  volatile uint16_t ref = 0x01;
  CS_L;
  PS2_Cmd(Comd[0]);                // 开始命令
  PS2_Cmd(Comd[1]);                // 请求数据
  for (byte = 2; byte < 9; byte++) // 开始接受数据
  {
    for (ref = 0x01; ref < 0x100; ref <<= 1) {
      CLK_H;
      //			delay_us(16);
      CLK_L;
      delay_us(10);
      CLK_H;
      // 当按下相应按键时，取或使得当ref为1时，相应的位也为1，即使得Data记录的即为它的哪个按键被按下了
      if (DI)
        Data[byte] = ref | Data[byte];
    }
    delay_us(10);
  }
  CS_H;
}

// 处理PS2读出来的数据，仅仅对于按键
/**
 * @brief 处理PS2读出来的数据，仅仅对于按键
 *
 * @return uint8_t
 */
uint8_t PS2_DataKey(void) {
  uint8_t index;

  PS2_ClearData();
  PS2_ReadData();

  Handkey = (Data[4] << 8) | Data[3]; // 这是16个按键  按下为0， 未按下为1
  for (index = 0; index < 16; index++) {
    ps2_test = Handkey & (1 << (MASK[index] - 1));
    if (ps2_test == 0)
      return index + 1;
  }
  return 0; // 没有任何按键按下
}

// 得到一个摇杆的模拟量	 范围0~256
/**
 * @brief 得到一个摇杆的模拟量
 *
 * @param button
 * @return uint8_t 范围0~255
 */
uint8_t PS2_AnologData(uint8_t button) { return Data[button]; }

// 清除数据缓冲区
/**
 * @brief 清除数据缓冲区
 *
 */
void PS2_ClearData(void) {
  uint8_t a;
  for (a = 0; a < 9; a++)
    Data[a] = 0x00;
}

/**
 * @brief 开启震动模式
 *
 * @param motor1
 * @param motor2
 */
void PS2_Vibration(uint8_t motor1, uint8_t motor2) {
  CS_L;
  delay_us(16);
  PS2_Cmd(0x01); // 开始命令
  PS2_Cmd(0x42); // 请求数据
  PS2_Cmd(0X00);
  PS2_Cmd(motor1);
  PS2_Cmd(motor2);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  delay_us(16);
}

// short poll 手柄配置初始化
/**
 * @brief 手柄配置初始化
 *
 */
void PS2_ShortPoll(void) {
  CS_L;
  delay_us(16);
  PS2_Cmd(0x01);
  PS2_Cmd(0x42);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0x00);
  CS_H;
  delay_us(16);
}

// 进入配置
/**
 * @brief 进入配置
 *
 */
void PS2_EnterConfing(void) {
  CS_L;
  delay_us(16);
  PS2_Cmd(0x01);
  PS2_Cmd(0x43);
  PS2_Cmd(0X00);
  PS2_Cmd(0x01);
  PS2_Cmd(0x00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  delay_us(16);
}

// 发送模式设置
/**
 * @brief 发送模式设置
 *
 */
void PS2_TurnOnAnalogMode(void) {
  CS_L;
  PS2_Cmd(0x01);
  PS2_Cmd(0x44);
  PS2_Cmd(
      0X00); // 指令值为0x01则可发送摇杆模拟量，即红灯模式；指令值为0x00则为绿灯模式，不发送模拟量。
  PS2_Cmd(0x01); // analog=0x01;digital=0x00  软件设置发送模式
  PS2_Cmd(0xEE); // Ox03锁存设置，即不可通过按键“MODE”设置模式。
                 // 0xEE不锁存软件设置，可通过按键“MODE”设置模式。
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  PS2_Cmd(0X00);
  CS_H;
  delay_us(16);
}

// 振动设置
/**
 * @brief 振动设置
 *
 */
void PS2_VibrationMode(void) {
  CS_L;
  delay_us(16);
  PS2_Cmd(0x01);
  PS2_Cmd(0x4D);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0X01);
  CS_H;
  delay_us(16);
}

// 完成并保存配置
/**
 * @brief 完成并保存配置
 *
 */
void PS2_ExitConfing(void) {
  CS_L;
  delay_us(16);
  PS2_Cmd(0x01);
  PS2_Cmd(0x43);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  PS2_Cmd(0x5A);
  CS_H;
  delay_us(16);
}

/**
 * @brief 初始化PS2的GPIO
 *
 */
static void PS2_initGPIO(void) {
  // DI
  gpio_init(PS2_DI, GPI, GPIO_LOW, GPI_PULL_DOWN);
  // DO
  gpio_init(PS2_DO, GPO, GPIO_LOW, GPO_PUSH_PULL);
  // CS
  gpio_init(PS2_CS, GPO, GPIO_LOW, GPO_PUSH_PULL);
  // CLK
  gpio_init(PS2_CLK, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

// 手柄配置初始化
/**
 * @brief 手柄配置初始化
 *
 */
void PS2_SetInit(void) {
  ps2_data.norm_PS_LX = 0;
  ps2_data.norm_PS_LY = 0;
  ps2_data.norm_PS_RX = 0;
  ps2_data.norm_PS_RY = 0;
  PS2_initGPIO();
  PS2_ShortPoll();
  PS2_ShortPoll();
  PS2_ShortPoll();
  PS2_EnterConfing();     // 进入配置模式
  PS2_TurnOnAnalogMode(); // “红绿灯”配置模式，并选择是否保存
  PS2_VibrationMode();    // 开启震动模式
  PS2_ExitConfing();      // 完成并保存配置
}