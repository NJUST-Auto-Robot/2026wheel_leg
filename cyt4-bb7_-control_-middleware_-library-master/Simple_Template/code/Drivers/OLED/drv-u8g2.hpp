/**
 * @file drv-u8g2.hpp
 * @author Meng Yang (2895422061@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-07-08
 *
 * @copyright Copyright Meng Yang(c) 2024
 *
 */
#pragma once
#include "Utility/u8g2/u8g2.h"
#include "zf_common_headfile.h"
/***********OLED接口配置********** */
#define OLED_SPI_SPEED (60 * 1000 * 1000) // 硬件 SPI 速率
#define OLED_SPI (SPI_1)                  // 硬件 SPI 号
#define OLED_D0_PIN (SPI1_CLK_P12_2)      // 硬件 SPI SCK 引脚
#define OLED_D1_PIN (SPI1_MOSI_P12_1)     // 硬件 SPI MOSI 引脚
#define OLED_D1_PIN_IN                                                         \
  (SPI_MISO_NULL) // 定义SPI_MISO引脚
                  // OLED没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
#define OLED_RES_PIN (P12_5) // 液晶复位引脚定义
#define OLED_DC_PIN (P12_4)  // 液晶命令位引脚定义
#define OLED_CS_PIN (P12_3)  // CS 片选引脚
#ifdef __cplusplus
extern "C" {
#endif
void u8g2Init(u8g2_t *u8g2);
void U8G2PeripheralInit();
void draw(u8g2_t *u8g2);
void Draw_Process(u8g2_t *u8g2);
#ifdef __cplusplus
}
#endif