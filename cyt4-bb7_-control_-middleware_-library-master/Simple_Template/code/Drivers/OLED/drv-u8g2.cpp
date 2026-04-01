#include "drv-u8g2.hpp"
/**
 * @brief 初始化屏幕SPI外设
 *
 */
void U8G2PeripheralInit() {
  spi_init(OLED_SPI, SPI_MODE0, OLED_SPI_SPEED, OLED_D0_PIN, OLED_D1_PIN,
           OLED_D1_PIN_IN, SPI_CS_NULL);
  gpio_init(OLED_RES_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(OLED_DC_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
  gpio_init(OLED_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                               void *arg_ptr) {
  switch (msg) {
  case U8X8_MSG_BYTE_SEND: /*通过SPI发送arg_int个字节数据*/
    spi_write_8bit_array(OLED_SPI, (uint8_t *)arg_ptr, arg_int);
    break;
  case U8X8_MSG_BYTE_INIT: /*初始化函数*/
    break;
  case U8X8_MSG_BYTE_SET_DC: /*设置DC引脚,表明发送的是数据还是命令*/
    gpio_set_level(OLED_DC_PIN, arg_int);

    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
    u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO,
                            u8x8->display_info->post_chip_enable_wait_ns, NULL);
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO,
                            u8x8->display_info->pre_chip_disable_wait_ns, NULL);
    u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
    break;
  default:
    return 0;
  }
  return 1;
}
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
                                  U8X8_UNUSED uint8_t msg,
                                  U8X8_UNUSED uint8_t arg_int,
                                  U8X8_UNUSED void *arg_ptr) {
  switch (msg) {
  case U8X8_MSG_GPIO_AND_DELAY_INIT: /*delay和GPIO的初始化，在main中已经初始化完成了*/
    break;

  case U8X8_MSG_DELAY_MILLI: /*延时函数*/
    vTaskDelay(arg_int);
    break;

  case U8X8_MSG_GPIO_CS: /*片选信号*/
    gpio_set_level(OLED_CS_PIN, arg_int);
    break;

  case U8X8_MSG_GPIO_DC:
    gpio_set_level(OLED_DC_PIN, arg_int);
    break;

  case U8X8_MSG_GPIO_RESET:
    break;
  }
  return 1;
}
void u8g2Init(u8g2_t *u8g2) {
  /*
U8G2_R0： 顺时针旋转0度
U8G2_R1： 顺时针旋转90度
U8G2_R2： 顺时针旋转180度
U8G2_R3： 顺时针旋转270度
U8G2_MIRROR：U8G2_R0的镜像
  */
  u8g2_Setup_ssd1306_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi,
                                     u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(
      u8g2); // 根据所选的芯片进行初始化工作，初始化完成后，显示器处于关闭状态
  u8g2_SetPowerSave(u8g2, 0); // 打开显示器
  u8g2_ClearBuffer(u8g2);

  u8g2_DrawLine(u8g2, 0, 0, 127, 63);
  u8g2_DrawLine(u8g2, 127, 0, 0, 63);
  u8g2_SendBuffer(u8g2);
}
/** 画进度条 */
void Draw_Process(u8g2_t *u8g2) {
  for (size_t i = 10; i <= 80; ++i) {
    u8g2_ClearBuffer(u8g2);

    char buff[6];
    int hundred = (int)(i / 80.0 * 100) / 100 %
                  10; // 将[80范围]等比例映射到[100范围],取百位
    int quotient = (int)(i / 80.0 * 100) / 10 %
                   10; // 将[80范围]等比例映射到[100范围],取十位
    int remainder =
        (int)(i / 80.0 * 100) % 10; // 将[80范围]等比例映射到[100范围],取个位
    buff[0] = hundred + '0';
    buff[1] = quotient + '0';
    buff[2] = remainder + '0';
    buff[3] = '%';
    buff[4] = '\0';

    u8g2_SetFont(u8g2, u8g2_font_profont15_mf);
    u8g2_DrawStr(u8g2, 32, 16, "S-C-U-T"); // 写一个字符串

    u8g2_SetFont(u8g2, u8g2_font_profont12_mf);
    u8g2_DrawStr(u8g2, 100, 41,
                 buff); // 写百分比的字符串(感觉可以用sprintf来改进表达)

    u8g2_DrawRBox(u8g2, 16, 32, i, 10, 4); // 画实心的圆角矩形
    u8g2_DrawRFrame(u8g2, 16, 32, 80, 10,
                    4); // 画空心的圆角矩形(80个像素点为总长度)

    u8g2_SendBuffer(u8g2);
    vTaskDelay(10);
  }
}
void draw(u8g2_t *u8g2) {
  u8g2_ClearBuffer(u8g2);

  u8g2_SetFontMode(u8g2, 1);              /*字体模式选择*/
  u8g2_SetFontDirection(u8g2, 0);         /*字体方向选择*/
  u8g2_SetFont(u8g2, u8g2_font_inb24_mf); /*字库选择*/
  u8g2_DrawStr(u8g2, 0, 20, "U");

  u8g2_SetFontDirection(u8g2, 1);
  u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
  u8g2_DrawStr(u8g2, 21, 8, "8");

  u8g2_SetFontDirection(u8g2, 0);
  u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
  u8g2_DrawStr(u8g2, 51, 30, "g");
  u8g2_DrawStr(u8g2, 67, 30, "\xb2");

  u8g2_DrawHLine(u8g2, 2, 35, 47);
  u8g2_DrawHLine(u8g2, 3, 36, 47);
  u8g2_DrawVLine(u8g2, 45, 32, 12);
  u8g2_DrawVLine(u8g2, 46, 33, 12);

  u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
  u8g2_DrawStr(u8g2, 1, 54, "github.com/olikraus/u8g2");
  u8g2_SendBuffer(u8g2);
}