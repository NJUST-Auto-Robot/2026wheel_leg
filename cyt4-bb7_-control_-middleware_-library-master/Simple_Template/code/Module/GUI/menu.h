/*
 * @Author: JFeng 2834294740@qq.com
 * @Date: 2023-07-02 23:52:10
 * @LastEditors: JFeng 2834294740@qq.com
 * @LastEditTime: 2023-07-24 22:33:24
 * @FilePath: \MY_GUI_RTOS\HARDWARE\menu\menu.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MENU_H__
#define __MENU_H__

#include "Drivers/Key/key.hpp"
#include "Drivers/OLED/drv-u8g2.hpp"

#define FirstLine 9 // 第一行的y坐标是9
#define FirstPos 0
#define Font_Size 12

#define LINE_MAX 48
#define LINE_MIN 12
#define Max_Visible_Number 4

#define Menu_Up 0
#define Menu_Down 1
#define MENU_RUN 0
#define APP_RUN 1

#define MENU_MOVE 0
// 光标静止
#define CURSOR_STATIC 1

#define Head_To_Tail // 首尾链接

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MenuPage *xpMenu;
typedef struct Item *xpItem;

// 页链表元素,有PageName,长度(Item的数目),父Item,第一个Item(头Item),和最后一个(尾)Item
typedef struct MenuPage {
  const char *PageName;
  uint8_t len;
  xpItem ParentItem; // 跳转而来的父Item
  xpItem itemHead;
  xpItem itemTail;
  xpItem ChosenItem; // 跳转离开的被选择Item
} xMenu;

// Item链表,有Item名,Item顺序号,指向所在的MenuPage的指针,指向所引导的MenuPage的指针,上一个Item,下一个Item,以及回调函数[类型为void(void)]
typedef struct Item {
  char *itemName;
  uint8_t Number;
  xpMenu location;
  xpMenu JumpPage;
  xpItem lastItem;
  xpItem nextItem;
  uint8_t is_return;
  void (*Item_function)();
} xItem;

extern u8g2_t u8g2;
extern xItem ApplicationItem_MainPage;

void Menu_Team(void);
void AddPage(const char *name, xpMenu page);
void AddItem(char *Name, xpItem item, xpMenu LocalPage, xpMenu NextPage);

int8_t Line(uint8_t AllTime, uint8_t Time_Now, int8_t Tgt, int8_t Now);
void DialogScale_Show(u8g2_t *u8g2, uint16_t x, uint16_t y, uint16_t Tgt_w,
                      uint16_t Tgt_h);

void Draw_DialogBox(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w,
                    u8g2_uint_t h);
void Draw_DialogRBox(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w,
                     u8g2_uint_t h, u8g2_uint_t r);

void Menu_Task_Run();
void Menu_Init(void);

#ifdef __cplusplus
}
#endif

#endif
