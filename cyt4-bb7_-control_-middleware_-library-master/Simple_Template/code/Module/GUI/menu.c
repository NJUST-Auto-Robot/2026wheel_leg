/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-07
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-07
 * @FilePath: menu.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
#include "menu.h"
#include "GUI_Fun_Exporter.hpp"
u8g2_t u8g2;
void placeholder_fun(void) {}
// 页面状态
uint8_t Page_State = 0;
// 1为白天模式，0为黑夜模式(背景颜色)
uint8_t BgColor = 0x00;
uint8_t Dialog_Time = 70;

// 选项缓动动画持续时间（次数）
uint8_t Options_Time = 30;
/* Page*/
// MenuPage结构体成员,分别代表了不同的Page
xMenu MainPage; // 一级Page
xMenu ModeSubPage, DisplaySubPage, TargetPointsOPSubPage, GnssParametersSubPage,
    SoundParametersSubPage, ClosedSpdParametersSubPage; // 二级Page

/* item */
// MainPage的item
struct {
  xItem Mode;
  xItem Display;
  xItem TargetPointsOP;
  xItem GNSSParameters;
  xItem SoundParameters;
  xItem ClosedSpdParameters;
} MainPageItem = {0};
// 修改模式页面的item
struct {
  xItem Indoor;
  xItem Outdoor;
  xItem ReverseOn;
  xItem ReverseOff;
  xItem SpdClosedOn;
  xItem SpdClosedOff;
  xItem Return;
} ModePageItem = {0};
// 信息显示页面的item
struct {
  xItem GNSS;
  xItem Wireless;
  xItem CarStatus;
  xItem Return;
} DisplayPageItem = {0};

// 目标点编辑页面的item
struct {
  xItem CollectPoints;
  xItem ClearPoints;
  xItem Return;
} TargetPointsOPPageItem = {0};

// GPS循迹改参页面的item
struct {
  xItem ChangeGnssP;
  xItem ChangeGnssI;
  xItem ChangeGnssD;
  xItem ChangeGnssL;
  xItem ChangeGnssS;
  xItem ChangeGnssDIS;
  xItem Return;
} GnssParametersPageItem = {0};
// 声音循迹改参页面的item
struct {
  xItem ChangeSoundP;
  xItem ChangeSoundI;
  xItem ChangeSoundD;
  xItem ChangeSoundL;
  xItem ChangeSoundS;
  xItem ChangeSoundR;
  xItem ChangeSoundDIS;
  xItem Return;
} SoundParametersPageItem = {0};

struct {
  xItem ChangeClosedSpdP;
  xItem ChangeClosedSpdI;
  xItem ChangeClosedSpdD;
  xItem ChangeClosedSpdL;
  xItem ChangeClosedSpdT;
  xItem Return;
} ClosedSpdParametersPageItem = {0};
// char **ModeitemName = &(SettingPageItem.Mode.itemName);

xpItem handle_item = &MainPageItem.Mode; // 此时处理的Item;
void (*App_Function)();

extern uint8_t Dialog_Time;
extern uint8_t BgColor;
extern uint8_t Options_Time;
extern KEY_STATE_Enumdef last_key_state;

static uint8_t ui_disapper(uint8_t disapper);
// static void Draw_Process(void);
static void Draw_Menu(uint8_t pos, xpMenu Page, uint8_t LineSpacing,
                      xpItem now_item, xpItem next_item);
static void Draw_Page(uint8_t pos, xpMenu Page, uint8_t LineSpacing,
                      xpItem now_item, xpItem next_item);
static void Draw_OptionPlace(uint8_t now_time, xpItem now_item,
                             xpItem next_item);

/**
 * @brief 加载用户回调函数
 *
 */
void App_Function_Loading(void) {
  ModePageItem.Indoor.Item_function = IndoorModeCB;
  ModePageItem.Outdoor.Item_function = OutdoorModeCB;
  ModePageItem.ReverseOn.Item_function = ReverseOnCB;
  ModePageItem.ReverseOff.Item_function = ReverseOffCB;
  ModePageItem.SpdClosedOn.Item_function = SpdClosedOnCB;
  ModePageItem.SpdClosedOff.Item_function = SpdClosedOffCB;

  DisplayPageItem.GNSS.Item_function = GNSSDispCB;
  DisplayPageItem.Wireless.Item_function = WirelessDispCB;
  DisplayPageItem.CarStatus.Item_function = CarStatCB;

  TargetPointsOPPageItem.CollectPoints.Item_function = CollectPointsCB;
  TargetPointsOPPageItem.ClearPoints.Item_function = ClearPointsCB;

  GnssParametersPageItem.ChangeGnssP.Item_function = GNSSPCB;
  GnssParametersPageItem.ChangeGnssI.Item_function = GNSSICB;
  GnssParametersPageItem.ChangeGnssD.Item_function = GNSSDCB;
  GnssParametersPageItem.ChangeGnssL.Item_function = GNSSLCB;
  GnssParametersPageItem.ChangeGnssS.Item_function = GNSSSCB;
  GnssParametersPageItem.ChangeGnssDIS.Item_function = GNSSDISCB;

  SoundParametersPageItem.ChangeSoundP.Item_function = SoundPCB;
  SoundParametersPageItem.ChangeSoundI.Item_function = SoundICB;
  SoundParametersPageItem.ChangeSoundD.Item_function = SoundDCB;
  SoundParametersPageItem.ChangeSoundL.Item_function = SoundLCB;
  SoundParametersPageItem.ChangeSoundS.Item_function = SoundSCB;
  SoundParametersPageItem.ChangeSoundR.Item_function = SoundRCB;
  SoundParametersPageItem.ChangeSoundDIS.Item_function = SoundDISCB;

  ClosedSpdParametersPageItem.ChangeClosedSpdP.Item_function = ClosedSpdPCB;
  ClosedSpdParametersPageItem.ChangeClosedSpdI.Item_function = ClosedSpdICB;
  ClosedSpdParametersPageItem.ChangeClosedSpdD.Item_function = ClosedSpdDCB;
  ClosedSpdParametersPageItem.ChangeClosedSpdL.Item_function = ClosedSpdLCB;
  ClosedSpdParametersPageItem.ChangeClosedSpdT.Item_function = ClosedSpdTCB;
}
/**
 * @brief 菜单链表注册函数
 *
 */
void Menu_Team() {
  MainPage.ParentItem = NULL; // 主页没有父Item

  AddPage("[MainPage]", &MainPage);
  AddItem(" -Mode", &MainPageItem.Mode, &MainPage, &ModeSubPage);
  AddItem(" -Display", &MainPageItem.Display, &MainPage, &DisplaySubPage);
  AddItem(" -Targets", &MainPageItem.TargetPointsOP, &MainPage,
          &TargetPointsOPSubPage);
  AddItem(" -GNSSPara", &MainPageItem.GNSSParameters, &MainPage,
          &GnssParametersSubPage);
  AddItem(" -SoundPara", &MainPageItem.SoundParameters, &MainPage,
          &SoundParametersSubPage);
  AddItem(" -ClosedSpdPara", &MainPageItem.ClosedSpdParameters, &MainPage,
          &ClosedSpdParametersSubPage);

  /*MainPage - Application*/
  AddPage("[Mode]", &ModeSubPage);
  AddItem(" -Indoor", &ModePageItem.Indoor, &ModeSubPage, NULL);
  AddItem(" -Outdoor", &ModePageItem.Outdoor, &ModeSubPage, NULL);
  AddItem(" -ReverseOn", &ModePageItem.ReverseOn, &ModeSubPage, NULL);
  AddItem(" -ReverseOff", &ModePageItem.ReverseOff, &ModeSubPage, NULL);
  AddItem(" -SpdClosedOn", &ModePageItem.SpdClosedOn, &ModeSubPage, NULL);
  AddItem(" -SpdClosedOff", &ModePageItem.SpdClosedOff, &ModeSubPage, NULL);
  AddItem(" -Return", &ModePageItem.Return, &ModeSubPage,
          ModeSubPage.ParentItem->location);
  ModePageItem.Return.is_return = 1; // 标记为return相关Item

  /*MainPage - Application - System*/
  AddPage("[Display]", &DisplaySubPage);
  AddItem(" -GNSS", &DisplayPageItem.GNSS, &DisplaySubPage, NULL);
  AddItem(" -Wireless", &DisplayPageItem.Wireless, &DisplaySubPage, NULL);
  AddItem(" -CarStatus", &DisplayPageItem.CarStatus, &DisplaySubPage, NULL);
  AddItem(" -Return", &DisplayPageItem.Return, &DisplaySubPage,
          DisplaySubPage.ParentItem->location);
  DisplayPageItem.Return.is_return = 1; // 标记为return相关Item

  /*MainPage - Application - Games*/
  AddPage("Targets", &TargetPointsOPSubPage);
  AddItem(" -Collect", &TargetPointsOPPageItem.CollectPoints,
          &TargetPointsOPSubPage, NULL);
  AddItem(" -Clear", &TargetPointsOPPageItem.ClearPoints,
          &TargetPointsOPSubPage, NULL);
  AddItem(" -Return", &TargetPointsOPPageItem.Return, &TargetPointsOPSubPage,
          TargetPointsOPSubPage.ParentItem->location);
  TargetPointsOPPageItem.Return.is_return = 1; // 标记为return相关Item

  /*MainPage - Parameters*/
  AddPage("[GPSParameters]", &GnssParametersSubPage);
  AddItem(" -Kp", &GnssParametersPageItem.ChangeGnssP, &GnssParametersSubPage,
          NULL);
  AddItem(" -Ki", &GnssParametersPageItem.ChangeGnssI, &GnssParametersSubPage,
          NULL);
  AddItem(" -Kd", &GnssParametersPageItem.ChangeGnssD, &GnssParametersSubPage,
          NULL);
  AddItem(" -lim", &GnssParametersPageItem.ChangeGnssL, &GnssParametersSubPage,
          NULL);
  AddItem(" -spd", &GnssParametersPageItem.ChangeGnssS, &GnssParametersSubPage,
          NULL);
  AddItem(" -to snd dis", &GnssParametersPageItem.ChangeGnssDIS,
          &GnssParametersSubPage, NULL);
  AddItem(" -Return", &GnssParametersPageItem.Return, &GnssParametersSubPage,
          GnssParametersSubPage.ParentItem->location);
  GnssParametersPageItem.Return.is_return = 1; // 标记为return相关Item

  AddPage("[SoundParameters]", &SoundParametersSubPage);
  AddItem(" -Kp", &SoundParametersPageItem.ChangeSoundP,
          &SoundParametersSubPage, NULL);
  AddItem(" -Ki", &SoundParametersPageItem.ChangeSoundI,
          &SoundParametersSubPage, NULL);
  AddItem(" -Kd", &SoundParametersPageItem.ChangeSoundD,
          &SoundParametersSubPage, NULL);
  AddItem(" -lim", &SoundParametersPageItem.ChangeSoundL,
          &SoundParametersSubPage, NULL);
  AddItem(" -spd", &SoundParametersPageItem.ChangeSoundS,
          &SoundParametersSubPage, NULL);
  AddItem(" -rvs", &SoundParametersPageItem.ChangeSoundR,
          &SoundParametersSubPage, NULL);
  AddItem(" -to gps dis", &SoundParametersPageItem.ChangeSoundDIS,
          &SoundParametersSubPage, NULL);
  AddItem(" -Return", &SoundParametersPageItem.Return, &SoundParametersSubPage,
          SoundParametersSubPage.ParentItem->location);
  SoundParametersPageItem.Return.is_return = 1; // 标记为return相关Item

  AddPage("[ClosedSpdParameters]", &ClosedSpdParametersSubPage);
  AddItem(" -Kp", &ClosedSpdParametersPageItem.ChangeClosedSpdP,
          &ClosedSpdParametersSubPage, NULL);
  AddItem(" -Ki", &ClosedSpdParametersPageItem.ChangeClosedSpdI,
          &ClosedSpdParametersSubPage, NULL);
  AddItem(" -Kd", &ClosedSpdParametersPageItem.ChangeClosedSpdD,
          &ClosedSpdParametersSubPage, NULL);
  AddItem(" -lim", &ClosedSpdParametersPageItem.ChangeClosedSpdL,
          &ClosedSpdParametersSubPage, NULL);
  AddItem(" -tgt", &ClosedSpdParametersPageItem.ChangeClosedSpdT,
          &ClosedSpdParametersSubPage, NULL);
  AddItem(" -Return", &ClosedSpdParametersPageItem.Return,
          &ClosedSpdParametersSubPage,
          ClosedSpdParametersSubPage.ParentItem->location);
  ClosedSpdParametersPageItem.Return.is_return = 1; // 标记为return相关Item
}

void Menu_Task_Run() {
  static uint8_t MENU_STATE = MENU_RUN;
  uint8_t disapper = 1; // 渐变函数消失速度，越小越慢，最大值为8

  if (key_state !=
      USELESS //			&& key_state != last_key_state\

  ) {
    switch ((uint8_t)key_state) {
    case KEY2:
    case MID:
      SwitchUse();
      if (handle_item->JumpPage == NULL) { // 条件:不存在跳转页
        MENU_STATE = APP_RUN;              // 菜单状态为执行相关函数
        ui_disapper(disapper);             // ui渐变消失
        App_Function = handle_item->Item_function; // 修改当下处理的函数
      } else {                                     // 条件:存在跳转页
        MENU_STATE = MENU_RUN; // 菜单状态为菜单变换

        /*当前页面逐步消失*/
        for (size_t i = 0; i < 8; i++) {
          disapper = ui_disapper(disapper);
        }
        handle_item->location->ChosenItem =
            handle_item; // 记下当前页是哪一个Item被选择了
        if (!handle_item->is_return) // 如果不是return相关Item
        {
          Draw_Menu(
              FirstPos, handle_item->JumpPage, Font_Size, handle_item,
              handle_item->JumpPage
                  ->itemHead); // 画下一个页面的每一个Item以及光标行+进度条
          handle_item = handle_item->JumpPage
                            ->itemHead; // 处理中的Item变为跳转页面的第一个Item
        } else {                        // 如果是return相关Item
          // 回到上一次被选择的Item上
          Draw_Menu(
              FirstPos, handle_item->JumpPage, Font_Size, handle_item,
              handle_item->JumpPage
                  ->ChosenItem); // 画下一个页面的每一个Item以及光标行+进度条
          handle_item = handle_item->JumpPage->ChosenItem;
        }
      }
      break;
    case UP:
      SwitchUse();
      Draw_Menu(
          FirstPos, handle_item->location, Font_Size, handle_item,
          handle_item->lastItem); // 画当前页面的每一个Item以及光标行+进度条
      handle_item = handle_item->lastItem; // 处理中的Item变为上一个Item
      MENU_STATE = MENU_RUN;               // 标记当前为菜单移动状态
      break;
    case DOWN:
      SwitchUse();
      Draw_Menu(
          FirstPos, handle_item->location, Font_Size, handle_item,
          handle_item->nextItem); // 画当前页面的每一个Item以及光标行+进度条
      handle_item = handle_item->nextItem; // 处理中的Item变为上下一个Item
      MENU_STATE = MENU_RUN;               // 标记当前为菜单移动状态
      break;
    default:
      break;
    }
  }

  if (MENU_STATE == APP_RUN) { // 条件:要执行函数
    if (App_Function != NULL)
      (*App_Function)();   // 空指针判断,执行函数
    MENU_STATE = MENU_RUN; // 状态为页面变化
    Draw_Menu(
        FirstPos, handle_item->location, Font_Size, handle_item,
        handle_item); // 画当前页面的每一个Item以及光标行+进度条(感觉有些多余)
  }
  last_key_state = key_state;
}

void Menu_Init(void) {
  /* 下面这句在call之前实现 */
  //    u8g2Init(&u8g2);
  Menu_Team();         // 注册总链表
  Draw_Process(&u8g2); // 画进度条
  Draw_Menu(FirstPos, &MainPage, Font_Size, &MainPageItem.Mode,
            &MainPageItem.Display); // 画MainPage的每一个Item以及光标行+进度条
  App_Function_Loading(); // 注册用户回调函数
}

/**
 * @brief
 * 加MenuPage,但这个函数只是给MenuPage起个名字和给空指针初始化,没有其他作为
 *
 * @param name MenuPage的称呼
 * @param page 所要加入的MenuPage
 */
void AddPage(const char *name, xpMenu page) {
  page->PageName = name;
  page->itemHead = NULL;
  page->itemTail = NULL;
}

/**
 * @brief 加入Item
 *
 * @param Name Item的名字
 * @param item 要插入的Item
 * @param LocalPage 目前所在的Page
 * @param nextpage 会引导前往的Page
 */
void AddItem(char *Name, xpItem item, xpMenu LocalPage, xpMenu nextpage) {
  item->itemName = Name;
  item->location = LocalPage;
  item->JumpPage = nextpage;
  /* 新建item的下一个肯定是null */
  item->nextItem = NULL;
  /* 如果可以跳转，那么此item是跳转页面的父级 */
  if (nextpage != NULL)
    nextpage->ParentItem = item;
  /* 链式结构创建item */
  if (LocalPage->itemHead == NULL) // 如果是第一个iTem
  {
    item->lastItem = item;
    LocalPage->itemHead = item;
    LocalPage->itemTail = item;
    LocalPage->len = 1;
  } else // 不是第一个item
  {
    item->lastItem = LocalPage->itemTail; // 新item的last指向Local的tailitem
    LocalPage->itemTail->nextItem = item; // 让尾巴的next指向新的item，连接起来
    LocalPage->itemTail = LocalPage->itemTail->nextItem; // 让尾巴指向新的item
    LocalPage->len++; // 当前页的长度加一
  }
  item->Number =
      LocalPage->len; // Item加在了当前页的最后,所以长度就是Item的顺序号
}

/**
 * @brief 画当前页面
 *
 * @param pos x坐标
 * @param Page 用户输入的Page,指的是当前页面
 * @param LineSpacing 每行的大小
 * @param now_item 用户输入的当前Item
 * @param next_item 用户输入的下一个Item
 */
void Draw_Page(uint8_t pos, xpMenu Page, uint8_t LineSpacing, xpItem now_item,
               xpItem next_item) {
  static int8_t first_line =
      FirstLine; // first
                 // line的值是9,这个是有符号整形,证明可以往负数发展,同时是静态变量,证明这个首行数据不丢失
  xpItem temp = Page->itemHead; // 记住了用户输入的Page的头Item

  if ((next_item == now_item->JumpPage->itemHead) &&
      next_item !=
          now_item) // 切换页面时变量初始化,条件:用户提供的下一个Item不重复 &&
                    // 点击了现在的Item并引导到了下一页的头Item(用户提供的下一个Item)
  {
    first_line =
        FirstLine; // 因为是点击进入了新页面,除了第一行的标题后面开始刷新
  } else if (next_item == now_item->JumpPage->ChosenItem) // 返回上一页面
  {
    first_line = FirstLine;
    // 如果返回的item的Number > 4，则不能从Page->itemHead开始
    while (next_item->Number - temp->Number >= 4) {
      temp = temp->nextItem;
    }
  } else if ((next_item->Number - now_item->Number > 0) &&
             Page_State == CURSOR_STATIC) { // 条件:Item序列号增大 && 光标静止
    Page_State = MENU_MOVE;                 // 页面要移动了
    if ((next_item->Number - now_item->Number) >
        (Page->len - Max_Visible_Number)) // 条件:Item序列号递增数 >
                                          // Page长度和总示长度(4)的差值
      first_line -= ((Page->len - Max_Visible_Number) *
                     Font_Size); // 除去不移动时的项目数(?)
    else
      first_line -= Font_Size; // 第一行的y坐标自减一个字体大小
  } else if ((next_item->Number - now_item->Number < 0) &&
             Page_State == CURSOR_STATIC) { // 条件:下一个Item在当前Item的前面
                                            // && 光标静止
    Page_State = MENU_MOVE; // 初于页面移动状态
    if ((now_item->Number - next_item->Number) >
        (Page->len - Max_Visible_Number)) // 条件: 用户输入的两个Item序列号差值
                                          // > Page长度和总示长度(4)的差值
      first_line += ((Page->len - Max_Visible_Number) *
                     Font_Size); // 除去不移动时的项目数
    else
      first_line += Font_Size; // 第一行自增一个字体大小
  }

  u8g2_DrawStr(&u8g2, pos, FirstLine,
               Page->PageName); // 在开头的y坐标写上Page的名
  for (size_t i = 1; i <= Page->len; i++) {
    if ((first_line + i * LineSpacing) > FirstLine)
      u8g2_DrawStr(&u8g2, pos, first_line + i * LineSpacing, temp->itemName);
    temp = temp->nextItem;
  }
}

/**
 * @brief 画页面的每一个Item,进度条,光标行
 *
 * @param pos 用户输入的位置值
 * @param Page 用户输入Page
 * @param LineSpacing 用户输入的行距
 * @param now_item 用户输入的当前Item
 * @param next_item 用户输入的下一个Item
 */
void Draw_Menu(uint8_t pos, xpMenu Page, uint8_t LineSpacing, xpItem now_item,
               xpItem next_item) {
  uint8_t cnt_time = 0;                                   // 一个计数变量
  uint8_t item_wide = strlen(now_item->itemName) * 6 + 4; // item名字的宽度
  static uint8_t item_line = LINE_MIN;
  static int8_t Tgt_line = 0;
  static uint8_t first = 0; // 初始状态

  u8g2_SetMaxClipWindow(
      &u8g2); // 窗口范围从左上角(x0,y0)到右下角(x1,y1),也就是我们绘制的内容只能在规范范围内显示,此函数用于取消此限制
  u8g2_SetFont(&u8g2, u8g2_font_profont12_mf); // 设置一个12的字体

  if (next_item == now_item->JumpPage->itemHead &&
      next_item != now_item) // 切换页面时变量初始化,条件: 将进行页面切换
  {
    item_line = LINE_MIN; // Item的这行的y坐标是最小y坐标
    Tgt_line = 0;         // 目标行初始化
    first = 0;            // 首次flag
    Page_State = 0;       // 页面状态初始化
  }

  /*条件: (两个Item序列号相等 && 第一次进入此函数) || 将要进行页面转换*/
  if ((next_item->Number - now_item->Number == 0 && first == 0) ||
      next_item == now_item->JumpPage->itemHead) {
    Tgt_line = LINE_MIN; // 目标行y坐标是最小
    first = 1;           // 首次进行flag置一
  } else if (next_item == now_item->JumpPage->ChosenItem) {
    Tgt_line = (next_item->Number * Font_Size);
    if (Tgt_line > LINE_MAX) // 防止光标溢出可视范围
    {
      Page_State = CURSOR_STATIC; // 光标来到了最下面,标记光标静止了
      Tgt_line = LINE_MAX;        // 光标行永远保持在下面
    }
  } else if (next_item->Number - now_item->Number >
             0) { // 下一个Item的序列号 > 现在Item的序列号
    Tgt_line += ((next_item->Number - now_item->Number) *
                 Font_Size); // 目标行就要往下移动差的几行
    if (Tgt_line > LINE_MAX) // 防止光标溢出可视范围
    {
      Page_State = CURSOR_STATIC; // 光标来到了最下面,标记光标静止了
      Tgt_line = LINE_MAX;        // 光标行永远保持在下面
    }
  } else if (next_item->Number - now_item->Number < 0) {
    Tgt_line -=
        ((now_item->Number - next_item->Number) * Font_Size); // 光标上移
    if (Tgt_line < LINE_MIN) // 防止光标溢出可视范围
    {
      Page_State = CURSOR_STATIC; // 光标静止
      Tgt_line = LINE_MIN;        // 光标静止在最上方
    }
  }
#ifdef Head_To_Tail // 判断是否开启了首尾相接
  Page->itemTail->nextItem = Page->itemHead;
  Page->itemHead->lastItem = Page->itemTail;
#else
  Page->itemTail->nextItem = Page->itemTail;
  Page->itemHead->lastItem = Page->itemHead;
#endif
  // 渐变实现进度栏,光标行
  do {
    u8g2_ClearBuffer(&u8g2);                         // 清除缓存内容
    cnt_time++;                                      // 计时累加
    u8g2_SetDrawColor(&u8g2, BgColor);               // 设置黑夜白天模式
    u8g2_DrawBox(&u8g2, 0, 0, 128, 64);              // 画一个填满的底框
    u8g2_SetDrawColor(&u8g2, BgColor ^ 0x01);        // 改变颜色
    Draw_OptionPlace(cnt_time, now_item, next_item); // 画光标的进度栏
    Draw_Page(pos, Page, LineSpacing, now_item,
              next_item); // 画页面,每一个Item都画出来
    u8g2_SetDrawColor(&u8g2, 2); // 开启异或模式,则不管底色是什么都能有显示
    item_line = Line(Options_Time, cnt_time, Tgt_line, item_line);
    item_wide =
        Line(Options_Time, cnt_time, strlen(next_item->itemName) * 6 + 4,
             item_wide); // 12字体的字,宽度为6,在左右空出4的间隙
    u8g2_DrawRBox(&u8g2, pos + 1, item_line - 1, item_wide, Font_Size, 4);
    u8g2_SendBuffer(&u8g2);
  } while (cnt_time < Options_Time);
}

/**
 * @brief 绘制右侧进度栏
 *
 * @param now_time 当前时间
 * @param now_item 当前的Item
 * @param next_item 下一个Item
 */
void Draw_OptionPlace(uint8_t now_time, xpItem now_item, xpItem next_item) {
  static uint8_t now_Y = 0;  // 现在的y坐标
  static uint8_t next_Y = 0; // 下一次的y坐标
  next_Y =
      (next_item->Number - 1) *
      (64 /
       next_item->location
           ->len); // 计算下一次的Y坐标,[将当前Item的顺序号~64]等比例映射到[总长度64]
  u8g2_DrawVLine(&u8g2, 122, 2, 64); // 划一根竖线,这个是右侧总进度栏
  for (size_t i = 0; i < next_item->location->len; i++) {
    u8g2_DrawHLine(&u8g2, 119, i * (64 / next_item->location->len) + 2,
                   6); // 画若干条横线,是当前Item的进度展示
  }
  now_Y =
      Line(Options_Time, now_time, next_Y, now_Y); // 按时间等比例映射了y坐标
  u8g2_DrawBox(&u8g2, 118, now_Y, 8,
               4); // 画一个实心矩形,应该是用来表示进度栏的
}

/**
 * @brief 渐变消失函数
 *
 * @param disapper
 * @return uint8_t
 */
uint8_t ui_disapper(uint8_t disapper) {
  short disapper_temp =
      0; // 用来返回传入的disappear最后的值,其实可以用传入disappear的指针来搞定这个事情
  int len =
      8 * u8g2_GetBufferTileHeight(&u8g2) *
      u8g2_GetBufferTileWidth(
          &u8g2); // //返回页面缓冲区的大小,等于调用u8g2_GetBufferSize(u8g2_t
                  // *u8g2),一个title就是8个像素点
  uint8_t *p = u8g2_GetBufferPtr(&u8g2); // titlebuff的指针
  if (BgColor == 0) { // 黑夜模式下,底色为0,故会随机将一些本来为1的地方改成0
    /*先进行移位运算,再进行按位运算,会从高位逐步改成底色0*/
    for (int i = 0; i < len; i++) {
      p[i] = p[i] & (rand() % 0xff) >> disapper;
    } // 根据bg的模式,逐个元素进行修改,生成[0~0xff-1]的随机数,随机将一些本来是1的位置零,然后右移若干位
  } else { // 白天模式下,底色为1,故将一些本来为0的地方改成1
    /*先进行移位运算,再进行按位运算,先进行>>disappear,再进行p[i] |
     * x,高位没有逐步改成底色1*/
    for (int i = 0; i < len; i++) {
      p[i] = p[i] | (rand() % 0xff) >> disapper;
    } // 根据bg的模式,逐个元素进行修改,生成[0~0xff-1]的随机数,随机将一些本来是0的位置1,然后右移若干位
  }
  disapper += 2; // 每次都会消失2位
  if (disapper >= 8) {
    disapper = 0;
  }                       // 最多8位,所以将8位置0
  u8g2_SendBuffer(&u8g2); // 绘制OLED
  disapper_temp = disapper;
  return disapper_temp;
}
/**
 * @brief
 * 线性增长函数用于坐标移动,可以理解成,将[当前值和目标值]等比例映射到了[当前时间和总时长]
 *
 * @param AllTime 总时长
 * @param Time_Now 当前时间
 * @param Tgt 目标值
 * @param Now 当前值
 * @return int8_t
 */
int8_t Line(uint8_t AllTime, uint8_t Time_Now, int8_t Tgt, int8_t Now) {
  return (Tgt - Now) * Time_Now / AllTime + Now; // return c * t / d + b;
}

/**
 * @brief
 * 描绘方形对话框,在一个空心矩形里面画一个实心矩形,实心矩形四边距离外面的空心矩形只有1个像素点的差距
 *
 * @param u8g2
 * @param x
 * @param y
 * @param w
 * @param h
 */
void Draw_DialogBox(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w,
                    u8g2_uint_t h) {
  u8g2_SetDrawColor(u8g2,
                    BgColor ^ 0x01); // 按位异或,若为0,则会点亮OLED,则为黑夜模式
  u8g2_DrawFrame(u8g2, x, y, w, h); // 画一个空心矩形
  u8g2_SetDrawColor(u8g2, BgColor); // 换成相反的颜色进行绘制
  u8g2_DrawBox(
      u8g2, x + 1, y + 1, w - 2,
      h - 2); // 画一个实心矩形,实心矩形四边距离外面的空心矩形只有1个像素点的差距
  u8g2_SetDrawColor(u8g2, BgColor ^ 0x01); // 又把配色换成最开始的配色
}

/**
 * @brief
 * 描绘圆角对话框,在一个空心矩形里面画一个实心矩形,实心矩形四边距离外面的空心矩形只有1个像素点的差距
 *
 * @param u8g2
 * @param x
 * @param y
 * @param w
 * @param h
 * @param r
 */
void Draw_DialogRBox(u8g2_t *u8g2, u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w,
                     u8g2_uint_t h, u8g2_uint_t r) {
  u8g2_SetDrawColor(u8g2, BgColor ^ 0x01);
  u8g2_DrawRFrame(u8g2, x, y, w, h, r);
  u8g2_SetDrawColor(u8g2, BgColor);
  u8g2_DrawRBox(u8g2, x + 1, y + 1, w - 2, h - 2, r);
  u8g2_SetDrawColor(u8g2, BgColor ^ 0x01);
}

/**
 * @brief 对话框出现函数
 *
 * @param u8g2 U8G2
 * @param x 初始位置x
 * @param y 初始位置y
 * @param Tgt_w 目标宽度
 * @param Tgt_h 目标高度
 */
void DialogScale_Show(u8g2_t *u8g2, uint16_t x, uint16_t y, uint16_t Tgt_w,
                      uint16_t Tgt_h) {
  uint8_t t = 0;
  uint16_t Init_w = 0, Init_h = 0;
  u8g2_ClearBuffer(u8g2);
  do {
    t++;
    Init_w = Line(Dialog_Time, t, Tgt_w, Init_w);
    Init_h = Line(Dialog_Time, t, Tgt_h, Init_h);
    Draw_DialogBox(u8g2, x, y, Init_w, Init_h);
    u8g2_SendBuffer(u8g2);
  } while (t < Dialog_Time);
}