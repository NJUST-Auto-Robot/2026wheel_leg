/**
 * @file Key.hpp
 * @author Meng Yang (2895422061@qq.com)
 * @brief 按键消抖+驱动库
 * @version 0.1
 * @date 2024-07-08
 *
 * @copyright Copyright Meng Yang(c) 2024
 *
 */
#pragma once
#include "stdint.h"
#include "zf_common_headfile.h"
#define KEY_U P13_1
#define KEY_D P13_4
#define KEY_L P13_2
#define KEY_R P13_0
#define KEY_CENTER P13_3
typedef enum {
  KEY_RELEASED,
  KEY_CHECK,
  KEY_PRESSED,
  KEY_USED
} KeyState_TypeDef;
typedef enum KEY_STATE_Enumdef_ {
  USELESS = 0,
  RIGHT = 1,
  UP = 2,
  DOWN = 3,
  KEY1 = 4,
  LEFT = 5,
  MID = 6,
  KEY2 = 7
} KEY_STATE_Enumdef;
#ifdef __cplusplus
class Key {
private:
  KeyState_TypeDef KeyStage;

public:
  Key(void) {}
  ~Key(void) {}
  inline void KeyUpdate(uint8_t key_value) {
    switch (KeyStage) {
    case KEY_RELEASED:
      if (key_value == 0) { // 按压低电平有效
        KeyStage = KEY_CHECK;
      }
      break;
    case KEY_CHECK:
      if (key_value == 0) {
        KeyStage = KEY_PRESSED;
      } else {
        KeyStage = KEY_RELEASED;
      }
      break;
    case KEY_PRESSED:
    case KEY_USED:
      if (key_value == 1) {
        KeyStage = KEY_RELEASED;
      }
      break;
    default:
      break;
    }
  }
  inline bool GetKeyState(void) { return KeyStage == KEY_PRESSED; }
  inline void Use() { KeyStage = KEY_USED; }
};

extern "C" {
#endif
void Key_Init(void);
void Switch_Update();
void SwitchUse();
extern KEY_STATE_Enumdef key_state;
extern KEY_STATE_Enumdef last_key_state;

#ifdef __cplusplus
}
#endif