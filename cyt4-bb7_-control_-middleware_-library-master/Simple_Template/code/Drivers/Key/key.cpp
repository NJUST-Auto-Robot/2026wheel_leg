#include "key.hpp"
Key Up, Down, Left, Right, Center;
KEY_STATE_Enumdef key_state;
KEY_STATE_Enumdef last_key_state;
/**
 * @brief 初始化按键外设
 *
 */
void Key_Init(void) {
  gpio_init(KEY_U, GPI, 0, GPI_FLOATING_IN);
  gpio_init(KEY_D, GPI, 0, GPI_FLOATING_IN);
  gpio_init(KEY_L, GPI, 0, GPI_FLOATING_IN);
  gpio_init(KEY_R, GPI, 0, GPI_FLOATING_IN);
  gpio_init(KEY_CENTER, GPI, 0, GPI_FLOATING_IN);
}
/**
 * @brief 读取按键状态并更新按键状态变量
 *
 */
void Switch_Update() {
  Up.KeyUpdate(gpio_get_level(KEY_U));
  Down.KeyUpdate(gpio_get_level(KEY_D));
  Left.KeyUpdate(gpio_get_level(KEY_L));
  Right.KeyUpdate(gpio_get_level(KEY_R));
  Center.KeyUpdate(gpio_get_level(KEY_CENTER));
  if (Up.GetKeyState()) {
    key_state = UP;
  } else if (Down.GetKeyState()) {
    key_state = DOWN;
  } else if (Left.GetKeyState()) {
    key_state = LEFT;
  } else if (Right.GetKeyState()) {
    key_state = RIGHT;
  } else if (Center.GetKeyState()) {
    key_state = MID;
  } else {
    key_state = USELESS;
  }
}
void SwitchUse() {
  switch (key_state) {
  case UP:
    Up.Use();
    break;
  case DOWN:
    Down.Use();
    break;
  case LEFT:
    Left.Use();
    break;
  case RIGHT:
    Right.Use();
    break;
  case MID:
    Center.Use();
    break;
  default:
    break;
  }
  key_state = USELESS;
}
