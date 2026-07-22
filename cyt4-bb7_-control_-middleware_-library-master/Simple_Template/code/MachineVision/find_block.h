#ifndef _FIND_BLOCK_H_
#define _FIND_BLOCK_H_

#include "zf_common_typedef.h"

typedef struct
{
  uint16_t front;
  uint16_t rear;
  
  uint8_t label[120][188];
  uint8_t queue_x[120 * 188];
  uint8_t queue_y[120 * 188];

  uint8_t block_num;
  uint32_t cx[100];
  uint32_t pixel[100];
  uint32_t cy[100];
  uint16_t min_x[100];
  uint16_t min_y[100];
  uint16_t max_x[100];
  uint16_t max_y[100];
}BLOCK;

extern BLOCK block;

void Block_Init(BLOCK* Block);
void Find_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block);
void Find_Block_Pro(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block, uint8_t max_threshold, uint8_t min_threshold);
void Find_Block_Pro_Max(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block, uint8_t max_threshold, uint8_t min_threshold, uint8_t roi_x, uint8_t roi_y, uint8_t roi_w, uint8_t roi_h);
void Draw_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block);
void Draw_Max_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block);
void Draw_Merge_Block(uint8_t* image, uint8_t width, uint8_t height, BLOCK* Block);

#endif 
