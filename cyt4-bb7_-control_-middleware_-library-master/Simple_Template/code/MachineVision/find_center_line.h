#ifndef _FIND_CENTER_LINE_H_
#define _FIND_CENTER_LINE_H_

#include "zf_common_typedef.h"

typedef struct
{
  int16_t center_line_right_x[80];
  int16_t center_line_left_x[80];
  int16_t center_line_center_x[80];
  int16_t top_point_y;
}Line_Struct;

extern Line_Struct line;

void find_center_lin(uint8_t* image, uint8_t width, uint8_t height, Line_Struct*Line, uint8_t max_threshold, uint8_t min_threshold);

#endif 
