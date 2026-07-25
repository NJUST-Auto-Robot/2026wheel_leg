#include "MachineVision/find_center_line.h"

Line_Struct line;


void find_center_line(uint8_t* image, uint8_t width, uint8_t height, Line_Struct*Line, uint8_t max_threshold, uint8_t min_threshold)
{
  //从下往上自79至49逐行、由中间向两边逐点扫描矩形侧边找中线
  for(int16_t y = 79,  i = 0; y >= 49; y--, i++)
  {
    for(int16_t x_left = 93; x_left >= 0; x_left--)
    {
      if(image[y * width + x_left] >= min_threshold && image[y * width + x_left] <= max_threshold) 
      {
        Line->center_line_left_x[i] = x_left;
        break;
      }
      else
      {
        Line->center_line_left_x[i] = 0;
      }
    }
    
    for(int16_t x_right = 94; x_right < 188; x_right++)
    {
      if(image[y * width + x_right] >= min_threshold && image[y * width + x_right] <= max_threshold) 
      {
        Line->center_line_right_x[i] = x_right;
        break;
      }
      else
      {
        Line->center_line_right_x[i] = 188;
      }
    }
    
    Line->center_line_center_x[i] = (Line->center_line_left_x[i] + Line->center_line_right_x[i]) / 2;
  }
  //从上往下逐行判断x=94——中线的顶点，此点y值用于判断是否行至终点
  for(int16_t y = 100; y > 0; y--)
  {
    if(image[y * width + 94] >= min_threshold && image[y * width + 94] <= max_threshold) 
    {
      Line->top_point_y = y;
      break;
    }
    else
    {
      Line->top_point_y = 0;
    }
  }
}

void draw_center_line(uint8_t* image, uint8_t width, uint8_t height, Line_Struct*Line)
{
  for(int16_t y = 79,  i = 0; y >= 49; y--, i++)
  {
    image[y * width + Line->center_line_right_x[i]] = 255;
    image[y * width + Line->center_line_left_x[i]] = 255;
    image[y * width + Line->center_line_center_x[i]] = 255;
  }
  
}